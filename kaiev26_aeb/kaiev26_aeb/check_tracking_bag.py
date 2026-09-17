"""Offline PP/Stanley replay of existing cone paths and original vehicle state.

Copies the fusion visualization topics into a new MCAP and adds control results.
Recorded motion is fixed: this is command validation, never closed-loop tracking.
"""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from kaiev26_msgs.msg import ActuatorCommand
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .bag import OutputBag
from .messages import predicted_path_marker, stamp_ns
from .pursuit import Pursuit, PursuitConfig
from .stanley import Stanley, StanleyConfig


def mcap_path(path):
    p = Path(path).expanduser().resolve()
    if p.is_dir():
        files = list(p.glob('*.mcap'))
        if len(files) != 1:
            raise ValueError(f'Expected a single MCAP: {p}')
        p = files[0]
    return p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fusion', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--controller', choices=['pure_pursuit', 'stanley'], default='pure_pursuit')
    parser.add_argument('--controller-config', help='Optional controller YAML for tuning')
    parser.add_argument('--target-speed-kph', type=float)
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error('Output must be new')
    states, paths, gates = [], [], []
    with mcap_path(args.source).open('rb') as f:
        for schema, _, msg in make_reader(f).iter_messages(topics=['/vehicle/state']):
            state = deserialize_message(msg.data, get_message(schema.name))
            states.append((msg.log_time, state))
    if not states:
        parser.error('Original bag must contain /vehicle/state')
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = OutputBag(output)
    try:
        with mcap_path(args.fusion).open('rb') as f:
            for schema, channel, msg in make_reader(f).iter_messages():
                obj = deserialize_message(msg.data, get_message(schema.name))
                writer.write(channel.topic, obj, msg.log_time)
                if channel.topic == '/aeb/center_path':
                    paths.append((max(msg.log_time, stamp_ns(obj.header)), obj))
                elif channel.topic == '/aeb/red_gate':
                    gates.append((max(msg.log_time, stamp_ns(obj.header)), obj))
        for t, s in states:
            writer.write('/vehicle/state', s, t)
        paths.sort(key=lambda row: row[0])
        gates.sort(key=lambda row: row[0])
        is_stanley = args.controller == 'stanley'
        cls, cfg_cls = (Stanley, StanleyConfig) if is_stanley else (Pursuit, PursuitConfig)
        config_path = (Path(args.controller_config) if args.controller_config else
                       Path(get_package_share_directory('kaiev26_aeb')) / 'config/tracking.yaml')
        stem = 'stanley' if is_stanley else 'pursuit'
        p = yaml.safe_load(config_path.read_text())[f'cone_{stem}_node']['ros__parameters']
        config = cfg_cls(**{k: p.get(k, v) for k, v in vars(cfg_cls()).items()})
        if args.target_speed_kph is not None:
            config.target_speed_kph = args.target_speed_kph
        control = cls(config)
        pi, si, gi, last_path, last_state, last_gate = 0, 0, 0, None, None, None
        records = []
        start, end = min(paths[0][0], states[0][0]), max(paths[-1][0], states[-1][0]) + 1_000_000_000
        for t in range(start, end, 20_000_000):
            while pi < len(paths) and paths[pi][0] <= t:
                last_path = paths[pi][1]; pi += 1
            while si < len(states) and states[si][0] <= t:
                last_state = states[si][1]; si += 1
            while gi < len(gates) and gates[gi][0] <= t:
                last_gate = gates[gi][1]; gi += 1
            speed = float(last_state.speed_mps) if last_state else 0.0
            result = control.step(
                [(x.pose.position.x, x.pose.position.y) for x in last_path.poses] if last_path else [],
                speed, float(last_state.steering_rad) if last_state else 0.0,
                (t - stamp_ns(last_path.header))/1e9 if last_path else 1e6,
                (t - stamp_ns(last_state.header))/1e9 if last_state else 1e6, 0.02,
                frame=last_path.header.frame_id if last_path else 'base_link',
                estop=bool(last_state.estop_active) if last_state else False,
                reverse=last_state.gear != 0 if last_state else False,
                red_gate=([(p.position.x, p.position.y) for p in last_gate.poses]
                          if last_gate and last_gate.header.frame_id == 'base_link' else []),
                red_gate_stamp=stamp_ns(last_gate.header) if last_gate else None,
                red_gate_age=(t-stamp_ns(last_gate.header))/1e9 if last_gate else 1e6)
            command = ActuatorCommand()
            command.header.stamp.sec, command.header.stamp.nanosec = divmod(t, 1_000_000_000)
            command.header.frame_id = 'base_link'
            for k in ('speed_target_mps', 'steering_target_rad', 'brake_engage'):
                setattr(command, k, result[k])
            row = dict(result, source_time_s=(t-start)/1e9, speed_mps=speed,
                       evaluation='offline_fixed_recorded_motion')
            records.append(row)
            writer.write('/aeb/control_command', command, t)
            writer.write('/aeb/tracking_status', String(data=json.dumps(row)), t)
            marker = Marker()
            marker.header = command.header
            marker.ns, marker.id, marker.type = control.controller_name, 0, Marker.SPHERE
            marker.action = Marker.ADD if result['target_xy'] else Marker.DELETE
            marker.pose.orientation.w = 1.0
            if result['target_xy']:
                marker.pose.position.x, marker.pose.position.y = result['target_xy']
            marker.scale.x = marker.scale.y = marker.scale.z = 0.4
            marker.color.r = marker.color.g = marker.color.a = 1.0
            writer.write('/aeb/pursuit_target', marker, t)
            writer.write('/aeb/predicted_path', predicted_path_marker(
                result, control.cfg.wheelbase_m, command.header), t)
    finally:
        writer.close()
    active = [r for r in records if r['tracking']]
    report = dict(fusion=str(mcap_path(args.fusion)), source=str(mcap_path(args.source)),
                  evaluation='offline command calculation; not closed-loop tracking',
                  hz=50, samples=len(records), status_counts=dict(Counter(r['reason'] for r in records)),
                  controller=control.controller_name, controller_config=vars(control.cfg),
                  red_gate_messages=len(gates), red_stop_latched=control.red.latched,
                  max_command_kph=max(r['speed_target_mps'] for r in records)*3.6,
                  max_recorded_speed_kph=max(s.speed_mps for _, s in states)*3.6,
                  tracking_steering_abs_deg_p50_p95_max=(np.percentile(
                      [abs(r['steering_target_rad'])*180/np.pi for r in active], [50,95,100]).tolist() if active else []),
                  all_invalid_brake=all(r['brake_engage'] and r['speed_target_mps']==0 for r in records if not r['tracking']))
    (output/'tracking_metrics.json').write_text(json.dumps(report, indent=2))
    (output/'tracking.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
