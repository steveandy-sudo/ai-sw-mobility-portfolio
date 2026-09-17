"""Run the downloaded K-City cone-course tracking trial and save its evidence.

Gazebo-only fixture: places the car, enables the tracking node, measures GT
error and ends the run. It never supplies steering, a path or perception labels.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
from kaiev26_msgs.msg import ActuatorCommand, VehicleState

from .bag import OutputBag


class Trial(Node):
    def __init__(self, args):
        super().__init__('aeb_sim_trial', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.args = args
        self.writer = OutputBag(Path(args.output) / 'bag')
        self.client = self.create_client(SetBool, '/aeb/set_tracking_enabled')
        self.reset_client = self.create_client(Trigger, '/aeb/reset_red_stop')
        self.state = self.odom = self.status = None
        self.rows, self.status_rows, self.path_rows = [], [], []
        self.phase = 'setup'
        self.yaw = math.radians(args.course_yaw_deg)
        self.forward = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        self.left = np.array([-self.forward[1], self.forward[0]])
        # Station and lateral offset are measured from this world-map origin.
        # With the default +Y heading, station equals world Y and +offset is west.
        self.origin = np.array([args.center_x, args.center_y])
        topics = {
            '/simulation/ground_truth/odometry': Odometry, '/vehicle/state': VehicleState,
            '/aeb/center_path': RosPath, '/aeb/cones': MarkerArray, '/aeb/boundaries': MarkerArray,
            '/aeb/red_gate': PoseArray,
            '/aeb/status': String, '/aeb/tracking_status': String, '/aeb/control_command': ActuatorCommand,
            '/planning/command': ActuatorCommand, '/vehicle/command': ActuatorCommand,
            '/aeb/pursuit_target': Marker, '/aeb/predicted_path': Marker, '/aeb/points': PointCloud2,
            '/aeb/left/image/compressed': CompressedImage, '/aeb/right/image/compressed': CompressedImage,
        }
        self.subs = [self.create_subscription(cls, topic, lambda msg, t=topic: self.receive(t, msg),
                                              qos_profile_sensor_data) for topic, cls in topics.items()]

    def receive(self, topic, msg):
        ns = self.get_clock().now().nanoseconds
        self.writer.write(topic, msg, ns)
        if topic == '/vehicle/state':
            self.state = msg
        elif topic == '/aeb/tracking_status':
            self.status = json.loads(msg.data)
            self.status_rows.append(dict(self.status, time_s=ns/1e9, phase=self.phase))
        elif topic == '/aeb/status':
            self.path_rows.append(dict(json.loads(msg.data), time_s=ns/1e9, phase=self.phase))
        elif topic == '/simulation/ground_truth/odometry':
            self.odom = msg
            position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            relative = position - self.origin
            station = float(self.forward @ relative)
            lateral = float(self.left @ relative)
            q = msg.pose.pose.orientation
            heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.rows.append(dict(time_s=ns/1e9, phase=self.phase, station_m=station,
                                  lateral_error_m=lateral,
                                  world_x_m=float(position[0]), world_y_m=float(position[1]),
                                  heading_error_deg=math.degrees(math.atan2(math.sin(heading-self.yaw),
                                                                            math.cos(heading-self.yaw))),
                                  gt_speed_mps=float(math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y))))

    def wait_until(self, condition, timeout=30):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return
        raise RuntimeError('Timed out waiting for simulator/path/control response')

    def enable(self, value):
        if not self.client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Start aeb_sim.launch.py first')
        future = self.client.call_async(SetBool.Request(data=value))
        self.wait_until(future.done, 5)
        if not future.result().success:
            raise RuntimeError(future.result().message)

    def run(self):
        self.wait_until(lambda: self.odom is not None and self.state is not None)
        if self.state.mode != VehicleState.MODE_AUTO or self.state.estop_active:
            raise RuntimeError('Simulator must be in AUTO with its E-stop released')
        self.enable(False)
        self.wait_until(lambda: abs(self.state.speed_mps) < 0.05)
        pose = self.origin + self.forward*self.args.start_station + self.left*self.args.offset_m
        yaw = self.yaw + math.radians(self.args.yaw_error_deg)
        request = (f'name: "kaiev26" position {{ x: {pose[0]} y: {pose[1]} z: 0.0592 }} '
                   f'orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }}')
        response = subprocess.run([
            'gz', 'service', '-s', '/world/square_intersection_loop/set_pose',
            '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '4000',
            '--req', request], check=True, capture_output=True, text=True)
        if 'true' not in response.stdout:
            raise RuntimeError(f'Gazebo pose change failed: {response.stdout}')
        self.wait_until(lambda: self.rows and math.hypot(
            self.rows[-1]['world_x_m']-pose[0], self.rows[-1]['world_y_m']-pose[1])<0.3)
        settled = self.get_clock().now().nanoseconds + 3_000_000_000
        self.wait_until(lambda: self.get_clock().now().nanoseconds >= settled, 30)
        if not self.reset_client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Restart the tracking node: reset_red_stop service is missing')
        reset = self.reset_client.call_async(Trigger.Request())
        self.wait_until(reset.done, 5)
        if not reset.result().success:
            raise RuntimeError(reset.result().message)
        self.wait_until(lambda: self.path_rows and self.path_rows[-1].get('valid', False), 30)
        self.phase = 'driving'
        enabled_after = self.get_clock().now().nanoseconds
        self.status = None
        self.enable(True)
        self.wait_until(lambda: self.status and self.status.get('enabled')
                        and self.status.get('stamp_ns', -1) >= enabled_after, 5)
        begin = self.get_clock().now().nanoseconds / 1e9
        wall_deadline = time.monotonic() + 300
        outcome = 'time_limit'
        red_trigger = None
        while self.get_clock().now().nanoseconds / 1e9 - begin < self.args.max_seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > wall_deadline:
                raise RuntimeError('Trial wall timeout: simulator may be paused')
            row = self.rows[-1]
            if self.status and self.status.get('red_latched'):
                outcome = 'red_stop_detected'
                red_trigger = dict(row, red_distance_m=self.status.get('red_distance_m'))
                break
            if abs(row['lateral_error_m']) > 1.5:
                outcome = 'lateral_error_limit'; break
            if row['station_m'] >= self.args.end_station:
                outcome = 'course_test_end'; break
        self.phase = 'braking'
        self.enable(False)
        # A momentary low wheel-speed estimate is not proof that the car stopped.
        stationary_since = None
        def stable_stop():
            nonlocal stationary_since
            now_s = self.get_clock().now().nanoseconds / 1e9
            if abs(self.state.speed_mps) < .05 and self.rows[-1]['gt_speed_mps'] < .05:
                stationary_since = now_s if stationary_since is None else stationary_since
                return now_s - stationary_since >= 1.0
            stationary_since = None
            return False
        stopped = False
        try:
            self.wait_until(stable_stop, 35)
            stopped = True
            self.phase = 'stopped'
        except RuntimeError:
            self.get_logger().warning('Stable standstill was not verified before timeout')
        drive = [r for r in self.rows if r['phase']=='driving']
        statuses = [r for r in self.status_rows if r['phase']=='driving']
        moving = [r for r in drive if r['gt_speed_mps'] > 0.2]
        lookaheads = [r['lookahead_m'] for r in statuses if r['lookahead_m'] is not None]
        perception_modes = Counter(
            r.get('perception_mode', 'fusion')
            for r in self.path_rows if r['phase'] == 'driving')
        perception_mode = (perception_modes.most_common(1)[0][0]
                           if perception_modes else 'unknown')
        perception_label = ('camera-only YOLO bbox' if perception_mode == 'yolo_only'
                            else 'camera+LiDAR+YOLO fusion')
        controller = self.status.get('controller', 'pure_pursuit')
        report = dict(outcome=outcome, configuration=vars(self.args),
                      lookahead_m_min_max=[min(lookaheads), max(lookaheads)] if lookaheads else [],
                      requested_speed_kph=self.status['target_speed_kph'],
                      max_gt_speed_kph=max((r['gt_speed_mps'] for r in drive), default=0.)*3.6,
                      lateral_abs_m_p50_p95_max=(np.percentile([abs(r['lateral_error_m']) for r in drive], [50,95,100]).tolist() if drive else []),
                      distance_m=drive[-1]['station_m']-drive[0]['station_m'] if drive else 0.,
                      final_station_m=self.rows[-1]['station_m'],
                      status_counts=dict(Counter(r['reason'] for r in statuses)),
                      max_command_kph=max((r['speed_target_mps'] for r in statuses), default=0.)*3.6,
                      moving_lateral_abs_m_p50_p95_max=(np.percentile(
                          [abs(r['lateral_error_m']) for r in moving], [50,95,100]).tolist() if moving else []),
                      stopped=stopped, red_trigger=red_trigger,
                      red_stop_latched=bool(self.status.get('red_latched', False)),
                      perception_mode=perception_mode, controller=controller,
                      method=f'Live {perception_label} -> cone path -> {controller} -> governor -> unmodified SITL plant. GT only initializes/measures/ends trial.')
        root = Path(self.args.output)
        (root/'report.json').write_text(json.dumps(report, indent=2))
        (root/'trajectory.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.rows))
        (root/'tracking.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in self.status_rows))
        print(json.dumps(report, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output',
        default=str(
            Path.home() / 'KAI_ws/records' /
            ('aeb_sim_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))),
        help='New result directory')
    parser.add_argument('--offset-m', type=float, default=0.0)
    parser.add_argument('--yaw-error-deg', type=float, default=0.0)
    parser.add_argument('--center-x', type=float, default=142.25, help='Course origin world X (m)')
    parser.add_argument('--center-y', type=float, default=0.0, help='Course origin world Y (m)')
    parser.add_argument('--course-yaw-deg', type=float, default=90.0, help='Forward heading in world XY')
    parser.add_argument('--start-station', type=float, default=-23.0, help='Forward distance from course origin (m)')
    parser.add_argument('--end-station', type=float, default=49.0, help='Fixed trial stop trigger; not red-cone detection')
    parser.add_argument('--max-seconds', type=float, default=90.0)
    args = parser.parse_args()
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error('Output must be new')
    args.output = str(output)
    output.mkdir(parents=True)
    # Keep the ROS context alive during Python's Ctrl+C cleanup, so the disable
    # service can actually be delivered before this fixture exits.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Trial(args)
    try:
        node.run()
    except KeyboardInterrupt:
        print('Interrupted: disabling pursuit', flush=True)
    finally:
        try:
            node.enable(False)
        finally:
            node.writer.close()
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
