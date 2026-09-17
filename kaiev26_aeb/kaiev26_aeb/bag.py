"""Run the live detector/geometry on one MCAP, writing Foxglove-readable ROS output.

This adapter does not start ROS nodes or publish any topics. The optional Ouster
decoder runs in a separate interpreter so the live ROS/PyTorch environment does
not need the SDK's unrelated CLI dependencies.
"""
import argparse
from collections import Counter
from dataclasses import fields
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from rclpy.serialization import serialize_message
import rosbag2_py

from .core import Config, transform_points, fuse_and_plan
from .detector import Detector, overlay
from .messages import (stamp_ns, transform_matrix, camera_info, cloud_xyz,
                       image_message, cloud_message, output_messages, public_result, empty_result)
from .trial_profiles import default_model_path


class StaticTransforms:
    def __init__(self):
        self.edges = {}

    def add(self, msg):
        for tf in msg.transforms:
            parent, child = tf.header.frame_id, tf.child_frame_id
            t = transform_matrix(tf.transform)
            self.edges.setdefault(parent, {})[child] = t
            self.edges.setdefault(child, {})[parent] = np.linalg.inv(t)

    def lookup(self, target, source):
        todo, seen = [(target, np.eye(4))], {target}
        while todo:
            name, transform = todo.pop(0)
            if name == source:
                return transform
            for neighbor, edge in self.edges.get(name, {}).items():
                if neighbor not in seen:
                    seen.add(neighbor)
                    todo.append((neighbor, transform @ edge))
        raise ValueError(f'Missing recorded TF: {target} <- {source}')


class OutputBag:
    def __init__(self, path):
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                         rosbag2_py.ConverterOptions('', ''))
        self.topics = set()

    def write(self, topic, msg, log_time):
        if topic not in self.topics:
            typ = f'{type(msg).__module__.split(".")[0]}/msg/{type(msg).__name__}'
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ, serialization_format='cdr'))
            self.topics.add(topic)
        self.writer.write(topic, serialize_message(msg), int(log_time))

    def close(self):
        # Humble finalizes metadata when SequentialWriter is destroyed.
        self.writer = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, help='MCAP file or single-MCAP rosbag directory')
    parser.add_argument('--output', required=True, help='New output rosbag directory; existing paths refused')
    parser.add_argument('--config', default=str(
        Path(get_package_share_directory('kaiev26_aeb')) / 'config/perception.yaml'))
    parser.add_argument('--model')
    parser.add_argument('--device', default=None)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--ouster-python', default=sys.executable,
                        help='Only for packet-only bags: Python environment containing ouster-sdk, mcap and mcap_ros2')
    args = parser.parse_args()
    source = Path(args.input).expanduser().resolve()
    if source.is_dir():
        files = list(source.glob('*.mcap'))
        if len(files) != 1:
            parser.error('Pass a single MCAP file; multi-file merging is not implemented')
        source = files[0]
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error(f'Output already exists: {output}')
    if args.hz <= 0:
        parser.error('--hz must be positive')
    p = yaml.safe_load(Path(args.config).read_text())['cone_path_node']['ros__parameters']
    cfg = Config(**{f.name: p.get(f.name, f.default) for f in fields(Config)})
    before = source.stat()
    model_path = args.model or p['model_path'] or default_model_path()
    detector = Detector(model_path, args.device or p['device'], p['image_size'], p['confidence'])
    transforms = StaticTransforms()
    images, infos, state = {'left': [], 'right': []}, {}, []
    with source.open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        start_ns = summary.statistics.message_start_time
        topic_counts = {c.topic: summary.statistics.channel_message_counts.get(c.id, 0) for c in summary.channels.values()}
        topics = ['/tf_static', '/vehicle/state'] + [p[f'{s}_{kind}_topic'] for s in images for kind in ('image', 'info')]
        # The calibration is required to be static throughout this simple run.
        for _, c, msg, decoded in reader.iter_decoded_messages(topics=topics):
            if c.topic == '/tf_static':
                transforms.add(decoded)
            elif c.topic == '/vehicle/state':
                state.append((msg.log_time, float(decoded.speed_mps)))
            else:
                for side in images:
                    if c.topic == p[side + '_info_topic']:
                        info = camera_info(decoded)
                        if side in infos and info != infos[side]:
                            raise ValueError(f'{side} calibration changes within bag; split at the change')
                        infos[side] = info
                    elif c.topic == p[side + '_image_topic']:
                        images[side].append({'stamp': stamp_ns(decoded.header), 'data': bytes(decoded.data),
                                             'frame': decoded.header.frame_id})
    for side in images:
        if not images[side] or side not in infos:
            raise ValueError(f'Bag needs {side} compressed images and CameraInfo')
        images[side].sort(key=lambda r: r['stamp'])
        if any(r['frame'] != infos[side]['frame'] for r in images[side]):
            raise ValueError(f'{side} image and calibration frames differ')
    image_times = {s: np.array([r['stamp'] for r in rows], dtype=np.int64) for s, rows in images.items()}
    temp = tempfile.TemporaryDirectory(prefix='kaiev26-aeb-scans-')
    output.parent.mkdir(parents=True, exist_ok=True)
    time_basis = 'pointcloud_acquisition_header_plus_offset'
    has_points = topic_counts.get(p['points_topic'], 0) > 0
    scan_rows = None
    if not has_points:
        if not topic_counts.get('/ouster/lidar_packets', 0) or not topic_counts.get('/ouster/metadata', 0):
            raise ValueError('No PointCloud2 or decodable Ouster packets/metadata')
        command = [args.ouster_python, str(Path(__file__).with_name('decode_packets.py')),
                   str(source), temp.name, '--hz', str(args.hz)]
        import os
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        subprocess.run(command, env=env, check=True)
        decoded = json.loads((Path(temp.name) / 'scans.json').read_text())
        scan_rows, time_basis = decoded['scans'], decoded['time_basis']

    def scans():
        if scan_rows is not None:
            for row in scan_rows:
                yield row['stamp_ns'], row['log_ns'], row['frame'], np.load(Path(temp.name) / row['file'])['xyz']
        else:
            last = None
            with source.open('rb') as stream:
                reader = make_reader(stream, decoder_factories=[DecoderFactory()])
                for _, _, msg, decoded in reader.iter_decoded_messages(topics=[p['points_topic']]):
                    stamp = stamp_ns(decoded.header) + round(p['cloud_time_offset_s'] * 1e9)
                    if last is not None and stamp - last < 1e9 / args.hz * .95:
                        continue
                    last = stamp
                    yield stamp, msg.log_time, decoded.header.frame_id, cloud_xyz(decoded)

    writer = OutputBag(output)
    records, counts = [], Counter()
    begin = time.monotonic()
    try:
        for seq, (stamp, log_time, frame, xyz) in enumerate(scans()):
            if args.max_frames and seq >= args.max_frames:
                break
            tick = time.monotonic()
            selected, camera_data, frames, deltas = {}, {}, {}, {}
            for side in images:
                index = int(np.argmin(abs(image_times[side] - stamp)))
                selected[side] = images[side][index]
                deltas[side] = (selected[side]['stamp'] - stamp) / 1e6
            speed = min(state, key=lambda r: abs(r[0] - log_time))[1] if state else None
            if any(abs(dt) > p['max_camera_dt_s'] * 1000 for dt in deltas.values()):
                result = empty_result('camera_unsynchronized')
                inference_ms = 0.0
            else:
                for side, record in selected.items():
                    im = cv2.imdecode(np.frombuffer(record['data'], np.uint8), cv2.IMREAD_COLOR)
                    if im is None or (im.shape[1], im.shape[0]) != (infos[side]['width'], infos[side]['height']):
                        raise ValueError(f'{side} invalid image or calibration dimensions')
                    frames[side] = im
                    camera_data[side] = {'info': infos[side],
                        'transform': transforms.lookup(infos[side]['frame'], p['output_frame'])}
                detections, inference_ms = detector.predict(frames)
                for side in images:
                    camera_data[side]['detections'] = detections[side]
                base_xyz = transform_points(xyz, transforms.lookup(p['output_frame'], frame))
                result = fuse_and_plan(base_xyz, camera_data, cfg)
            extra = {'source_time_s': (log_time - start_ns) / 1e9, 'camera_dt_ms': deltas,
                     'lidar_time_basis': time_basis, 'inference_ms': inference_ms,
                     'processing_ms': (time.monotonic() - tick) * 1000, 'speed_mps': speed}
            for topic, msg in output_messages(result, stamp, p['output_frame'], extra).items():
                writer.write(topic, msg, log_time)
            if 'xyz' in result:
                writer.write('/aeb/points', cloud_message(result['xyz'], stamp, p['output_frame']), log_time)
                for side, im in frames.items():
                    ok, encoded = cv2.imencode('.jpg', overlay(im, side, detections[side], result))
                    if not ok:
                        raise ValueError('JPEG encode failed')
                    writer.write(f'/aeb/{side}/image/compressed', image_message(encoded, stamp, infos[side]['frame']), log_time)
            record = {**public_result(result), **extra}
            records.append(record)
            counts[result['plan']['reason']] += 1
            if seq % 40 == 0:
                print(f"{seq:4d} +{extra['source_time_s']:.2f}s {result['plan']['reason']} "
                      f"cones={dict(Counter(c['color'] for c in result['cones']))}", flush=True)
    finally:
        writer.close()
        temp.cleanup()
    after = source.stat()
    assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
    stats = {'source': str(source), 'model': str(Path(model_path).resolve()),
             'frames': len(records), 'status_counts': dict(counts), 'lidar_time_basis': time_basis,
             'elapsed_s': time.monotonic() - begin, 'source_unchanged': True,
             'processing_ms_p50_p95': np.percentile([r['processing_ms'] for r in records], [50, 95]).tolist() if records else []}
    (output / 'results.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records))
    (output / 'metrics.json').write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats, indent=2), flush=True)


if __name__ == '__main__':
    main()
