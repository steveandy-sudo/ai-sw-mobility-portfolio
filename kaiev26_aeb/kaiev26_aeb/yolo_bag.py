"""Run the YOLO-only path generator on one recorded MCAP."""

import argparse
from collections import Counter
from dataclasses import fields
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from .bag import OutputBag, StaticTransforms
from .core import Config
from .detector import Detector
from .messages import (camera_info, empty_result, image_message, output_messages,
                       public_result, stamp_ns)
from .trial_profiles import default_model_path
from .yolo_core import overlay_yolo_only, yolo_only_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True,
                        help='MCAP file or a directory containing one MCAP')
    parser.add_argument('--output', required=True,
                        help='New output rosbag directory; existing paths are refused')
    parser.add_argument('--config', default=str(
        Path(get_package_share_directory('kaiev26_aeb')) / 'config/perception.yaml'))
    parser.add_argument('--model')
    parser.add_argument('--device', default=None)
    parser.add_argument('--hz', type=float, default=10.0)
    parser.add_argument('--max-frames', type=int, default=0)
    args = parser.parse_args()

    source = Path(args.input).expanduser().resolve()
    if source.is_dir():
        files = list(source.glob('*.mcap'))
        if len(files) != 1:
            parser.error('Pass a directory containing exactly one MCAP')
        source = files[0]
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        parser.error(f'Output already exists: {output}')
    if args.hz <= 0:
        parser.error('--hz must be positive')

    p = yaml.safe_load(Path(args.config).read_text())[
        'cone_yolo_path_node']['ros__parameters']
    cfg = Config(**{item.name: p.get(item.name, item.default)
                    for item in fields(Config)})
    detector = Detector(args.model or p['model_path'] or default_model_path(),
                        args.device or p['device'],
                        p['image_size'], p['confidence'])
    transforms = StaticTransforms()
    images = {side: [] for side in ('left', 'right')}
    infos, states = {}, []
    before = source.stat()
    with source.open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        summary = reader.get_summary()
        start_ns = summary.statistics.message_start_time
        topics = ['/tf_static', '/vehicle/state'] + [
            p[f'{side}_{kind}_topic'] for side in images for kind in ('image', 'info')]
        for _, channel, message, decoded in reader.iter_decoded_messages(topics=topics):
            if channel.topic == '/tf_static':
                transforms.add(decoded)
            elif channel.topic == '/vehicle/state':
                states.append((message.log_time, float(decoded.speed_mps)))
            else:
                side = 'left' if '/left/' in channel.topic else 'right'
                if channel.topic == p[side + '_info_topic']:
                    info = camera_info(decoded)
                    if side in infos and info != infos[side]:
                        raise ValueError(f'{side} calibration changes within the bag')
                    infos[side] = info
                elif channel.topic == p[side + '_image_topic']:
                    images[side].append({
                        'stamp': stamp_ns(decoded.header), 'log': message.log_time,
                        'frame': decoded.header.frame_id, 'data': bytes(decoded.data)})
    for side in images:
        if not images[side] or side not in infos:
            raise ValueError(f'Bag needs {side} compressed images and CameraInfo')
        images[side].sort(key=lambda row: row['stamp'])
        if any(row['frame'] != infos[side]['frame'] for row in images[side]):
            raise ValueError(f'{side} image and CameraInfo frames differ')

    right_times = np.asarray([row['stamp'] for row in images['right']], dtype=np.int64)
    camera_transforms = {
        side: transforms.lookup(p['output_frame'], infos[side]['frame'])
        for side in images
    }
    writer = OutputBag(output)
    records, counts = [], Counter()
    last_source_stamp = None
    last_output_stamp = None
    begin = time.monotonic()
    try:
        for left in images['left']:
            if (last_source_stamp is not None
                    and left['stamp'] - last_source_stamp < .95e9 / args.hz):
                continue
            right = images['right'][int(np.argmin(abs(right_times - left['stamp'])))]
            stamp = round((left['stamp'] + right['stamp']) / 2)
            if last_output_stamp is not None and stamp <= last_output_stamp:
                continue
            last_source_stamp = left['stamp']
            last_output_stamp = stamp
            if args.max_frames and len(records) >= args.max_frames:
                break
            tick = time.monotonic()
            delta_ms = (right['stamp'] - left['stamp']) / 1e6
            frames, detections = {}, {'left': [], 'right': []}
            if abs(delta_ms) > p['max_camera_dt_s'] * 1000:
                result = empty_result('camera_unsynchronized')
                inference_ms = 0.0
            else:
                for side, row in (('left', left), ('right', right)):
                    image = cv2.imdecode(
                        np.frombuffer(row['data'], np.uint8), cv2.IMREAD_COLOR)
                    info = infos[side]
                    if image is None or (image.shape[1], image.shape[0]) != (
                            info['width'], info['height']):
                        raise ValueError(f'{side} invalid image or calibration dimensions')
                    frames[side] = image
                detections, inference_ms = detector.predict(frames)
                camera_data = {
                    side: {'info': infos[side], 'transform': camera_transforms[side],
                           'detections': detections[side]}
                    for side in images
                }
                result = yolo_only_plan(
                    camera_data, cfg, p['min_bbox_height_px'],
                    p['min_bbox_width_px'], p['bbox_bottom_margin_px'],
                    p['bbox_bottom_offset_px'], p['duplicate_merge_radius_m'])
            log_time = max(left['log'], right['log'])
            speed = min(states, key=lambda row: abs(row[0] - log_time))[1] if states else None
            extra = {
                'perception_mode': 'yolo_only',
                'distance_method': 'bbox_bottom_ground_intersection',
                'source_time_s': (log_time - start_ns) / 1e9,
                'camera_dt_ms': {'left': (left['stamp'] - stamp) / 1e6,
                                 'right': (right['stamp'] - stamp) / 1e6},
                'inference_ms': inference_ms,
                'processing_ms': (time.monotonic() - tick) * 1000,
                'speed_mps': speed,
            }
            for topic, message in output_messages(
                    result, stamp, p['output_frame'], extra).items():
                writer.write(topic, message, log_time)
            for side, image in frames.items():
                debug = overlay_yolo_only(image, side, detections[side], result)
                ok, encoded = cv2.imencode('.jpg', debug)
                if not ok:
                    raise ValueError('JPEG encode failed')
                writer.write(f'/aeb/{side}/image/compressed',
                             image_message(encoded, stamp, infos[side]['frame']), log_time)
            record = {**public_result(result), **extra}
            records.append(record)
            counts[result['plan']['reason']] += 1
            if len(records) % 40 == 1:
                print(f"{len(records)-1:4d} +{extra['source_time_s']:.2f}s "
                      f"{result['plan']['reason']} "
                      f"cones={dict(Counter(c['color'] for c in result['cones']))}",
                      flush=True)
    finally:
        writer.close()
    after = source.stat()
    assert (before.st_size, before.st_mtime_ns) == (after.st_size, after.st_mtime_ns)
    stats = {
        'source': str(source),
        'model': str(Path(args.model or p['model_path']).expanduser().resolve()),
        'mode': 'yolo_only',
        'distance_method': 'bbox_bottom_ground_intersection',
        'frames': len(records),
        'status_counts': dict(counts),
        'red_gate_frames': sum(bool(row.get('red_gate')) for row in records),
        'elapsed_s': time.monotonic() - begin,
        'source_unchanged': True,
        'processing_ms_p50_p95': np.percentile(
            [row['processing_ms'] for row in records], [50, 95]).tolist()
            if records else [],
    }
    (output / 'results.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in records), encoding='utf-8')
    (output / 'metrics.json').write_text(
        json.dumps(stats, indent=2), encoding='utf-8')
    print(json.dumps(stats, indent=2), flush=True)


if __name__ == '__main__':
    main()
