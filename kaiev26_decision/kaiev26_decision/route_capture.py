"""Manual MCAP capture conversion and fixed route phase tables (no publishers)."""

from __future__ import annotations

from bisect import bisect_right
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml
from kaiev26_motion_control.motion_control_node import (
    _sample_path, _signed_curvature, median_curvature_profile,
    validate_route_segments,
)


LOCALIZATION_PARAMETERS = (
    'map_origin.configured', 'map_origin.latitude_deg', 'map_origin.longitude_deg',
    'map_origin.altitude_m', 'frames.map', 'frames.base', 'frames.gnss',
    'mode.gnss_bypass', 'time.max_delay_sec', 'time.sync_tolerance_sec',
)
MAP_PACKAGE_PATHS = ('src/kaiev26_maps', 'src/Simulation/kaiev26_maps')


def workspace_root() -> Path:
    for root in Path(__file__).resolve().parents:
        if any((root / path / 'package.xml').is_file() for path in MAP_PACKAGE_PATHS):
            return root
    return Path.home() / 'KAI_ws'


def route_directory() -> Path:
    root = workspace_root()
    for path in MAP_PACKAGE_PATHS:
        if (root / path / 'package.xml').is_file():
            return root / path / 'maps/midan'
    return root / MAP_PACKAGE_PATHS[0] / 'maps/midan'


def origin_from_parameters(parameters: dict) -> dict:
    if parameters.get('map_origin.configured') is not True:
        raise ValueError('Localization map origin is unknown; MCAP retained')
    origin = {key: parameters.get('map_origin.' + key)
              for key in ('latitude_deg', 'longitude_deg', 'altitude_m')}
    if (any(v is None or not math.isfinite(v) for v in origin.values())
            or abs(origin['latitude_deg']) > 90 or abs(origin['longitude_deg']) > 180
            or parameters.get('frames.map') != 'map'
            or parameters.get('frames.base') != 'base_footprint'):
        raise ValueError('Invalid localization origin/reference frame')
    return origin


def verify_origin(document: dict, parameters: dict) -> None:
    current = origin_from_parameters(parameters)
    if document.get('map_origin') != current:
        raise ValueError('Collected route and running Localization origins differ')


def route_geometry(document: dict) -> list[tuple[float, float]]:
    if document.get('frame_id') != 'map' or document.get('reference_point') != 'base_footprint':
        raise ValueError('Expected map -> base_footprint route')
    points = [(float(p['x']), float(p['y'])) for p in document['waypoints']]
    if len(points) < 2 or any(not math.isfinite(v) for p in points for v in p):
        raise ValueError('Route needs at least two finite points')
    if any(math.dist(a, b) <= 1e-9 for a, b in zip(points, points[1:])):
        raise ValueError('Duplicate adjacent route points')
    return points


def geometry_hash(document: dict) -> str:
    value = {key: document[key] for key in ('frame_id', 'reference_point', 'map_origin')}
    value['xy'] = route_geometry(document)
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def make_tracking_profile(document: dict) -> dict:
    points = route_geometry(document)
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    # Use the entire polyline, not the controller's ego-forward path cropping.
    samples = []
    s = 2.0
    while s <= length - 2.0 + 1e-9:
        samples.append((s, _signed_curvature(_sample_path(points, s - 2),
                                            _sample_path(points, s),
                                            _sample_path(points, s + 2))))
        s += 1.0
    samples = median_curvature_profile(samples)
    corners, run = [], []

    def flush():
        if len(run) >= 3 and max(abs(k) for _, k in run) >= 0.015:
            corners.append((run[0][0], run[-1][0] + 1, 1 if run[0][1] > 0 else -1))

    for sample in samples:
        if abs(sample[1]) < 0.010:
            flush()
            run = []
        else:
            if run and sample[1] * run[-1][1] < 0:
                flush()
                run = []
            run.append(sample)
    flush()
    merged = []
    for start, end, sign in corners:
        if merged and sign == merged[-1][2] and start - merged[-1][1] <= 4:
            merged[-1] = (merged[-1][0], end, sign)
        else:
            merged.append((start, end, sign))
    # Require a sustained heading change as well as a curvature peak.
    corner_details = [dict(start_s_m=a, end_s_m=b, sign=sign,
                           heading_change_deg=math.degrees(sum(k for s, k in samples if a <= s < b)))
                      for a, b, sign in merged]
    corner_details = [c for c in corner_details if abs(c['heading_change_deg']) >= 10.]
    merged = [(c['start_s_m'], c['end_s_m'], c['sign']) for c in corner_details]
    windows = []
    for start, end, sign in merged:
        windows.extend([(start - 14, start - 6, 'turn_approach', 2),
                        (start - 6, end, 'turning', 3),
                        (end, end + 4, 'turn_exit', 1)])
    for first, second in zip(merged, merged[1:]):
        if first[2] != second[2] and second[0] - first[1] <= 12:
            windows.append((first[0] - 6, second[1], 'chained_turn', 4))
    windows = [(max(0., a), min(length, b), phase, rank)
               for a, b, phase, rank in windows if min(length, b) > max(0., a)]
    boundaries = sorted({0., length, *(a for a, _, _, _ in windows),
                         *(b for _, b, _, _ in windows)})
    segments = []
    for a, b in zip(boundaries, boundaries[1:]):
        active = [(rank, phase) for x, y, phase, rank in windows if x <= (a+b)/2 < y]
        phase = max(active)[1] if active else 'straight'
        if segments and segments[-1]['phase'] == phase:
            segments[-1]['end_s_m'] = b
        else:
            segments.append(dict(start_s_m=a, end_s_m=b, phase=phase))
    return dict(mode='route_phase_table', version=1, generator_version=1,
                geometry_sha256=geometry_hash(document), route_length_m=length,
                generator_settings=dict(sample_half_width_m=2., sample_spacing_m=1.,
                                        median_samples=3, minimum_samples=3,
                                        curvature_on_1pm=.015, curvature_off_1pm=.010,
                                        minimum_heading_change_deg=10., same_direction_gap_m=4.,
                                        chained_turn_gap_m=12.,
                                        approach_lead_m=14., turning_lead_m=6., exit_m=4.),
                segments=segments, corner_count=len(merged), corners=corner_details)


def profile_parameters(document: dict) -> dict:
    points = route_geometry(document)
    profile = document['tracking_profile']
    if (profile.get('mode') != 'route_phase_table' or profile.get('version') != 1
            or profile.get('geometry_sha256') != geometry_hash(document)):
        raise ValueError('Route phase table schema/hash mismatch')
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    if abs(length - profile['route_length_m']) > 1e-6:
        raise ValueError('Route phase table length mismatch')
    segments = profile['segments']
    if not segments or any(abs(a['end_s_m'] - b['start_s_m']) > 1e-8
                           for a, b in zip(segments, segments[1:])):
        raise ValueError('Route phase table gap/overlap')
    boundaries, phases = validate_route_segments(
        [segments[0]['start_s_m']] + [s['end_s_m'] for s in segments],
        [s['phase'] for s in segments], length)
    return {'hybrid_route_length_m': length,
            'hybrid_route_segment_boundaries_m': list(boundaries),
            'hybrid_route_segment_phases': list(phases)}


def load_custom_route(path: str | Path) -> dict:
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    profile_parameters(document)
    run_id = document.get('route_id')
    if (document.get('capture_mode') != 'manual_tui' or not isinstance(run_id, str)
            or not run_id or run_id in ('.', '..') or Path(run_id).name != run_id
            or not isinstance(document.get('route_name'), str) or not document['route_name'].strip()):
        raise ValueError('Not a manually captured route')
    return document


def discover_routes(root: Path) -> tuple[dict, list[str]]:
    routes, errors = {}, []
    for path in sorted(root.glob('*/route.yaml')):
        try:
            document = load_custom_route(path)
            key = 'custom:' + document['route_id']
            if key in routes:
                raise ValueError('Duplicate route ID')
            routes[key] = (document['route_name'], str(path.resolve()))
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
            errors.append(f'{path}: {error}')
    return routes, errors


def resample_route(samples: list[tuple[float, float, float]], spacing=1.0):
    if len(samples) < 2:
        raise ValueError('Not enough moving odometry')
    points, distances = [(samples[0][1], samples[0][2])], [0.]
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        distance = math.dist(previous[1:], current[1:])
        if dt <= 0 or not math.isfinite(distance):
            raise ValueError('Odometry time reversal/invalid position')
        if distance > max(1.0, 20. * dt):
            raise ValueError('Odometry position jump; no gap interpolation')
        if distance > 1e-5:
            points.append(current[1:])
            distances.append(distances[-1] + distance)
    if distances[-1] <= 1e-3:
        raise ValueError('No measured movement')
    result = [points[0]]
    s = spacing
    while s < distances[-1] - 1e-6:
        index = bisect_right(distances, s) - 1
        fraction = (s - distances[index]) / (distances[index+1] - distances[index])
        result.append(tuple(a + fraction*(b-a) for a, b in zip(points[index], points[index+1])))
        s += spacing
    result.append(points[-1])
    return result


def bag_messages(bag: Path, topics: list[str]):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
             if item.name in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    while reader.has_next():
        topic, payload, received_ns = reader.read_next()
        yield topic, deserialize_message(payload, types[topic]), received_ns


def stamp_ns(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def recording_quality(bag: Path, topics: tuple[str, ...]) -> dict:
    """Receipt timing for small dynamics topics; images/clouds remain in the MCAP."""
    selected = [t for t in topics if not t.startswith(('/perception/', '/debug/', '/pv26_perception/'))
                and t not in ('/ouster/points', '/tf', '/rosout', '/parameter_events')]
    result = {}
    for topic, message, received in bag_messages(bag, selected):
        item = result.setdefault(topic, dict(count=0, first_received_ns=received,
                                             last_received_ns=received, max_receipt_gap_s=0.,
                                             nonincreasing_header_stamps=0))
        item['count'] += 1
        item['max_receipt_gap_s'] = max(item['max_receipt_gap_s'],
                                       (received-item['last_received_ns'])*1e-9)
        item['last_received_ns'] = received
        if hasattr(message, 'header'):
            stamp = stamp_ns(message)
            if stamp <= item.get('last_header_stamp_ns', -1):
                item['nonincreasing_header_stamps'] += 1
            item.setdefault('first_header_stamp_ns', stamp)
            item['last_header_stamp_ns'] = stamp
            item['frame_id'] = message.header.frame_id
        if topic == '/tf_static':
            item['transforms'] = [dict(parent=t.header.frame_id, child=t.child_frame_id,
                translation=[t.transform.translation.x, t.transform.translation.y, t.transform.translation.z],
                rotation=[t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
                for t in message.transforms]
    return result


def capture_from_bag(bag: Path, start_ns: int, end_ns: int, parameters: dict,
                     name: str, run_id: str, destination: Path, settings_snapshot=None) -> dict:
    from kaiev26_msgs.msg import VehicleState
    origin = origin_from_parameters(parameters)
    if not name.strip() or not run_id or run_id in ('.', '..') or Path(run_id).name != run_id:
        raise ValueError('Invalid route name/run ID')
    if not 0 < start_ns < end_ns:
        raise ValueError('Invalid manual capture interval')
    vehicles, odometry = [], []
    for topic, message, received in bag_messages(bag, ['/vehicle/state', '/localization/odometry']):
        stamp = stamp_ns(message)
        if start_ns <= stamp <= end_ns and received <= end_ns:
            (vehicles if topic == '/vehicle/state' else odometry).append((stamp, message))
    if not vehicles or not odometry:
        raise ValueError('Missing vehicle/odometry in capture interval')
    if any(m.mode != VehicleState.MODE_MANUAL or m.estop_active or not math.isfinite(m.speed_mps)
           or m.speed_mps < -.1 for _, m in vehicles):
        raise ValueError('Capture includes AUTO, E-stop, reverse or invalid speed')
    stamps = [s for s, _ in vehicles]
    if any(b <= a or b-a > 1_000_000_000 for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Vehicle sample time gap/reversal')
    samples, last_stamp, was_moving, previous_raw = [], None, False, None
    for stamp, message in odometry:
        if last_stamp == stamp:
            continue
        if last_stamp is not None and (stamp < last_stamp or stamp-last_stamp > 1_000_000_000):
            raise ValueError('Odometry sample gap/reversal')
        last_stamp = stamp
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (message.header.frame_id != 'map' or message.child_frame_id != 'base_footprint'
                or any(not math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                or any(not 0 < message.pose.covariance[i] <= .25 for i in (0, 7))):
            raise ValueError('Invalid odometry frame/position/covariance')
        index = max(0, bisect_right(stamps, stamp)-1)
        if abs(stamps[index] - stamp) > 1_000_000_000:
            raise ValueError('Vehicle/odometry time mismatch')
        moving = abs(vehicles[index][1].speed_mps) > .1
        point = (stamp*1e-9, float(p.x), float(p.y))
        if previous_raw is not None:
            if math.dist(point[1:], previous_raw[1:]) > max(1., 20.*(point[0]-previous_raw[0])):
                raise ValueError('Odometry position jump; no gap interpolation')
        previous_raw = point
        if samples and moving and was_moving:
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            dx, dy = p.x-samples[-1][1], p.y-samples[-1][2]
            if dx*math.cos(yaw)+dy*math.sin(yaw) < -.2:
                raise ValueError('Reverse trajectory in manual capture')
        if not samples or moving or was_moving:
            samples.append(point)
        was_moving = moving
    points = resample_route(samples)
    document = dict(route_name=name.strip(), route_id=run_id, capture_mode='manual_tui',
                    frame_id='map', reference_point='base_footprint', map_origin=origin,
                    coordinate_system='Recorded eskf_localization map; no reprojection',
                    localization_parameters=parameters, source_bag=str(bag.resolve()),
                    source_interval_ros_ns=[start_ns, end_ns], sampling_distance_m=1.0,
                    waypoint_index_base=0,
                    generation_settings_snapshot=settings_snapshot or {},
                    waypoints=[dict(x=x, y=y, z=0.0, zone='NORMAL_ZONE') for x, y in points])
    document['tracking_profile'] = make_tracking_profile(document)
    profile_parameters(document)
    if not destination.is_dir() or not (destination.parent.parent / 'package.xml').is_file():
        raise ValueError('route_dir must be the existing kaiev26_maps/maps/midan directory')
    target = destination / run_id
    target.mkdir(exist_ok=False)
    fd, temporary = tempfile.mkstemp(prefix='.route-', suffix='.tmp', dir=target)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        load_custom_route(temporary)
        os.replace(temporary, target / 'route.yaml')
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return dict(path=str(target / 'route.yaml'), name=name, points=len(points),
                length_m=document['tracking_profile']['route_length_m'],
                corners=document['tracking_profile']['corner_count'],
                yaml_sha256=hashlib.sha256((target / 'route.yaml').read_bytes()).hexdigest())
