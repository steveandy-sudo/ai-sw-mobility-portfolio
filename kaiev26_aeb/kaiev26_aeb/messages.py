"""ROS input decoding and output construction used in both live and offline runs."""
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def stamp_ns(header):
    return int(header.stamp.sec) * 10**9 + int(header.stamp.nanosec)


def header(stamp, frame):
    return Header(stamp=Time(sec=int(stamp // 10**9), nanosec=int(stamp % 10**9)), frame_id=frame)


def transform_matrix(transform):
    t, q = transform.translation, transform.rotation
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [t.x, t.y, t.z]
    return out


def camera_info(msg):
    if msg.width <= 0 or msg.height <= 0 or msg.k[0] <= 0 or msg.k[4] <= 0:
        raise ValueError('CameraInfo is not calibrated')
    if (msg.binning_x not in (0, 1) or msg.binning_y not in (0, 1)
            or msg.roi.x_offset or msg.roi.y_offset):
        raise ValueError('Binned/cropped CameraInfo requires adjusted intrinsics')
    return {'width': msg.width, 'height': msg.height, 'k': list(msg.k), 'd': list(msg.d),
            'distortion_model': msg.distortion_model, 'frame': msg.header.frame_id}


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if not all(n in fields and fields[n].datatype in (7, 8) and fields[n].count == 1 for n in ('x', 'y', 'z')):
        raise ValueError('PointCloud2 needs scalar FLOAT32/FLOAT64 x, y, z fields')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError('Truncated PointCloud2')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [endian + ('f4' if fields[n].datatype == 7 else 'f8') for n in ('x', 'y', 'z')],
                      'offsets': [fields[n].offset for n in ('x', 'y', 'z')], 'itemsize': msg.point_step})
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step))
    return np.column_stack([data[n].ravel() for n in ('x', 'y', 'z')])


def cloud_message(points, stamp, frame):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    msg = PointCloud2(header=header(stamp, frame), height=1, width=len(points),
                      is_bigendian=False, point_step=12, row_step=12 * len(points), is_dense=True)
    msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.data = points.tobytes()
    return msg


def image_message(encoded, stamp, frame):
    return CompressedImage(header=header(stamp, frame), format='jpeg', data=bytes(encoded))


def predicted_path_marker(result, wheelbase_m, hdr):
    """Display limited steering held constant over PP lookahead, or 7 m for Stanley.

    This planar rear-axle bicycle arc is a command visualization, not measured
    motion or a simulation of future controller updates/braking/actuator lag.
    """
    marker = Marker(header=hdr, ns='pp_predicted_path', id=0,
                    type=Marker.LINE_STRIP, action=Marker.DELETE)
    marker.pose.orientation.w = 1.0
    marker.scale.x = .09
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, .15, .85, 1.0
    marker.lifetime = Duration(sec=0, nanosec=500_000_000)
    if not result['tracking'] or result['brake_engage']:
        return marker
    length = result.get('lookahead_m') or 7.0  # Stanley visualization only, not control preview.
    curvature = math.tan(result['steering_target_rad']) / wheelbase_m
    for s in np.linspace(0.0, length, max(2, math.ceil(length / .1) + 1)):
        # sinc avoids loss of precision when steering is close to zero.
        angle = curvature * s
        x = s * np.sinc(angle / np.pi)
        y = s * np.sin(angle / 2) * np.sinc(angle / (2 * np.pi))
        marker.points.append(Point(x=float(x), y=float(y), z=.05))
    marker.action = Marker.ADD
    return marker


def public_result(result):
    plan = result['plan']
    return {**{k: v for k, v in plan.items() if k != 'path'},
            'path_points': len(plan['path']),
            'cones': [{'xyz': c['xyz'].tolist(), 'color': c['color'], 'side': c['side'],
                       'score': c['score'], 'lidar_points': len(c['indices']), 'matches': c['matches']}
                      for c in result['cones']]}


COLORS = {'blue': (0.1, 0.4, 1.0), 'yellow': (1.0, 0.9, 0.0),
          'red': (1.0, 0.1, 0.1), 'unknown': (0.55, 0.55, 0.55)}


def output_messages(result, stamp, frame, extra=None):
    hdr = header(stamp, frame)
    path = Path(header=hdr)
    yaw = result['plan'].get('heading', 0.0)
    plane = result['plane']
    for x, y, _ in result['plan']['path']:
        pose = PoseStamped(header=hdr)
        pose.pose.position = Point(x=float(x), y=float(y), z=float(plane @ [x, y, 1] + .08) if plane is not None else 0.0)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        path.poses.append(pose)
    cones = MarkerArray(markers=[Marker(header=hdr, action=Marker.DELETEALL)])
    for i, c in enumerate(result['cones']):
        marker = Marker(header=hdr, ns='cones', id=i, type=Marker.SPHERE, action=Marker.ADD)
        marker.pose.position = Point(x=float(c['xyz'][0]), y=float(c['xyz'][1]), z=float(c['xyz'][2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = .25
        marker.color.r, marker.color.g, marker.color.b = COLORS[c['color']]
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)
        cones.markers.append(marker)
    boundaries = MarkerArray(markers=[Marker(header=hdr, action=Marker.DELETEALL)])
    gate = PoseArray(header=hdr)
    for x, y in result['plan'].get('red_gate', []):
        pose = Pose()
        pose.position = Point(x=float(x), y=float(y), z=.1)
        pose.orientation.w = 1.0
        gate.poses.append(pose)
    if len(gate.poses) == 2:
        marker = Marker(header=hdr, ns='red_entry', id=3, type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.scale.x = .15
        marker.color.r, marker.color.a = 1.0, 1.0
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)
        marker.points = [p.position for p in gate.poses]
        boundaries.markers.append(marker)
    for i, side in enumerate(('left', 'right')):
        line = result['plan'][side]
        if line is None:
            continue
        marker = Marker(header=hdr, ns=side, id=i, type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.scale.x = .07
        marker.color.r, marker.color.g, marker.color.b = COLORS['blue' if side == 'left' else 'yellow']
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)
        for x in (0.0, line['x_max']):
            y = line['m'] * x + line['b']
            z = float(plane @ [x, y, 1] + .06) if plane is not None else 0.0
            marker.points.append(Point(x=float(x), y=float(y), z=z))
        boundaries.markers.append(marker)
    if path.poses:
        marker = Marker(header=hdr, ns='center', id=2, type=Marker.LINE_STRIP, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.scale.x = .10
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .1, 1.0, .5, 1.0
        marker.lifetime = Duration(sec=0, nanosec=500_000_000)
        marker.points = [pose.pose.position for pose in path.poses]
        boundaries.markers.append(marker)
    status = String(data=json.dumps({**public_result(result), **(extra or {})}, allow_nan=False))
    return {'/aeb/center_path': path, '/aeb/cones': cones, '/aeb/boundaries': boundaries,
            '/aeb/status': status, '/aeb/red_gate': gate}


def empty_result(reason):
    return {'cones': [], 'plane': None,
            'plan': {'valid': False, 'reason': reason, 'path': np.empty((0, 3)), 'left': None, 'right': None}}
