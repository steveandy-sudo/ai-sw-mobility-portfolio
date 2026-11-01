"""Offline regression tests for time-consistent cached-path reuse (no ROS nodes)."""
from copy import deepcopy
import json
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import Centerline, RouteContext
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import Header

from kaiev26_decision.main_planning_engine_node import MainPlanningEngineNode
from kaiev26_decision.obstacle_observation import ObstacleObservationBuffer


def header(t, frame='base_footprint'):
    result = Header(frame_id=frame)
    ns = round(t * 1_000_000_000)
    result.stamp.sec, result.stamp.nanosec = divmod(ns, 1_000_000_000)
    return result


def path(t=10.0):
    return Centerline(
        header=header(t), detection_id=1, track_id=7, confidence=0.85,
        points=[Point(x=10.0, y=1.0, z=0.2), Point(x=20.0, y=2.0, z=0.3)],
        source='relaxed_sequential_bypass_1_rows',
    )


def odom(t, x=0.0, y=0.0, z=0.0, yaw=0.0, parent='map', child='base_footprint'):
    result = Odometry(header=header(t, parent), child_frame_id=child)
    result.pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
    result.pose.pose.orientation.z = math.sin(yaw / 2)
    result.pose.pose.orientation.w = math.cos(yaw / 2)
    return result


def engine(buffer=None, previous=None):
    """Only the route-follow state of the engine; no ROS node is started."""
    node = MainPlanningEngineNode.__new__(MainPlanningEngineNode)
    node.base_speed_mps = 6.0
    node.odometry = buffer if buffer is not None else ObstacleObservationBuffer()
    node.previous_path = previous
    return node


def hold(node, t):
    output = node.route_follow(None, False, t if isinstance(t, Time) else Time(seconds=t))
    assert output.path_request == ('HOLD_PREVIOUS_PATH' if node.previous_path is not None else '')
    return output.path


def test_without_odometry_keeps_original_timestamp_instead_of_restamping():
    core = engine(previous=path())
    saved = deepcopy(core.previous_path)
    result = hold(core, 10.6)
    assert result.header == saved.header
    assert result.points == saved.points
    assert result.source == 'previous_safe_path'
    assert result.confidence == 0.5
    assert (result.detection_id, result.track_id) == (1, 7)
    result.points[0].x = -99.0
    result.header.stamp.sec = 99
    assert core.previous_path == saved


def core_with_history():
    buffer = ObstacleObservationBuffer()
    for t, x in [(10.0, 0.0), (10.2, 1.0), (10.4, 2.0), (10.6, 3.0)]:
        buffer.add_odometry(odom(t, x=x))
    return engine(buffer, path()), buffer


def test_three_metre_ego_translation_rebases_ten_metre_target_to_seven():
    core, _ = core_with_history()
    original = deepcopy(core.previous_path)
    result = hold(core, 10.6)
    assert (result.points[0].x, result.points[0].y) == pytest.approx((7.0, 1.0))
    assert result.header == header(10.6)
    assert result.source == 'previous_safe_path'
    assert result.confidence == 0.5
    assert core.previous_path == original


def test_repeated_fallback_uses_original_anchor_without_accumulating_motion():
    core, _ = core_with_history()
    for t, x in [(10.2, 9.0), (10.4, 8.0), (10.6, 7.0), (10.6, 7.0)]:
        result = hold(core, t)
        assert result.points[0].x == pytest.approx(x)
        assert core.previous_path.header == header(10.0)
        assert core.previous_path.points[0].x == 10.0


@pytest.mark.parametrize('source_yaw,target_yaw', [
    (0.0, math.pi/2), (0.6, -0.4), (math.radians(179), math.radians(-179)),
])
def test_reprojection_preserves_world_points_including_translation_rotation_and_height(source_yaw, target_yaw):
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0, x=2.0, y=4.0, z=1.0, yaw=source_yaw))
    buffer.add_odometry(odom(10.2, x=3.0, y=6.0, z=1.3, yaw=target_yaw))
    original = path()
    aligned = buffer.align_path(original, header(10.2))
    assert aligned is not None
    for old, new in zip(original.points, aligned.points):
        old_world = (2+math.cos(source_yaw)*old.x-math.sin(source_yaw)*old.y,
                     4+math.sin(source_yaw)*old.x+math.cos(source_yaw)*old.y, 1+old.z)
        new_world = (3+math.cos(target_yaw)*new.x-math.sin(target_yaw)*new.y,
                     6+math.sin(target_yaw)*new.x+math.cos(target_yaw)*new.y, 1.3+new.z)
        assert new_world == pytest.approx(old_world, abs=1e-12)


def test_rotation_sign_moves_forward_point_right_when_vehicle_turns_left():
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0))
    buffer.add_odometry(odom(10.2, yaw=math.pi/2))
    result = buffer.align_path(path(), header(10.2))
    assert (result.points[0].x, result.points[0].y) == pytest.approx((1.0, -10.0))


def test_interpolates_both_poses_and_allows_only_bounded_target_extrapolation():
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0))
    buffer.add_odometry(odom(10.2, x=1.0))
    original = path(10.1)
    result = buffer.align_path(original, header(10.25))
    assert result.points[0].x == pytest.approx(9.25)
    assert result.header == header(10.25)
    assert buffer.align_path(original, header(10.31)) is None
    assert buffer.align_path(path(10.21), header(10.25)) is None


@pytest.mark.parametrize('case', ['empty', 'missing_source', 'missing_target', 'gap',
                                  'frame_reset', 'clock_reset', 'different_child'])
def test_unavailable_transform_preserves_old_header_so_target_age_is_not_hidden(case):
    buffer = ObstacleObservationBuffer()
    if case != 'empty':
        if case == 'missing_source':
            buffer.add_odometry(odom(10.1))
        else:
            buffer.add_odometry(odom(10.0))
        if case not in {'missing_target', 'gap'}:
            buffer.add_odometry(odom(10.2, x=1.0))
        if case == 'gap':
            buffer.add_odometry(odom(10.5, x=2.5))
        elif case == 'frame_reset':
            buffer.add_odometry(odom(10.3, parent='odom'))
        elif case == 'clock_reset':
            buffer.add_odometry(odom(8.0))
        elif case == 'different_child':
            buffer.add_odometry(odom(10.3, child='base_link'))
    core = engine(buffer, path())
    result = hold(core, 10.1 if case == 'gap' else 10.3)
    assert result.header == header(10.0)
    assert result.points == core.previous_path.points
    assert core.previous_path.header == header(10.0)


def test_expired_source_pose_does_not_extend_cache_lifetime():
    core, buffer = core_with_history()
    for i in range(1, 31):
        buffer.add_odometry(odom(10.6+i*0.1, x=3.0+i*0.5))
    assert buffer.pose_at(10.0) is None
    result = hold(core, 13.6)
    assert result.header == header(10.0)
    assert result.points == core.previous_path.points


@pytest.mark.parametrize('frame', ['', 'base_link', 'map'])
def test_different_or_empty_target_frame_cannot_relabel_old_coordinates(frame):
    _, buffer = core_with_history()
    assert buffer.align_path(path(), header(10.6, frame)) is None


def test_slash_normalization_and_stationary_vehicle_still_return_independent_messages():
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0))
    buffer.add_odometry(odom(10.2))
    original = path()
    original.header.frame_id = '/base_footprint'
    target = header(10.2)
    result = buffer.align_path(original, target)
    assert result.header == target and result.points == original.points
    result.header.stamp.sec = 20
    result.points[0].x = -10.0
    assert original.header == header(10.0, '/base_footprint')
    assert original.points[0].x == 10.0
    assert target == header(10.2)


def test_same_timestamp_needs_no_transform_but_cannot_alias_the_anchor():
    buffer = ObstacleObservationBuffer()
    original = path()
    result = buffer.align_path(original, header(10.0))
    assert result == original and result is not original
    result.points[0].x = -1.0
    assert original.points[0].x == 10.0


@pytest.mark.parametrize('case', ['zero_stamp', 'backwards_time', 'nan', 'one_point'])
def test_invalid_or_time_reversed_paths_are_not_restamped(case):
    _, buffer = core_with_history()
    original, target = path(), header(10.2)
    if case == 'zero_stamp':
        original.header = header(0.0)
    elif case == 'backwards_time':
        target = header(9.9)
    elif case == 'nan':
        original.points[0].x = float('nan')
    else:
        original.points = original.points[:1]
    assert buffer.align_path(original, target) is None


def test_no_previous_path_requests_no_path():
    core, _ = core_with_history()
    core.previous_path = None
    assert hold(core, 10.6) is None


def test_new_valid_route_path_does_not_use_cached_path_reprojection():
    core, buffer = core_with_history()
    buffer.align_path = lambda *_args: pytest.fail('new route path must not use cached-path reprojection')
    route = RouteContext(header=header(10.6), route_projection_valid=True,
                         projection_confidence=0.9, local_path_points=[Point(), Point(x=30.0)])
    output = core.route_follow(route, True, Time(seconds=10.6))
    assert output.path_request == 'ROUTE_LOCAL_PATH'
    assert output.path.source == 'global_route_local_path'
    assert output.path.points == route.local_path_points


def test_reprojection_does_not_change_the_route_follow_speed():
    core, _ = core_with_history()
    unchanged = engine(previous=path())
    fixed = core.route_follow(None, False, Time(seconds=10.6))
    old = unchanged.route_follow(None, False, Time(seconds=10.6))
    assert fixed.path.points[0].x == pytest.approx(7.0)
    assert fixed.cap_mps == old.cap_mps == 6.0


@pytest.mark.parametrize('causal_only', [False, True])
def test_recorded_145703_snapshot_keeps_world_geometry(causal_only):
    data = json.loads((Path(__file__).parent / 'fixtures' / 'previous_safe_path_145703.json').read_text())

    def recorded_header(value):
        result = Header(frame_id=value['frame_id'])
        result.stamp.sec = value['sec']
        result.stamp.nanosec = value['nanosec']
        return result

    target = recorded_header(data['reused_header'])
    target_ns = target.stamp.sec * 1_000_000_000 + target.stamp.nanosec
    buffer = ObstacleObservationBuffer()
    for sample in data['odometry']:
        sample_ns = sample['header']['sec'] * 1_000_000_000 + sample['header']['nanosec']
        if causal_only and sample_ns > target_ns:
            continue  # Production may need bounded extrapolation, not a future sample.
        msg = Odometry(header=recorded_header(sample['header']), child_frame_id=sample['child_frame_id'])
        msg.pose.pose.position = Point(**dict(zip(('x','y','z'), map(float, sample['position']))))
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = map(float, sample['quaternion'])
        buffer.add_odometry(msg)
    saved = Centerline(header=recorded_header(data['saved_header']), detection_id=1,
                       confidence=0.85, source='relaxed_sequential_bypass_1_rows',
                       points=[Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in data['points']])
    original = deepcopy(saved)
    core = engine(buffer, saved)
    corrected = hold(core, Time(seconds=target.stamp.sec, nanoseconds=target.stamp.nanosec))
    assert corrected.header == target
    assert corrected.points[2].x == pytest.approx(6.932, abs=0.001)

    def world(point, pose):
        return (pose.x+math.cos(pose.yaw)*point.x-math.sin(pose.yaw)*point.y,
                pose.y+math.sin(pose.yaw)*point.x+math.cos(pose.yaw)*point.y,
                pose.z+point.z)

    from kaiev26_decision.obstacle_observation import stamp_seconds
    source_pose = buffer.pose_at(stamp_seconds(saved.header.stamp))
    target_pose = buffer.pose_at(stamp_seconds(target.stamp))
    for old, new in zip(saved.points, corrected.points):
        assert world(new, target_pose) == pytest.approx(world(old, source_pose), abs=1e-10)
    assert core.previous_path == original
