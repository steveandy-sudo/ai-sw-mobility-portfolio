import math

import pytest
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from kaiev26_decision.obstacle_observation import ObstacleObservationBuffer
from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner, Row


def header(t, frame='base_footprint'):
    value = Header(frame_id=frame)
    value.stamp.sec = int(t)
    value.stamp.nanosec = round((t-int(t))*1e9)
    return value


def odom(t, x=0.0, y=0.0, yaw=0.0):
    value = Odometry(header=header(t, 'map'), child_frame_id='base_footprint')
    value.pose.pose.position.x, value.pose.pose.position.y = float(x), float(y)
    value.pose.pose.orientation.z, value.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return value


def test_interpolates_wrapped_heading_and_bounds_extrapolation():
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0, yaw=math.radians(179)))
    buffer.add_odometry(odom(10.2, x=0.4, yaw=math.radians(-179)))
    assert buffer.pose_at(10.1).yaw == pytest.approx(math.pi)
    assert buffer.pose_at(10.25).x == pytest.approx(0.5)
    assert buffer.pose_at(10.4) is None
    assert buffer.pose_at(9.9) is None


def test_duplicate_or_older_observation_does_not_refresh_row_memory():
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98,
                                      clearance_m=0.5, row_memory_sec=0.8)
    row = Row(s=10.0, low=0.2, high=1.4, front=9.7, rear=10.3, members=3)
    planner.tracked_rows([row], 100.0, 10.0, 30.0, observation_stamp_sec=10.0)
    for now in (10.1, 10.2, 10.5):
        planner.tracked_rows([row], 100.0, now, 30.0, observation_stamp_sec=10.0)
    planner.tracked_rows([row], 100.0, 10.6, 30.0, observation_stamp_sec=9.9)
    assert len(planner.row_tracks[0].samples) == 1
    assert planner.row_tracks[0].last_seen_sec == 10.0
    assert planner.tracked_rows([row], 100.0, 10.9, 30.0, observation_stamp_sec=10.0) == []


def test_empty_new_scan_does_not_erase_unexpired_row():
    planner = ObstacleSequencePlanner(vehicle_width_m=1.18, front_overhang_m=1.98, clearance_m=0.5)
    row = Row(s=10.0, low=0.2, high=1.4, front=9.7, rear=10.3, members=3)
    planner.tracked_rows([row], 100.0, 10.0, 30.0, observation_stamp_sec=10.0)
    assert len(planner.tracked_rows([], 100.2, 10.1, 30.0, observation_stamp_sec=10.1)) == 1


def test_odometry_frame_reset_and_out_of_order_handling():
    buffer = ObstacleObservationBuffer()
    buffer.add_odometry(odom(10.0))
    buffer.add_odometry(odom(10.2))
    buffer.add_odometry(odom(10.1))
    assert len(buffer.samples) == 2
    changed = odom(10.3)
    changed.header.frame_id = 'odom'
    buffer.add_odometry(changed)
    assert len(buffer.samples) == 1
    assert buffer.pose_at(10.0) is None
