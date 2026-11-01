import math

import pytest
from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray, VehicleState

from kaiev26_decision.aeb_logic import AEBLogic
from kaiev26_decision.obstacle_geometry import (
    box_corners,
    ego_box_extents,
    polygon_clearance,
    rectangle_corners,
)


def obstacle(x=5.0, y=1.7, yaw=math.pi / 2):
    obj = PerceptionObject()
    obj.class_id = PerceptionObject.CLASS_OBSTACLE
    obj.pose.position.x, obj.pose.position.y = x, y
    obj.pose.orientation.z = math.sin(yaw / 2)
    obj.pose.orientation.w = math.cos(yaw / 2)
    obj.size.x, obj.size.y = 1.6, 0.3
    return obj


def test_rotated_box_preserves_crosswise_row_width():
    obj = obstacle()
    assert ego_box_extents(obj) == pytest.approx((4.85, 5.15, 0.9, 2.5))
    obj.pose.orientation.z *= 2
    obj.pose.orientation.w *= 2
    assert ego_box_extents(obj) == pytest.approx((4.85, 5.15, 0.9, 2.5))


def test_legacy_zero_quaternion_is_axis_aligned():
    obj = obstacle()
    obj.pose.orientation.z = obj.pose.orientation.w = 0.0
    assert ego_box_extents(obj) == pytest.approx((4.2, 5.8, 1.55, 1.85))


@pytest.mark.parametrize('field,value', [('size', 0.0), ('size', -1.0), ('position', float('nan'))])
def test_invalid_geometry_is_not_used(field, value):
    obj = obstacle()
    if field == 'size':
        obj.size.x = value
    else:
        obj.pose.position.x = value
    assert box_corners(obj) == ()


def test_polygon_gap_checks_real_rectangle_not_circumscribed_circle():
    body = rectangle_corners(0, 0, 0, -0.6, 1.98, 0.59)
    beside = rectangle_corners(0, 2, 0, -4, 4, 0.15)
    crossing = rectangle_corners(0, 0, math.pi / 2, -1, 1, 0.15)
    assert polygon_clearance(body, beside) == pytest.approx(1.26)
    assert polygon_clearance(body, crossing) < 0
    assert polygon_clearance(body, body) < 0


def test_aeb_uses_projected_box_extents():
    objects = PerceptionObjectArray(objects=[obstacle()])
    assert ego_box_extents(obstacle())[:2] == pytest.approx((4.85, 5.15))
    aeb = AEBLogic.__new__(AEBLogic)
    aeb.obstacle_path_half_width_m = 1.4
    speed = VehicleState(speed_mps=5.0)
    assert aeb.compute_front_risk(objects, speed) == pytest.approx((4.85, 0.97))


def test_size_unknown_observation_keeps_legacy_front_risk_detection():
    obj = obstacle(y=0.0, yaw=0.0)
    obj.size.x = obj.size.y = 0.0
    assert box_corners(obj) == ()
    assert ego_box_extents(obj) == pytest.approx((4.95, 5.05, -0.05, 0.05))
