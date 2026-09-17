"""Unit tests for curvature feedforward helpers."""
import math

from geometry_msgs.msg import Point

from kaiev26_motion_control.motion_control_node import (
    curvature_feedforward_steering,
    reference_curvature_1pm,
)


def point(x: float, y: float) -> Point:
    msg = Point()
    msg.x = x
    msg.y = y
    return msg


def left_arc(radius_m: float, length_m: int = 12) -> list[Point]:
    return [
        point(
            radius_m * math.sin(distance / radius_m),
            radius_m * (1.0 - math.cos(distance / radius_m)),
        )
        for distance in range(length_m + 1)
    ]


def test_feedforward_zero_on_straight_curvature():
    assert curvature_feedforward_steering(0.0, 1.2991017929, 1.0, 0.20, 0.30) == 0.0


def test_feedforward_matches_bicycle_model_inside_limits():
    curvature = 0.05
    wheelbase = 1.2991017929
    steering = curvature_feedforward_steering(
        curvature,
        wheelbase,
        1.0,
        0.20,
        0.30,
    )
    assert math.isclose(steering, math.atan(wheelbase * curvature), rel_tol=1.0e-9)


def test_feedforward_respects_steering_limit():
    steering = curvature_feedforward_steering(1.0, 1.2991017929, 1.0, 1.0, 0.10)
    assert math.isclose(steering, 0.10)


def test_reference_curvature_detects_left_arc():
    curvature = reference_curvature_1pm(
        left_arc(10.0),
        preview_distance_m=3.0,
        sample_half_width_m=2.0,
    )
    assert curvature > 0.0
    assert math.isclose(curvature, 0.1, rel_tol=0.25)
