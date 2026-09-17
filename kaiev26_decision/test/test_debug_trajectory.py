import pytest

from kaiev26_decision.debug_monitor_node import kinematic_trajectory_points


def test_kinematic_trajectory_is_straight_at_zero_steering() -> None:
    points = kinematic_trajectory_points(2.0, 0.0, 1.3, 3.0, 0.1)

    assert points[0].x == pytest.approx(0.0)
    assert points[-1].x == pytest.approx(6.0)
    assert all(point.y == pytest.approx(0.0) for point in points)


def test_kinematic_trajectory_preserves_steering_sign() -> None:
    left = kinematic_trajectory_points(2.0, 0.2, 1.3, 2.0, 0.1)
    right = kinematic_trajectory_points(2.0, -0.2, 1.3, 2.0, 0.1)

    assert left[-1].x == pytest.approx(right[-1].x)
    assert left[-1].y == pytest.approx(-right[-1].y)
    assert left[-1].y > 0.0
