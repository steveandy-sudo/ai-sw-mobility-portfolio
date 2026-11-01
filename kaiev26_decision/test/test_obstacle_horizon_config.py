"""Offline horizon configuration checks; never constructs or starts a ROS node."""
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from kaiev26_decision.route_zone_manager_node import (
    PolylineRoute, RouteWaypoint, RouteZoneManagerNode,
)


def route_parameters():
    config = Path(__file__).parents[1] / "config" / "decision_pipeline.yaml"
    return yaml.safe_load(config.read_text())["kaiev26_route_zone_manager"]["ros__parameters"]


def straight_route():
    return PolylineRoute([RouteWaypoint(x=float(i), y=0.0) for i in range(101)])


@pytest.mark.parametrize("horizon", [35.0, 42.0, 45.0, 50.0])
def test_obstacle_horizon_remains_adjustable(horizon):
    params = route_parameters()
    data = SimpleNamespace(route=straight_route(), lookahead_distance_m=params["lookahead_distance_m"],
                           local_path_spacing_m=1.0)
    points = RouteZoneManagerNode.local_path_points(
        data, 10.0, 10.0, 0.0, 0.0, lookahead_distance_m=horizon,
    )
    assert points[0].x == pytest.approx(0.0)
    assert points[-1].x == pytest.approx(horizon)
    assert all(point.y == pytest.approx(0.0) for point in points)


def test_normal_route_horizon_is_the_lookahead_distance():
    params = route_parameters()
    data = SimpleNamespace(route=straight_route(), lookahead_distance_m=params["lookahead_distance_m"],
                           local_path_spacing_m=1.0)
    points = RouteZoneManagerNode.local_path_points(data, 10.0, 10.0, 0.0, 0.0)
    assert points[-1].x == pytest.approx(params["lookahead_distance_m"])


def test_longer_horizon_does_not_extrapolate_beyond_route_end():
    params = route_parameters()
    data = SimpleNamespace(route=straight_route(), lookahead_distance_m=params["lookahead_distance_m"],
                           local_path_spacing_m=1.0)
    points = RouteZoneManagerNode.local_path_points(
        data, 85.0, 85.0, 0.0, 0.0, lookahead_distance_m=params["obstacle_lookahead_distance_m"],
    )
    assert points[-1].x == pytest.approx(15.0)
