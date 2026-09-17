from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext, SceneSummary

from kaiev26_decision.planning_core import PlanningCore
from kaiev26_decision.priority_selector import PrioritySelector
from kaiev26_decision.route_zone_manager_node import (
    INF_DISTANCE,
    PolylineRoute,
    RouteWaypoint,
    distance_to_next_stopline,
    parse_route_stoplines,
)
from kaiev26_decision.scenario_modules.static_stop_fsm import StaticStopFSM


def make_route() -> PolylineRoute:
    return PolylineRoute(
        [
            RouteWaypoint(0.0, 0.0),
            RouteWaypoint(10.0, 0.0),
            RouteWaypoint(20.0, 0.0),
        ]
    )


def test_stopline_can_use_waypoint_or_xy_and_is_sorted() -> None:
    stoplines = parse_route_stoplines(
        make_route(),
        {
            "landmarks": [
                {"id": "second", "type": "STOP_LINE", "waypoint": 3},
                {"id": "first", "type": "STOP_LINE", "x": 8.0, "y": 0.2},
                {"id": "ignored", "type": "CROSSWALK", "waypoint": 2},
            ]
        },
    )

    assert [item.landmark_id for item in stoplines] == ["first", "second"]
    assert abs(stoplines[0].progress_s - 8.0) < 1e-6


def test_passed_stopline_is_not_selected_again() -> None:
    stoplines = parse_route_stoplines(
        make_route(),
        {"landmarks": [{"type": "STOP_LINE", "route_s": 10.0}]},
    )

    assert distance_to_next_stopline(7.0, stoplines) == 3.0
    assert distance_to_next_stopline(10.1, stoplines) == 0.0
    assert distance_to_next_stopline(10.3, stoplines) == INF_DISTANCE


def test_static_stop_is_opt_in_and_uses_route_distance() -> None:
    route = RouteContext()
    route.route_projection_valid = True
    route.distance_to_stopline = 7.0

    disabled = PrioritySelector(18.0, static_stop_enabled=False)
    enabled = PrioritySelector(18.0, static_stop_enabled=True)

    assert disabled.choose(SceneSummary(), route, None, False).behavior == "LANE_FOLLOW"
    assert enabled.choose(SceneSummary(), route, None, False).behavior == "STATIC_STOP"


def test_static_stop_path_prefers_global_route() -> None:
    route = RouteContext()
    route.route_projection_valid = True
    route.projection_confidence = 0.9
    route.distance_to_stopline = 8.0
    route.local_path_points = [Point(x=0.0, y=0.0), Point(x=10.0, y=0.0)]

    scene = SceneSummary()
    scene.centerline_valid = True
    scene.centerline_quality = 1.0
    scene.centerline_points = [Point(x=0.0, y=3.0), Point(x=10.0, y=3.0)]

    fsm = StaticStopFSM(0.6, 18.0, 1.98, 1.0)
    decision = fsm.update(scene, route, None)
    path, speed = PlanningCore(0.6, 0.6).make_plan(
        decision, scene, route, []
    )

    assert abs(decision.stop_target_distance - 5.02) < 1e-6
    assert path.source == "longitudinal_stop_from_global_route_local_path"
    assert all(abs(point.y) < 1e-6 for point in path.points)
    assert speed.need_stop
