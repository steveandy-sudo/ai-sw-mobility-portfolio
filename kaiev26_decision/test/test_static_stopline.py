from kaiev26_decision.route_zone_manager_node import (
    INF_DISTANCE,
    PolylineRoute,
    RouteWaypoint,
    distance_to_next_stopline,
    parse_route_stoplines,
)


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
