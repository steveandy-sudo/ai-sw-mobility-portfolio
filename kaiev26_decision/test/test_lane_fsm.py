from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext, SceneSummary

from kaiev26_decision.scenario_modules.lane_fsm import LaneFSM


def make_route(cross_track_error: float, heading_error: float) -> RouteContext:
    route = RouteContext()
    route.route_projection_valid = True
    route.cross_track_error = cross_track_error
    route.heading_error = heading_error
    route.local_path_points = [Point(x=0.0), Point(x=10.0)]
    return route


def test_route_rejoin_speed_is_fast_near_recovery_threshold() -> None:
    fsm = LaneFSM(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        rejoin_speed_min_mps=1.4,
        rejoin_speed_max_mps=2.6,
        rejoin_slow_cross_track_m=2.5,
    )

    decision = fsm.update(SceneSummary(), make_route(0.81, 0.0), None)

    assert decision.active_behavior == "ROUTE_REJOIN"
    assert 2.5 < decision.target_speed_limit_mps <= 2.6


def test_route_rejoin_speed_slows_for_large_route_error() -> None:
    fsm = LaneFSM(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        rejoin_speed_min_mps=1.4,
        rejoin_speed_max_mps=2.6,
        rejoin_slow_cross_track_m=2.5,
        rejoin_slow_heading_error_rad=0.8,
    )

    large_cte = fsm.update(SceneSummary(), make_route(3.0, 0.0), None)
    large_heading = fsm.update(SceneSummary(), make_route(0.0, 0.9), None)

    assert large_cte.target_speed_limit_mps == 1.4
    assert large_heading.target_speed_limit_mps == 1.4


def test_manual_resume_rejoin_stays_latched_until_small_error() -> None:
    fsm = LaneFSM(base_speed_mps=4.0, degraded_speed_mps=0.8)
    fsm.request_route_rejoin()

    initial = fsm.update(SceneSummary(), make_route(0.5, 0.1), None)
    still_rejoining = fsm.update(SceneSummary(), make_route(0.4, 0.1), None)
    complete = fsm.update(SceneSummary(), make_route(0.2, 0.1), None)

    assert initial.active_behavior == "ROUTE_REJOIN"
    assert still_rejoining.active_behavior == "ROUTE_REJOIN"
    assert complete.active_behavior == "GLOBAL_ROUTE_FOLLOW"
