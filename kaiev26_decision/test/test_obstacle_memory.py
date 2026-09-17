from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand
from kaiev26_decision.planning_core import PlanningCore
from kaiev26_decision.scenario_modules.obstacle_fsm import ObstacleFSM
from kaiev26_decision.scenario_modules.traffic_light_fsm import TrafficLightFSM


def make_scene(
    obstacle: bool,
    x: float = 10.0,
    y: float = 1.0,
    length: float = 2.0,
    width: float = 0.9,
) -> SceneSummary:
    scene = SceneSummary()
    scene.obstacle_on_path = obstacle
    scene.front_obstacle_distance = max(0.0, x - length * 0.5)
    scene.front_obstacle_x = x
    scene.front_obstacle_y = y
    scene.front_obstacle_length = length
    scene.front_obstacle_width = width
    scene.obstacle_risk_level = "LOW"
    scene.centerline_valid = False
    scene.left_lane_center_valid = True
    scene.left_lane_center_points = [point(float(px), 3.2) for px in range(0, 25, 2)]
    scene.right_lane_center_valid = True
    scene.right_lane_center_points = [point(float(px), -3.2) for px in range(0, 25, 2)]
    scene.lane_width_estimate_m = 3.5
    return scene


def make_route(progress_s: float) -> RouteContext:
    route = RouteContext()
    route.progress_s = progress_s
    route.route_projection_valid = True
    route.projection_confidence = 0.9
    route.local_path_points = [point(float(x), 0.0) for x in range(0, 25, 2)]
    return route


def point(x: float, y: float) -> Point:
    msg = Point()
    msg.x = x
    msg.y = y
    return msg


def test_obstacle_fsm_keeps_bypass_until_route_progress_clears_memory() -> None:
    fsm = ObstacleFSM(slow_speed_mps=0.7)

    first = fsm.update(make_scene(True, x=10.0), make_route(100.0), None)
    assert first.fsm_state == "BYPASS_READY"
    assert first.path_request == "BYPASS_PATH"

    remembered = fsm.update(make_scene(False), make_route(105.0), None)
    assert remembered.fsm_state == "BYPASS_HOLD"
    assert remembered.path_request == "BYPASS_PATH"
    assert "OBSTACLE_MEMORY" in remembered.constraints

    cleared = fsm.update(make_scene(False), make_route(113.0), None)
    assert cleared.fsm_state == "RETURN"
    assert cleared.path_request == "CENTERLINE"


def test_obstacle_fsm_uses_faster_speed_only_after_bypass_clears() -> None:
    fsm = ObstacleFSM(slow_speed_mps=2.2, return_speed_mps=2.6)

    bypass = fsm.update(make_scene(True, x=10.0), make_route(100.0), None)
    assert bypass.target_speed_limit_mps == 2.2

    fsm.update(make_scene(False), make_route(113.0), None)
    returning = fsm.update(make_scene(False), make_route(113.1), None)
    assert returning.fsm_state == "RETURN"
    assert returning.target_speed_limit_mps == 2.6


def test_obstacle_memory_scales_with_detected_obstacle_length() -> None:
    short_fsm = ObstacleFSM(slow_speed_mps=0.7)
    short_fsm.update(make_scene(True, x=10.0, length=2.0), make_route(100.0), None)
    assert short_fsm.update(make_scene(False), make_route(113.0), None).fsm_state == "RETURN"

    long_fsm = ObstacleFSM(slow_speed_mps=0.7)
    long_fsm.update(make_scene(True, x=10.0, length=6.0), make_route(100.0), None)
    assert long_fsm.update(make_scene(False), make_route(113.0), None).fsm_state == "BYPASS_HOLD"
    assert long_fsm.update(make_scene(False), make_route(115.0), None).fsm_state == "RETURN"


def test_obstacle_fsm_only_stops_at_hard_close_distance() -> None:
    fsm = ObstacleFSM(slow_speed_mps=0.7)

    bypass = fsm.update(make_scene(True, x=1.8, length=2.0), make_route(100.0), None)
    assert bypass.fsm_state == "BYPASS_READY"
    assert bypass.path_request == "BYPASS_PATH"

    blocked = fsm.update(make_scene(True, x=1.4, length=2.0), make_route(100.0), None)
    assert blocked.fsm_state == "BLOCKED_STOP"
    assert blocked.path_request == "STOP_PATH"


def test_obstacle_fsm_keeps_bypassing_when_lateral_clearance_is_safe() -> None:
    fsm = ObstacleFSM(
        slow_speed_mps=1.5,
        vehicle_width_m=1.18,
        obstacle_clearance_m=0.30,
    )

    bypass = fsm.update(
        make_scene(True, x=2.5, y=-1.9, length=4.5, width=1.8),
        make_route(100.0),
        None,
    )

    assert bypass.fsm_state == "BYPASS_READY"
    assert bypass.path_request == "BYPASS_PATH"
    assert bypass.need_stop is False


def test_planning_core_reuses_remembered_obstacle_for_bypass_path() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    decision = DecisionCommand(path_request="BYPASS_PATH")

    first_path, _ = core.make_plan(decision, make_scene(True, x=10.0, y=1.0), make_route(100.0), [])
    assert first_path.source == "local_bypass_min_clearance_right_from_global_route_local_path"
    assert -0.4 < min(point.y for point in first_path.points) < -0.3

    remembered_path, _ = core.make_plan(decision, make_scene(False), make_route(104.0), [])
    assert remembered_path.source == (
        "local_bypass_memory_min_clearance_right_from_global_route_local_path"
    )
    assert -0.4 < min(point.y for point in remembered_path.points) < -0.3


def test_planning_core_routes_right_obstacle_to_left_lane_center() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    decision = DecisionCommand(path_request="BYPASS_PATH")

    path, _ = core.make_plan(decision, make_scene(True, x=10.0, y=-1.0), make_route(100.0), [])
    assert path.source == "local_bypass_min_clearance_left_from_global_route_local_path"
    assert 0.3 < max(point.y for point in path.points) < 0.4


def test_planning_core_locks_centered_obstacle_bypass_to_preferred_left() -> None:
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        obstacle_side_deadband_m=0.30,
        obstacle_preferred_side="left",
    )
    decision = DecisionCommand(path_request="BYPASS_PATH")

    first_path, _ = core.make_plan(
        decision,
        make_scene(True, x=14.0, y=-0.05),
        make_route(100.0),
        [],
    )
    flipped_observation_path, _ = core.make_plan(
        decision,
        make_scene(True, x=13.5, y=0.05),
        make_route(100.5),
        [],
    )

    assert "min_clearance_left" in first_path.source
    assert "min_clearance_left" in flipped_observation_path.source
    assert 1.3 < max(point.y for point in flipped_observation_path.points) < 1.5


def test_planning_core_keeps_locked_side_when_lane_center_temporarily_drops() -> None:
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        obstacle_preferred_side="left",
    )
    decision = DecisionCommand(path_request="BYPASS_PATH")
    core.make_plan(decision, make_scene(True, x=14.0, y=0.0), make_route(100.0), [])

    dropped = make_scene(True, x=13.5, y=0.1)
    dropped.left_lane_center_valid = False
    dropped.left_lane_center_points = []
    path, _ = core.make_plan(decision, dropped, make_route(100.5), [])

    assert "min_clearance_offset" in path.source
    assert 1.4 < max(point.y for point in path.points) < 1.5


def test_planning_core_builds_gradual_bypass_toward_obstacle() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    decision = DecisionCommand(path_request="BYPASS_PATH")

    path, _ = core.make_plan(decision, make_scene(True, x=14.0, y=1.0), make_route(100.0), [])
    assert path.source == "local_bypass_min_clearance_right_from_global_route_local_path"

    point_at_four_m = next(point for point in path.points if point.x == 4.0)
    point_at_twelve_m = next(point for point in path.points if point.x == 12.0)
    assert point_at_four_m.y > -0.3
    assert -0.4 < point_at_twelve_m.y < -0.3


def test_obstacle_bypass_applies_slow_speed_limit() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    decision = DecisionCommand(
        active_behavior="OBSTACLE",
        path_request="BYPASS_PATH",
        target_speed_limit_mps=1.5,
    )
    scene = make_scene(True, x=14.0, y=1.0)
    scene.perception_health = "DEGRADED"
    scene.events = ["CENTERLINE_INVALID", "OBSTACLE_ON_PATH"]

    _, speed = core.make_plan(decision, scene, make_route(100.0), [])

    assert speed.speed_limit_mps == 1.5
    assert speed.target_speed_mps == 1.5
    assert "OBSTACLE_PRESENT" in speed.constraints
    assert "PERCEPTION_DEGRADED" not in speed.constraints


def test_signal_passing_uses_route_without_centerline_degraded_speed_cap() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    decision = DecisionCommand(
        active_behavior="TRAFFIC_LIGHT",
        fsm_state="PASSING",
        path_request="ROUTE_LOCAL_PATH",
        target_speed_limit_mps=4.0,
    )
    scene = make_scene(False)
    scene.perception_health = "DEGRADED"
    scene.events = ["CENTERLINE_INVALID", "STOPLINE_NEAR"]

    _, speed = core.make_plan(decision, scene, make_route(100.0), [])

    assert speed.speed_limit_mps == 4.0
    assert "PERCEPTION_DEGRADED" not in speed.constraints


def test_lane_center_bypass_keeps_vehicle_clear_of_wide_obstacle() -> None:
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        vehicle_width_m=1.18,
        obstacle_clearance_m=0.30,
    )
    decision = DecisionCommand(path_request="BYPASS_PATH")
    scene = make_scene(True, x=14.0, y=-0.02, length=4.5, width=1.8)
    scene.left_lane_center_points = [point(float(px), 1.73) for px in range(0, 25, 2)]

    path, _ = core.make_plan(decision, scene, make_route(100.0), [])
    obstacle_front_x = 14.0 - 4.5 * 0.5
    first_longitudinal_overlap_x = obstacle_front_x - 1.98
    path_at_overlap = core.interpolate_route_point(path.points, first_longitudinal_overlap_x)
    required_clearance = 1.18 * 0.5 + 1.8 * 0.5 + 0.30

    assert path_at_overlap is not None
    assert path.source == "local_bypass_min_clearance_left_from_global_route_local_path"
    assert path_at_overlap.y - scene.front_obstacle_y >= required_clearance - 1.0e-6
    assert 1.7 < max(point.y for point in path.points) < 2.0


def test_fallback_bypass_can_leave_lane_for_required_clearance() -> None:
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        vehicle_width_m=1.18,
        obstacle_clearance_m=0.30,
        obstacle_lane_departure_allowed=True,
    )
    decision = DecisionCommand(path_request="BYPASS_PATH")
    scene = make_scene(True, x=14.0, y=0.0, length=4.5, width=1.8)
    scene.left_lane_center_valid = False
    scene.left_lane_center_points = []
    scene.right_lane_center_valid = False
    scene.right_lane_center_points = []

    path, _ = core.make_plan(decision, scene, make_route(100.0), [])
    path_at_obstacle = core.interpolate_route_point(path.points, scene.front_obstacle_x)
    required_clearance = 1.18 * 0.5 + 1.8 * 0.5 + 0.30

    assert path_at_obstacle is not None
    assert "min_clearance_offset" in path.source
    assert abs(path_at_obstacle.y - scene.front_obstacle_y) >= required_clearance - 1.0e-6


def test_bypass_weight_has_smooth_transition_endpoints() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)

    assert core.quintic_smoothstep(0.0) == 0.0
    assert core.quintic_smoothstep(1.0) == 1.0
    assert core.quintic_smoothstep(0.01) < 0.001
    assert core.quintic_smoothstep(0.99) > 0.999


def test_bypass_offset_follows_path_normal_through_curve() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    base_path = [
        point(0.0, 0.0),
        point(5.0, 0.0),
        point(10.0, 5.0),
        point(10.0, 10.0),
        point(10.0, 15.0),
    ]

    shifted = core.blend_to_normal_offset_path(
        base_path,
        lateral_shift_m=2.0,
        approach_start=0.0,
        hold_start=5.0,
        hold_end=30.0,
        return_end=40.0,
    )

    # 세로 구간의 왼쪽 법선은 -x 방향이다. 고정 y 이동이면 이 검사를 통과할 수 없다.
    assert abs(shifted[3].x - 8.0) < 1.0e-6
    assert abs(shifted[3].y - 10.0) < 1.0e-6


def test_obstacle_projection_uses_path_station_and_lateral_offset() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    base_path = [point(0.0, 0.0), point(5.0, 0.0), point(10.0, 5.0)]

    path_s, lateral_m = core.project_point_to_path(base_path, 6.0, 2.0)

    assert 7.0 < path_s < 7.2
    assert 0.6 < lateral_m < 0.8


def test_traffic_light_stop_target_keeps_vehicle_front_before_stopline() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=10.0,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=0.5,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 10.0

    decision = fsm.update(scene, None, None)

    assert decision.fsm_state == "DECELERATE"
    assert decision.need_stop is True
    assert abs(decision.stop_target_distance - 7.52) < 1e-6


def test_traffic_light_deceleration_uses_distance_curve_without_low_speed_cap() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=0.5,
    )
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        comfortable_decel_mps2=1.4,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=0.5,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 7.0

    decision = fsm.update(scene, None, None)
    _, speed = core.make_plan(decision, scene, make_route(100.0), [])

    assert decision.fsm_state == "DECELERATE"
    assert decision.need_stop is True
    assert decision.target_speed_limit_mps == 4.0
    assert speed.need_stop is True
    assert 3.0 < speed.target_speed_mps < 4.0


def test_traffic_light_hold_uses_vehicle_front_stop_target_not_line_center() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=10.0,
        stop_hold_distance_m=1.0,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=0.5,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 3.0

    decision = fsm.update(scene, None, None)

    assert decision.fsm_state == "STOP_HOLD"
    assert decision.stop_target_distance == 0.0


def test_traffic_light_release_requires_continuous_green_time() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.4,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 2.0
    scene.header.stamp.sec = 10
    assert fsm.update(scene, None, None).fsm_state == "STOP_HOLD"

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    scene.header.stamp.nanosec = 100_000_000
    assert fsm.update(scene, None, None).fsm_state == "WAIT_GREEN"

    scene.header.stamp.nanosec = 400_000_000
    assert fsm.update(scene, None, None).fsm_state == "WAIT_GREEN"

    scene.header.stamp.nanosec = 500_000_000
    assert fsm.update(scene, None, None).fsm_state == "PASSING"


def test_traffic_light_release_timer_resets_when_green_is_interrupted() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.4,
    )
    scene = SceneSummary()
    scene.stopline_distance = 2.0
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.header.stamp.sec = 20
    fsm.update(scene, None, None)

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    scene.header.stamp.nanosec = 100_000_000
    fsm.update(scene, None, None)
    scene.traffic_light_state = SceneSummary.TRAFFIC_UNKNOWN
    scene.header.stamp.nanosec = 300_000_000
    assert fsm.update(scene, None, None).fsm_state == "WAIT_GREEN"

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    scene.header.stamp.nanosec = 600_000_000
    assert fsm.update(scene, None, None).fsm_state == "WAIT_GREEN"
    scene.header.stamp.sec = 21
    scene.header.stamp.nanosec = 0
    assert fsm.update(scene, None, None).fsm_state == "PASSING"


def test_traffic_light_arrow_releases_only_left_maneuver() -> None:
    scene = SceneSummary()
    scene.stopline_distance = 2.0
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    route = make_route(100.0)

    left_fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.0,
    )
    left_fsm.update(scene, route, None)
    scene.traffic_light_state = SceneSummary.TRAFFIC_ARROW
    route.next_maneuver = "LEFT"
    assert left_fsm.update(scene, route, None).fsm_state == "PASSING"

    right_fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.0,
    )
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    right_fsm.update(scene, route, None)
    scene.traffic_light_state = SceneSummary.TRAFFIC_ARROW
    route.next_maneuver = "RIGHT"
    assert right_fsm.update(scene, route, None).fsm_state == "WAIT_GREEN"


def test_traffic_light_arrow_releases_configured_right_maneuver() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.0,
    )
    scene = SceneSummary()
    scene.stopline_distance = 2.0
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    route = make_route(100.0)
    route.next_maneuver = "RIGHT"

    fsm.update(scene, route, None, arrow_maneuver="RIGHT")
    scene.traffic_light_state = SceneSummary.TRAFFIC_ARROW

    assert (
        fsm.update(scene, route, None, arrow_maneuver="RIGHT").fsm_state
        == "PASSING"
    )


def test_traffic_light_deceleration_latches_through_temporary_unknown() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.4,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 12.0
    scene.header.stamp.sec = 30
    assert fsm.update(scene, None, None).fsm_state == "DECELERATE"
    assert fsm.active is True

    scene.traffic_light_state = SceneSummary.TRAFFIC_UNKNOWN
    scene.stopline_distance = 6.0
    scene.header.stamp.nanosec = 200_000_000
    decision = fsm.update(scene, None, None)

    assert decision.fsm_state == "DECELERATE"
    assert decision.need_stop is True
    assert decision.path_request == "STOP_PATH"


def test_traffic_light_passing_ignores_same_signal_until_stopline_is_behind() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        release_confirm_sec=0.4,
    )
    scene = SceneSummary()
    scene.stopline_distance = 2.0
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.header.stamp.sec = 40
    fsm.update(scene, None, None)
    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    scene.header.stamp.nanosec = 100_000_000
    fsm.update(scene, None, None)
    scene.header.stamp.nanosec = 500_000_000
    assert fsm.update(scene, None, None).fsm_state == "PASSING"

    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 0.2
    passing = fsm.update(scene, None, None)
    assert passing.fsm_state == "PASSING"
    assert passing.need_stop is False
    assert passing.path_request == "ROUTE_LOCAL_PATH"

    scene.stopline_distance = 1.0e6
    complete = fsm.update(scene, None, None)
    assert complete.fsm_state == "START"
    assert complete.need_stop is False
    assert fsm.active is False


def test_traffic_light_yellow_commits_when_stopping_distance_is_insufficient() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        yellow_stop_decel_mps2=2.0,
        yellow_reaction_time_sec=0.25,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_YELLOW
    scene.stopline_distance = 6.0
    vehicle = VehicleState()
    vehicle.speed_mps = 4.0

    decision = fsm.update(scene, None, vehicle)

    assert decision.fsm_state == "PASSING"
    assert decision.need_stop is False


def test_traffic_light_yellow_stops_when_enough_distance_remains() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        yellow_stop_decel_mps2=2.0,
        yellow_reaction_time_sec=0.25,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_YELLOW
    scene.stopline_distance = 15.0
    vehicle = VehicleState()
    vehicle.speed_mps = 4.0

    decision = fsm.update(scene, None, vehicle)

    assert decision.fsm_state == "DECELERATE"
    assert decision.need_stop is True


def test_traffic_light_does_not_stop_after_front_bumper_crosses_line() -> None:
    fsm = TrafficLightFSM(
        base_speed_mps=4.0,
        approach_distance_m=18.0,
        vehicle_front_overhang_m=1.98,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 1.5
    vehicle = VehicleState()
    vehicle.speed_mps = 1.0

    decision = fsm.update(scene, None, vehicle)

    assert decision.fsm_state == "PASSING"
    assert decision.need_stop is False


def test_planning_core_stopline_fallback_reserves_front_overhang_and_clearance() -> None:
    core = PlanningCore(
        base_speed_mps=4.0,
        degraded_speed_mps=0.8,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=0.5,
    )
    scene = SceneSummary()
    scene.stopline_distance = 10.0
    decision = DecisionCommand(path_request="STOP_PATH")

    assert abs(core.resolve_stop_distance(decision, scene) - 7.52) < 1e-6


def test_stop_plan_keeps_forward_route_for_lateral_control() -> None:
    core = PlanningCore(base_speed_mps=4.0, degraded_speed_mps=0.8)
    scene = make_scene(False)
    decision = DecisionCommand(
        active_behavior="TRAFFIC_LIGHT",
        need_stop=True,
        path_request="STOP_PATH",
        stop_target_distance=0.0,
    )

    path, speed = core.make_plan(decision, scene, make_route(100.0), [])

    assert path.source == "longitudinal_stop_from_global_route_local_path"
    assert len(path.points) >= 2
    assert max(point.x for point in path.points) >= 20.0
    assert speed.target_speed_mps == 0.0
