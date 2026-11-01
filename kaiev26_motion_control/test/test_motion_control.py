"""분할 전 `test_control_command_gate.py`의 계획정지 판정을 그대로 옮겼다.

이 시험이 고정하는 것은 판단 소유 절반이다 — 속도 ramp와 계획정지 제동 시점.
안전 override는 Control의 bridge 시험이 갖는다.
"""
import math
from types import SimpleNamespace

from geometry_msgs.msg import Point
import pytest

from kaiev26_motion_control.motion_control_node import (
    adaptive_lookahead_decision,
    adaptive_stanley_profile,
    blend_lateral_steering,
    filtered_steering_rate,
    hybrid_pp_weight_for_phase,
    interpolated_lookahead_target,
    LookaheadDecision,
    MotionControlNode,
    slew_lookahead,
    stanley_path_reference,
    stanley_preview_distance,
    stanley_steering_decision,
    sustained_curvature_profile,
    SteeringCompensationConfig,
    SteeringCompensator,
    StanleyDecision,
)
from kaiev26_msgs.msg import ActuatorCommand, Centerline, RouteContext, TargetSpeed, VehicleState


# A captured route's phase table: straight, one corner, straight.
ROUTE_BOUNDARIES = (0.0, 98.0, 106.0, 146.0, 150.0, 286.433416)
ROUTE_PHASES = ("straight", "turn_approach", "turning", "turn_exit", "straight")


def make_route_phase_motion() -> MotionControlNode:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.hybrid_route_segment_boundaries_m = ROUTE_BOUNDARIES
    motion.hybrid_route_segment_phases = ROUTE_PHASES
    motion.hybrid_route_length_m = ROUTE_BOUNDARIES[-1]
    motion.target_stale_sec = 0.35
    motion.latest_target_path = Centerline()
    motion.latest_target_path.source = "global_route_local_path"
    context = RouteContext()
    context.progress_s = ROUTE_BOUNDARIES[2] + 1.0
    context.distance_to_finish = ROUTE_BOUNDARIES[-1] - context.progress_s
    context.route_projection_valid = True
    context.header.stamp.sec = 10
    motion.latest_route_context = context
    motion.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_020_000_000)
    )
    return motion


def test_route_phase_keeps_corner_mix_despite_local_phase_flicker() -> None:
    motion = make_route_phase_motion()
    for geometry in ("straight", "turn_approach", "turning", "turn_exit"):
        assert motion.select_hybrid_phase(geometry) == "turning"
        assert motion.hybrid_route_phase_active


@pytest.mark.parametrize("reason", ["disabled", "missing", "stale", "future", "invalid", "map", "rejoin", "obstacle", "outside"])
def test_route_phase_falls_back_without_stop_or_cross_mission_override(reason) -> None:
    motion = make_route_phase_motion()
    context = motion.latest_route_context
    if reason == "disabled":
        motion.hybrid_route_segment_phases = ()
    elif reason == "missing":
        motion.latest_route_context = None
    elif reason == "stale":
        context.header.stamp.sec = 9
    elif reason == "future":
        context.header.stamp.sec = 11
    elif reason == "invalid":
        context.route_projection_valid = False
    elif reason == "map":
        context.distance_to_finish += 1.0
    elif reason in {"rejoin", "obstacle"}:
        motion.latest_target_path.source = reason
    else:
        context.progress_s = -1.0
        context.distance_to_finish = ROUTE_BOUNDARIES[-1] + 1.0
    assert motion.select_hybrid_phase("turn_approach") == "turn_approach"
    assert not motion.hybrid_route_phase_active


def path_point(x: float, y: float) -> Point:
    point = Point()
    point.x = x
    point.y = y
    return point


def left_arc(radius_m: float, length_m: int = 16) -> list[Point]:
    return [
        path_point(
            radius_m * math.sin(distance / radius_m),
            radius_m * (1.0 - math.cos(distance / radius_m)),
        )
        for distance in range(length_m + 1)
    ]


def make_motion(
    commanded_speed: float,
    vehicle_speed: float,
    target_speed: float = 0.0,
    stop_target_distance: float = 10.0,
) -> MotionControlNode:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.max_accel_mps2 = 1.0
    motion.signal_release_accel_mps2 = 2.0
    motion.signal_release_boost_speed_mps = 5.0
    motion.signal_release_boost_active = False
    motion.max_decel_mps2 = 2.0
    motion.stop_pre_decel_mps2 = 10.0
    motion.obstacle_decel_mps2 = 10.0
    motion.stop_speed_epsilon_mps = 0.05
    motion.planned_stop_brake_distance_m = 2.0
    motion.brake_release_delay_sec = 0.8
    motion.brake_command_active = False
    motion.brake_release_pending = False
    motion.brake_release_elapsed_sec = 0.0
    motion.stopped_steering_hold_speed_mps = 0.10
    motion.planned_stop_steering_align_distance_m = 2.0
    motion.stop_release_steering_resume_speed_mps = 0.30
    motion.planned_stop_steering_hold_active = False
    motion.max_steering_rad = 0.4363
    # No offset and no friction push: the command is the tracker output.
    motion.steering_compensator = SteeringCompensator(SteeringCompensationConfig(
        neutral_offset_rad=0.0, friction_speeds_mps=(1.0,), friction_rad=(0.0,),
        error_start_rad=0.0, error_full_rad=1.0, error_ramp_s=0.0))
    motion.straight_integral_rad = 0.0
    motion.stanley_decision = StanleyDecision(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    motion.lookahead_phase = "straight"
    motion.steering_rate_limit_radps = 1.0471975512
    motion.steering_tracking_error_rad = 0.0
    motion.commanded_speed_mps = commanded_speed
    motion.commanded_steering_rad = 0.0
    motion.steering_rate_filter_time_constant_s = 0.15
    motion.steering_rate_estimate_limit_radps = math.radians(120.0)
    motion.actual_steering_rate_radps = 0.0
    motion.latest_target_path = None
    motion.latest_target_speed = TargetSpeed()
    motion.latest_target_speed.need_stop = True
    motion.latest_target_speed.target_speed_mps = target_speed
    motion.latest_target_speed.stop_target_distance = stop_target_distance
    motion.latest_vehicle_state = VehicleState()
    motion.latest_vehicle_state.speed_mps = vehicle_speed
    motion.compute_pp_stanley_steering = lambda: 0.0
    return motion


def test_filtered_steering_rate_rejects_a_single_encoder_step() -> None:
    rate = filtered_steering_rate(
        math.radians(1.0),
        math.radians(2.0),
        0.02,
        0.0,
        0.15,
        math.radians(120.0),
    )

    assert math.degrees(rate) == pytest.approx(5.882, abs=0.001)


def test_hybrid_uses_fixed_weights_for_debuggable_route_phases() -> None:
    weights = (0.0, 0.65, 0.35, 0.10)
    assert hybrid_pp_weight_for_phase("straight", *weights) == pytest.approx(0.0)
    assert hybrid_pp_weight_for_phase("turn_approach", *weights) == pytest.approx(
        0.65
    )
    assert hybrid_pp_weight_for_phase("turning", *weights) == pytest.approx(0.35)
    assert hybrid_pp_weight_for_phase("chained_turn", *weights) == pytest.approx(0.35)
    assert hybrid_pp_weight_for_phase("turn_exit", *weights) == pytest.approx(0.10)
    assert blend_lateral_steering(0.20, -0.10, 0.0) == pytest.approx(-0.10)
    assert blend_lateral_steering(0.20, -0.10, 0.5) == pytest.approx(0.05)
    assert blend_lateral_steering(0.20, -0.10, 1.0) == pytest.approx(0.20)


@pytest.mark.parametrize("route_window", [False, True])
def test_hybrid_preserves_pp_phase_and_slews_to_its_fixed_weight(route_window) -> None:
    motion = make_route_phase_motion() if route_window else MotionControlNode.__new__(MotionControlNode)
    motion.lookahead_decision = LookaheadDecision(
        8.0, "straight", 0.0, 0.0, math.inf
    )
    motion.lookahead_phase = "straight"
    motion.lookahead_curve_active = False
    motion.hybrid_pp_straight_weight = 0.0
    motion.hybrid_pp_approach_weight = 0.65
    motion.hybrid_pp_turning_weight = 0.35
    motion.hybrid_pp_exit_weight = 0.10
    motion.hybrid_blend_rate_per_s = 6.0
    motion.control_rate_hz = 50.0
    motion.hybrid_pp_weight = 0.0

    def pure_pursuit() -> float:
        motion.lookahead_decision = LookaheadDecision(
            5.0, "turn_approach", 0.0, 0.05, 6.0, 0.5
        )
        motion.lookahead_phase = "turn_approach"
        motion.lookahead_curve_active = True
        return 0.20

    def stanley() -> float:
        motion.lookahead_phase = "turning"
        motion.lookahead_curve_active = True
        return -0.10

    motion.compute_pure_pursuit_steering = pure_pursuit
    motion.compute_stanley_steering = stanley

    steering = motion.compute_pp_stanley_steering()

    assert motion.hybrid_phase == ("turning" if route_window else "turn_approach")
    assert motion.lookahead_phase == "turn_approach"
    assert motion.hybrid_target_pp_weight == pytest.approx(0.35 if route_window else 0.65)
    assert motion.hybrid_pp_weight == pytest.approx(0.12)
    assert steering == pytest.approx(-0.064)


def test_stanley_centered_straight_path_requests_zero_steering() -> None:
    decision = stanley_steering_decision(
        [path_point(0.0, 0.0), path_point(4.0, 0.0), path_point(8.0, 0.0)],
        speed_mps=6.0,
        front_axle_offset_m=1.2991017929,
        heading_gain=1.0,
        cross_track_gain=0.8,
        softening_speed_mps=2.0,
        cross_track_correction_limit_rad=0.12,
        heading_window_m=4.0,
    )

    assert decision.valid
    assert decision.heading_error_rad == pytest.approx(0.0)
    assert decision.cross_track_error_m == pytest.approx(0.0)
    assert decision.steering_rad == pytest.approx(0.0)


@pytest.mark.parametrize(("offset_m", "expected_sign"), [(0.5, 1.0), (-0.5, -1.0)])
def test_stanley_cross_track_error_steers_toward_path(offset_m, expected_sign) -> None:
    decision = stanley_steering_decision(
        [
            path_point(0.0, offset_m),
            path_point(4.0, offset_m),
            path_point(8.0, offset_m),
        ],
        speed_mps=6.0,
        front_axle_offset_m=1.2991017929,
        heading_gain=1.0,
        cross_track_gain=0.8,
        softening_speed_mps=2.0,
        cross_track_correction_limit_rad=0.12,
        heading_window_m=4.0,
    )

    assert math.copysign(1.0, decision.cross_track_error_m) == expected_sign
    assert math.copysign(1.0, decision.steering_rad) == expected_sign


def test_stanley_cross_track_correction_decreases_with_speed() -> None:
    path = [path_point(0.0, 0.5), path_point(4.0, 0.5), path_point(8.0, 0.5)]
    slow = stanley_steering_decision(
        path, 1.0, 1.2991017929, 1.0, 0.8, 2.0, 0.12, 4.0
    )
    fast = stanley_steering_decision(
        path, 6.0, 1.2991017929, 1.0, 0.8, 2.0, 0.12, 4.0
    )

    assert abs(fast.cross_track_correction_rad) < abs(
        slow.cross_track_correction_rad
    )


def test_stanley_cross_track_correction_is_bounded() -> None:
    decision = stanley_steering_decision(
        [path_point(0.0, 4.0), path_point(4.0, 4.0), path_point(8.0, 4.0)],
        speed_mps=0.0,
        front_axle_offset_m=1.2991017929,
        heading_gain=1.0,
        cross_track_gain=1.0,
        softening_speed_mps=0.1,
        cross_track_correction_limit_rad=0.12,
        heading_window_m=4.0,
    )

    assert decision.cross_track_correction_rad == pytest.approx(0.12)


def test_stanley_heading_correction_is_bounded_independently() -> None:
    decision = stanley_steering_decision(
        [path_point(0.0, 0.0), path_point(4.0, 4.0), path_point(8.0, 8.0)],
        speed_mps=6.0,
        front_axle_offset_m=1.2991017929,
        heading_gain=1.0,
        cross_track_gain=0.0,
        softening_speed_mps=3.0,
        cross_track_correction_limit_rad=0.05,
        heading_window_m=6.0,
        heading_correction_limit_rad=0.10,
    )

    assert decision.heading_error_rad > 0.5
    assert decision.heading_correction_rad == pytest.approx(0.10)
    assert decision.steering_rad == pytest.approx(0.10)


def test_adaptive_stanley_profile_blends_straight_and_curve_values() -> None:
    straight = adaptive_stanley_profile(
        0.0, 0.30, 0.45, 0.25, 0.35, 0.05, 0.07, 0.10, 0.16, 8.0, 6.0
    )
    middle = adaptive_stanley_profile(
        0.5, 0.30, 0.45, 0.25, 0.35, 0.05, 0.07, 0.10, 0.16, 8.0, 6.0
    )
    curve = adaptive_stanley_profile(
        1.0, 0.30, 0.45, 0.25, 0.35, 0.05, 0.07, 0.10, 0.16, 8.0, 6.0
    )

    assert straight.heading_gain == pytest.approx(0.30)
    assert middle.heading_gain == pytest.approx(0.375)
    assert middle.heading_window_m == pytest.approx(7.0)
    assert curve.cross_track_gain == pytest.approx(0.35)
    assert curve.heading_correction_limit_rad == pytest.approx(0.16)


def test_stanley_preview_is_curve_only_and_bounded() -> None:
    assert stanley_preview_distance(6.0, 0.0, 0.55, 3.5) == pytest.approx(0.0)
    assert stanley_preview_distance(6.0, 0.5, 0.55, 3.5) == pytest.approx(1.65)
    assert stanley_preview_distance(10.0, 1.0, 0.55, 3.5) == pytest.approx(3.5)


def test_stanley_preview_reads_a_corner_before_the_front_axle_reaches_it() -> None:
    path = [
        path_point(0.0, 0.0),
        path_point(5.0, 0.0),
        path_point(5.0, 5.0),
    ]
    front_reference = stanley_path_reference(path, 1.3, 2.0)
    preview_reference = stanley_path_reference(path, 1.3, 2.0, 3.0)

    assert front_reference is not None
    assert preview_reference is not None
    assert front_reference[0] == pytest.approx(0.0)
    assert preview_reference[0] > math.radians(5.0)


def test_stanley_reference_uses_smoothed_path_heading() -> None:
    reference = stanley_path_reference(
        [
            path_point(0.0, 0.0),
            path_point(1.0, 0.15),
            path_point(2.0, 0.0),
            path_point(3.0, 0.0),
            path_point(5.0, 0.0),
        ],
        front_axle_offset_m=1.3,
        heading_window_m=4.0,
    )

    assert reference is not None
    assert abs(reference[0]) < math.radians(3.0)


def test_lookahead_target_is_interpolated_between_route_points() -> None:
    target = interpolated_lookahead_target(
        [path_point(0.0, 0.0), path_point(1.0, 0.0), path_point(2.0, 0.0)],
        1.5,
    )

    assert target == pytest.approx((1.5, 0.0))


def test_interpolated_target_lies_on_the_lookahead_circle_with_cross_track_error() -> None:
    target = interpolated_lookahead_target(
        [path_point(0.0, 1.0), path_point(2.0, 1.0), path_point(4.0, 1.0)],
        2.0,
    )

    assert target is not None
    assert target[0] == pytest.approx(3.0**0.5)
    assert target[1] == pytest.approx(1.0)
    assert target[0] * target[0] + target[1] * target[1] == pytest.approx(4.0)


def test_large_cross_track_error_uses_a_local_fallback_instead_of_path_end() -> None:
    target = interpolated_lookahead_target(
        [path_point(0.0, 4.0), path_point(5.0, 4.0), path_point(10.0, 4.0)],
        3.2,
    )

    assert target == pytest.approx((3.2, 4.0))


def test_straight_lookahead_increases_with_vehicle_speed() -> None:
    path = [path_point(float(x), 0.0) for x in range(19)]
    slow = adaptive_lookahead_decision(
        path, 1.0, 4.0, 0.9, 3.2, 8.0, 0.015, 0.070, 4.0, 12.0
    )
    fast = adaptive_lookahead_decision(
        path, 3.5, 4.0, 0.9, 3.2, 8.0, 0.015, 0.070, 4.0, 12.0
    )

    assert slow.phase == "straight"
    assert fast.phase == "straight"
    assert slow.distance_m == pytest.approx(4.9)
    assert fast.distance_m == pytest.approx(7.15)


def test_tight_curve_uses_speed_dependent_floor_from_manual_data() -> None:
    decision = adaptive_lookahead_decision(
        left_arc(10.0),
        3.5,
        4.0,
        0.9,
        3.6,
        8.0,
        0.015,
        0.070,
        4.0,
        12.0,
        0.010,
        0.25,
        3,
    )

    assert decision.phase == "turning"
    assert decision.near_curvature_1pm == pytest.approx(0.1, rel=0.05)
    assert decision.distance_m == pytest.approx(4.475)


def test_curve_requires_three_consecutive_curvature_samples() -> None:
    profile = [(2.0, 0.08), (3.0, 0.07), (4.0, 0.0)]

    assert sustained_curvature_profile(profile, 0.015, 3) == []
    assert sustained_curvature_profile(profile, 0.015, 2) == profile[:2]


def test_curve_hysteresis_keeps_turn_active_below_entry_threshold() -> None:
    gentle_curve = left_arc(80.0)
    inactive = adaptive_lookahead_decision(
        gentle_curve,
        3.0,
        4.0,
        0.9,
        3.6,
        8.0,
        0.015,
        0.070,
        4.0,
        12.0,
        0.010,
        0.25,
        3,
        False,
    )
    active = adaptive_lookahead_decision(
        gentle_curve,
        3.0,
        4.0,
        0.9,
        3.6,
        8.0,
        0.015,
        0.070,
        4.0,
        12.0,
        0.010,
        0.25,
        3,
        True,
    )

    assert inactive.phase == "straight"
    assert active.phase == "turning"
    assert active.distance_m < inactive.distance_m


def test_lookahead_shortens_faster_than_it_lengthens_after_a_turn() -> None:
    shortened = slew_lookahead(7.0, 3.2, 0.1, 8.0, 4.0)
    lengthened = slew_lookahead(3.2, 7.0, 0.1, 8.0, 4.0)

    assert shortened == pytest.approx(6.2)
    assert lengthened == pytest.approx(3.6)


def test_tracking_error_is_diagnostic_only_and_does_not_reduce_speed() -> None:
    motion = make_motion(commanded_speed=8.0, vehicle_speed=8.0, target_speed=8.0)
    motion.commanded_steering_rad = math.radians(1.0)
    motion.latest_vehicle_state.steering_rad = math.radians(6.0)
    motion.latest_target_speed.need_stop = False
    command = ActuatorCommand()

    motion.update_steering_tracking_diagnostic()
    motion.apply_nominal_motion(command, dt=0.1)

    assert math.degrees(motion.steering_tracking_error_rad) == pytest.approx(6.0)
    assert command.speed_target_mps == pytest.approx(8.0)


def test_planned_stop_respects_distance_based_target_speed() -> None:
    motion = make_motion(commanded_speed=5.0, vehicle_speed=5.0, target_speed=4.5)
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 4.5
    assert command.brake_engage is False


@pytest.mark.parametrize(
    ("source", "constraint"),
    [
        ("STATIC_STOP", "BRAKE_TRIGGER_PRE_DECEL"),
        ("TRAFFIC_LIGHT", "SIGNAL_BRAKE_TRIGGER_PRE_DECEL"),
    ],
)
@pytest.mark.parametrize("initial_speed,ticks", [(7.0, 25), (10.0, 40)])
def test_stop_pre_deceleration_uses_ten_mps2_without_braking(
    source: str, constraint: str, initial_speed: float, ticks: int,
) -> None:
    motion = make_motion(initial_speed, initial_speed, target_speed=2.0)
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = source
    motion.latest_target_speed.constraints = [constraint]
    command = ActuatorCommand()

    for tick in range(1, ticks + 1):
        motion.apply_nominal_motion(command, dt=0.02)
        assert command.speed_target_mps == pytest.approx(max(2.0, initial_speed - 0.2*tick))
        assert command.brake_engage is False
    assert motion.max_decel_mps2 == 2.0


@pytest.mark.parametrize(
    ("source", "constraint", "need_stop"),
    [
        ("STATIC_STOP", "BRAKE_TRIGGER_APPROACH", False),
        ("TRAFFIC_LIGHT", "SIGNAL_BRAKE_TRIGGER_APPROACH", False),
        ("STATIC_STOP", "TIMED_STOP_RELEASE", False),
        ("TRAFFIC_LIGHT", "SIGNAL_PASS_COMMITTED", False),
        ("TRAFFIC_LIGHT", "SIGNAL_RELEASE", False),
        ("STATIC_STOP", "SIGNAL_BRAKE_TRIGGER_PRE_DECEL", False),
        ("TRAFFIC_LIGHT", "BRAKE_TRIGGER_PRE_DECEL", False),
        ("STATIC_STOP", "BRAKE_TRIGGER_PRE_DECEL", True),
        ("TRAFFIC_LIGHT", "SIGNAL_BRAKE_TRIGGER_PRE_DECEL", True),
        ("OBSTACLE", "OBSTACLE_SPEED_CAP", True),
        *[(source, constraint, False)
          for source in ("LANE_FOLLOW", "GLOBAL_ROUTE", "FINISH", "RECOVERY", "EMERGENCY")
          for constraint in ("BRAKE_TRIGGER_PRE_DECEL", "SIGNAL_BRAKE_TRIGGER_PRE_DECEL")],
    ],
)
def test_stop_deceleration_override_does_not_affect_other_targets(
    source: str, constraint: str, need_stop: bool,
) -> None:
    motion = make_motion(7.0, 7.0, target_speed=2.0)
    motion.latest_target_speed.source_behavior = source
    motion.latest_target_speed.constraints = [constraint]
    motion.latest_target_speed.need_stop = need_stop
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert motion.select_decel_limit() == 2.0
    assert command.speed_target_mps == pytest.approx(6.8)
    assert command.brake_engage is False


@pytest.mark.parametrize("initial_speed", [7.0, 10.0])
@pytest.mark.parametrize(
    ("target_speed", "constraints"),
    [
        (2.2, ["OBSTACLE_SPEED_CAP", "OBSTACLE_SEQUENCE_PATH"]),
        (3.8, ["OBSTACLE_SEQUENCE_PATH"]),
        (1.0, ["OBSTACLE_NO_SAFE_PATH", "LOW_SPEED_CONTINUE"]),
        (1.0, ["OBSTACLE_FORCED_S_PATH"]),
    ],
)
def test_obstacle_deceleration_uses_ten_mps2_without_braking(
    initial_speed: float, target_speed: float, constraints: list[str],
) -> None:
    motion = make_motion(initial_speed, initial_speed, target_speed=target_speed)
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "OBSTACLE"
    motion.latest_target_speed.constraints = constraints
    command = ActuatorCommand()

    ticks = math.ceil((initial_speed - target_speed) / 0.2)
    for tick in range(1, ticks + 1):
        motion.apply_nominal_motion(command, dt=0.02)
        assert command.speed_target_mps == pytest.approx(
            max(target_speed, initial_speed - 0.2*tick)
        )
        assert command.brake_engage is False
    assert command.speed_target_mps == pytest.approx(target_speed)
    assert motion.max_decel_mps2 == 2.0


def test_obstacle_deceleration_limit_is_independent_from_stop_limit() -> None:
    motion = make_motion(7.0, 7.0, target_speed=2.2)
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "OBSTACLE"
    motion.latest_target_speed.constraints = ["SIGNAL_BRAKE_TRIGGER_PRE_DECEL"]
    motion.obstacle_decel_mps2 = 4.0
    motion.stop_pre_decel_mps2 = 3.0
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert motion.select_decel_limit() == 4.0
    assert command.speed_target_mps == pytest.approx(6.6)
    assert command.brake_engage is False


def test_obstacle_deceleration_override_ends_when_normal_tracking_resumes() -> None:
    motion = make_motion(10.0, 10.0, target_speed=2.2)
    target = motion.latest_target_speed
    target.need_stop = False
    target.source_behavior = "OBSTACLE"
    command = ActuatorCommand()
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(9.0)

    target.source_behavior = "LANE_FOLLOW"
    motion.apply_nominal_motion(command, dt=0.1)

    assert motion.select_decel_limit() == 2.0
    assert command.speed_target_mps == pytest.approx(8.8)


def test_obstacle_return_keeps_normal_acceleration_without_braking() -> None:
    motion = make_motion(2.2, 2.2, target_speed=10.0)
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "OBSTACLE"
    motion.latest_target_speed.constraints = ["GLOBAL_ROUTE_REJOIN"]
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert command.speed_target_mps == pytest.approx(2.3)
    assert command.brake_engage is False
    assert motion.select_accel_limit(10.0) == 1.0


@pytest.mark.parametrize(
    ("commanded", "source_behavior", "constraints"),
    [
        (2.2, "LANE_FOLLOW", []),
        (0.0, "TRAFFIC_LIGHT", ["SIGNAL_RELEASE"]),
    ],
)
def test_zero_max_accel_steps_to_target_at_once_and_keeps_decel(
    commanded, source_behavior, constraints
) -> None:
    motion = make_motion(commanded, commanded, target_speed=10.0, stop_target_distance=1.0e6)
    motion.max_accel_mps2 = 0.0
    target = motion.latest_target_speed
    target.need_stop = False
    target.source_behavior = source_behavior
    target.constraints = constraints
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.02)
    assert command.speed_target_mps == pytest.approx(10.0)
    assert motion.signal_release_boost_active is False

    target.target_speed_mps = 3.0
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(9.8)


def test_stop_deceleration_override_ends_when_signal_is_released() -> None:
    motion = make_motion(7.0, 7.0, target_speed=2.0)
    target = motion.latest_target_speed
    target.need_stop = False
    target.source_behavior = "TRAFFIC_LIGHT"
    target.constraints = ["SIGNAL_BRAKE_TRIGGER_PRE_DECEL"]
    command = ActuatorCommand()
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(6.0)

    target.target_speed_mps = 7.0
    target.constraints = ["SIGNAL_PASS_COMMITTED"]
    motion.apply_nominal_motion(command, dt=0.1)
    assert motion.select_decel_limit() == 2.0
    assert command.speed_target_mps == pytest.approx(6.1)

    target.target_speed_mps = 3.0
    target.source_behavior = "LANE_FOLLOW"
    target.constraints = ["CURVE_SPEED_LIMIT"]
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(5.9)


@pytest.mark.parametrize("constraint", ["SIGNAL_PASS_COMMITTED", "SIGNAL_RELEASE"])
def test_green_release_uses_faster_acceleration_without_changing_normal_limit(constraint) -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=4.0,
        stop_target_distance=1.0e6,
    )
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "TRAFFIC_LIGHT"
    motion.latest_target_speed.constraints = [constraint]
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 1.0
    assert motion.signal_release_boost_active is True

    motion.latest_target_speed.source_behavior = "OBSTACLE"
    motion.latest_target_speed.constraints = []
    assert motion.select_accel_limit(2.2) == 1.0
    assert motion.signal_release_boost_active is False


def test_timed_stop_release_uses_same_faster_acceleration() -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=7.0,
        stop_target_distance=1.0e6,
    )
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "STATIC_STOP"
    motion.latest_target_speed.constraints = ["TIMED_STOP_RELEASE"]
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 1.0
    assert command.brake_engage is False
    assert motion.signal_release_boost_active is True


@pytest.mark.parametrize(
    ("source_behavior", "constraint"),
    [
        ("TRAFFIC_LIGHT", "SIGNAL_PASS_COMMITTED"),
        ("TRAFFIC_LIGHT", "SIGNAL_RELEASE"),
        ("STATIC_STOP", "TIMED_STOP_RELEASE"),
    ],
)
def test_restart_boost_switches_to_normal_acceleration_at_five_mps(
    source_behavior: str,
    constraint: str,
) -> None:
    motion = make_motion(
        commanded_speed=4.8,
        vehicle_speed=4.8,
        target_speed=7.0,
        stop_target_distance=1.0e6,
    )
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = source_behavior
    motion.latest_target_speed.constraints = [constraint]
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(5.0)
    assert motion.signal_release_boost_active is True

    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(5.1)
    assert motion.signal_release_boost_active is False


def test_planned_stop_decelerates_before_holding_brake_at_zero_target() -> None:
    motion = make_motion(commanded_speed=2.0, vehicle_speed=2.0)
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 1.0
    assert command.brake_engage is False


def test_planned_stop_holds_brake_after_vehicle_is_stopped() -> None:
    motion = make_motion(
        commanded_speed=0.02, vehicle_speed=0.02, stop_target_distance=0.0
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 0.0
    assert command.brake_engage is True


def test_planned_stop_holds_steering_after_vehicle_stops() -> None:
    motion = make_motion(
        commanded_speed=0.0, vehicle_speed=0.0, stop_target_distance=0.0
    )
    motion.commanded_steering_rad = 0.08
    motion.latest_vehicle_state.steering_rad = 0.08
    motion.compute_pp_stanley_steering = lambda: (_ for _ in ()).throw(
        AssertionError("stopped vehicle must not recalculate steering")
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert command.steering_target_rad == 0.08
    assert command.brake_engage is True


# situation: actual speed, target speed, stop distance -> brake, steering target. The
# measured wheel angle is 0.15 rad and the path asks for 0.30 rad. While the car still rolls
# under the brake the path keeps steering it (a brake-now stop is not straightened for the
# stop line); once it is slower than the steerable speed the target is the measured angle.
BRAKE_STEERING_CASES = [
    pytest.param(3.0, 0.0, 0.0, (True, "path"), id="brake-now-at-speed"),
    pytest.param(1.0, 0.0, 1.5, (True, "aligned"), id="brake-inside-two-metres"),
    pytest.param(0.2, 0.0, 0.0, (True, "measured"), id="braking-below-steerable-speed"),
    pytest.param(0.0, 0.0, 0.0, (True, "measured"), id="stopped-and-held"),
    pytest.param(1.0, 0.8, 1.5, (False, "aligned"), id="planned-stop-still-rolling-in"),
    pytest.param(3.0, 3.0, 1.0e6, (False, "path"), id="no-stop"),
]


@pytest.mark.parametrize("actual,target,distance,expected", BRAKE_STEERING_CASES)
def test_path_steers_until_the_braking_car_stops(actual, target, distance, expected) -> None:
    motion = make_motion(
        commanded_speed=actual, vehicle_speed=actual,
        target_speed=target, stop_target_distance=distance,
    )
    motion.latest_target_speed.need_stop = distance < 1.0e6
    motion.steering_rate_limit_radps = 0.0
    motion.commanded_steering_rad = 0.15
    motion.latest_vehicle_state.steering_rad = 0.15
    motion.compute_pp_stanley_steering = lambda: 0.30
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.02)

    brake, steering = expected
    assert command.brake_engage is brake
    if steering == "measured":
        assert command.steering_target_rad == pytest.approx(0.15)
    elif steering == "aligned":
        assert command.steering_target_rad == pytest.approx(0.30 * distance / 2.0)
    else:
        assert command.steering_target_rad == pytest.approx(0.30)


def test_planned_stop_straightens_steering_while_vehicle_is_still_rolling() -> None:
    motion = make_motion(
        commanded_speed=0.6,
        vehicle_speed=0.5,
        target_speed=0.4,
        stop_target_distance=0.5,
    )
    motion.commanded_steering_rad = 0.2
    motion.compute_pp_stanley_steering = lambda: 0.2
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert 0.0 < command.steering_target_rad < 0.2
    assert command.brake_engage is False


def test_stop_release_holds_wheel_angle_until_vehicle_rolls() -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=4.0,
        stop_target_distance=1.0e6,
    )
    motion.latest_target_speed.need_stop = False
    motion.commanded_steering_rad = 0.08
    motion.latest_vehicle_state.steering_rad = 0.08
    motion.planned_stop_steering_hold_active = True
    motion.compute_pp_stanley_steering = lambda: -0.2
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert command.steering_target_rad == 0.08
    assert motion.planned_stop_steering_hold_active is True

    motion.latest_vehicle_state.speed_mps = 0.31
    motion.apply_nominal_motion(command, dt=0.1)

    assert command.steering_target_rad < 0.08
    assert motion.planned_stop_steering_hold_active is False


def test_planned_stop_does_not_brake_while_positive_target_remains() -> None:
    motion = make_motion(
        commanded_speed=1.2,
        vehicle_speed=1.0,
        target_speed=0.8,
        stop_target_distance=1.5,
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.2)

    assert command.brake_engage is False
    assert command.speed_target_mps == 0.8


@pytest.mark.parametrize("constraint", ["SIGNAL_PASS_COMMITTED", "SIGNAL_RELEASE"])
def test_brake_release_precedes_signal_restart_acceleration(constraint) -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=0.0,
        stop_target_distance=0.0,
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)
    assert command.brake_engage

    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.target_speed_mps = 4.0
    motion.latest_target_speed.source_behavior = "TRAFFIC_LIGHT"
    motion.latest_target_speed.constraints = [constraint]

    motion.apply_nominal_motion(command, dt=0.4)
    assert command.speed_target_mps == 0.0
    assert not command.brake_engage
    assert motion.brake_release_pending

    motion.latest_target_speed.constraints = []
    motion.apply_nominal_motion(command, dt=0.4)
    assert command.speed_target_mps == 0.0
    assert not command.brake_engage
    assert motion.brake_release_pending

    motion.apply_nominal_motion(command, dt=0.4)
    assert command.speed_target_mps == 0.0
    assert not command.brake_engage
    assert not motion.brake_release_pending

    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(0.2)
    assert not command.brake_engage


@pytest.mark.parametrize("constraint", [None, "SIGNAL_RELEASE"])
def test_new_hard_stop_cancels_pending_brake_release(constraint) -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=0.0,
        stop_target_distance=0.0,
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.target_speed_mps = 4.0
    if constraint is not None:
        motion.latest_target_speed.source_behavior = "TRAFFIC_LIGHT"
        motion.latest_target_speed.constraints = [constraint]
    motion.apply_nominal_motion(command, dt=0.2)
    assert motion.brake_release_pending
    assert not command.brake_engage
    if constraint is not None:
        assert motion.signal_release_boost_active

    motion.latest_target_speed.need_stop = True
    motion.latest_target_speed.target_speed_mps = 0.0
    motion.latest_target_speed.stop_target_distance = 0.0
    motion.latest_target_speed.constraints = []
    motion.apply_nominal_motion(command, dt=0.1)

    assert command.speed_target_mps == 0.0
    assert command.brake_engage
    assert not motion.brake_release_pending
    assert not motion.signal_release_boost_active


@pytest.mark.parametrize('speed', [0.0, 0.27, -0.27])
def test_low_speed_steering_tracks_feedback_until_control_can_actuate(speed) -> None:
    motion = make_motion(0.0, speed, target_speed=3.0, stop_target_distance=100.0)
    motion.latest_target_speed.need_stop = False
    motion.commanded_steering_rad = 0.4
    motion.latest_vehicle_state.steering_rad = 0.08
    motion.compute_pp_stanley_steering = lambda: -0.2
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.02)
    assert command.steering_target_rad == 0.08
    assert command.speed_target_mps > 0.0
    assert not command.brake_engage

    motion.latest_vehicle_state.speed_mps = 0.28
    motion.apply_nominal_motion(command, dt=0.02)
    assert command.steering_target_rad < 0.08
    assert abs(command.steering_target_rad - 0.08) <= motion.steering_rate_limit_radps * 0.02
    assert command.speed_target_mps == pytest.approx(0.04)


def test_planned_stop_brakes_close_to_target_after_zero_is_requested() -> None:
    motion = make_motion(
        commanded_speed=1.2,
        vehicle_speed=1.0,
        target_speed=0.0,
        stop_target_distance=1.5,
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.2)

    assert command.brake_engage is True
    assert command.speed_target_mps == 0.0


def test_input_loss_brake_holds_the_measured_steering() -> None:
    """입력 끊김이 1초를 넘겨 제동할 때 조향 목표는 측정 조향각이다.

    발행을 멈추지는 않는다. 실차 STM_A는 `fw_current`에서 command deadman이 없어 발행이
    끊기면 마지막 목표를 계속 보내므로 명시적인 속도 0과 제동이 나가야 한다.
    """
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.max_steering_rad = 0.4363
    motion.latest_vehicle_state = VehicleState()
    motion.latest_vehicle_state.steering_rad = 0.1
    command = ActuatorCommand()

    motion.apply_stop_motion(command)

    assert command.speed_target_mps == 0.0
    assert command.brake_engage is True
    assert command.steering_target_rad == 0.1


def timer_motion(*, vehicle_speed=5.0, steering=0.1):
    from rclpy.time import Time
    motion = make_motion(commanded_speed=vehicle_speed, vehicle_speed=vehicle_speed, target_speed=4.0,
                         stop_target_distance=1.0e6)
    motion.latest_target_speed.need_stop = False
    motion.latest_vehicle_state.mode = VehicleState.MODE_AUTO
    motion.latest_vehicle_state.steering_rad = steering
    motion.target_stale_sec = 0.35
    motion.control_rate_hz = 50.0
    motion.latest_target_path = Centerline()
    motion.latest_target_path.confidence = 1.0
    motion.latest_target_path.points = [path_point(0.0, 0.0), path_point(10.0, 0.0)]
    motion.compute_limited_steering = lambda dt, scale=1.0: 0.0
    motion.update_steering_tracking_diagnostic = lambda: None
    published = []
    motion.command_pub = type('Pub', (), {'publish': lambda _self, msg: published.append(msg)})()
    clock = {'now': 10.0}
    motion.get_clock = lambda: type('Clock', (), {'now': lambda _self: Time(seconds=clock['now'])})()
    motion.last_control_time = Time(seconds=9.98)

    def tick(now, fresh, speed):
        clock['now'] = now
        if fresh:
            motion.latest_target_path.header.stamp = Time(seconds=now).to_msg()
            motion.latest_target_speed.header.stamp = Time(seconds=now).to_msg()
        motion.latest_vehicle_state.speed_mps = speed
        motion.on_control_timer()
        command = published[-1]
        return round(command.speed_target_mps, 3), command.brake_engage, command.steering_target_rad

    return motion, tick


# D2 in motion_control: (time, targets fresh, measured speed) -> (speed target > 0, brake)
TARGET_LOSS_SEQUENCE = (
    (10.0, True, 5.0, (True, False)),
    (10.5, False, 5.0, (False, False)),   # targets older than 0.35 s: speed 0, no brake
    (11.4, False, 4.0, (False, False)),
    (11.5, False, 3.5, (False, True)),    # stale for 1 s: brake
    (11.6, True, 2.0, (False, True)),     # targets back while moving: brake stays
    (11.7, True, 0.0, (False, False)),    # stopped: the brake releases (0.8 s traction hold)
    (12.6, True, 0.0, (False, False)),
    (12.7, True, 0.0, (True, False)),     # and the car drives again
)


def test_target_loss_follows_the_shared_input_loss_rule() -> None:
    _motion, tick = timer_motion()
    for now, fresh, speed, (moving, brake) in TARGET_LOSS_SEQUENCE:
        target, engaged, steering = tick(now, fresh, speed)
        assert (target > 0.0, engaged) == (moving, brake), now
        if not fresh:
            assert steering == pytest.approx(0.1)


# D7 with 0260a02: (measured speed, stop distance) -> steering target is the measured steering
# only once the braking car is slower than the steerable speed; while it rolls the path steers.
@pytest.mark.parametrize('speed, distance, held', [
    (3.0, 0.0, False),
    (3.0, 1.5, False),
    (3.0, 20.0, False),
    (0.2, 0.0, True),
    (0.0, 0.0, True),
])
def test_steering_is_held_only_once_the_braking_car_is_slow(speed, distance, held) -> None:
    motion = make_motion(commanded_speed=speed, vehicle_speed=speed, target_speed=0.0,
                         stop_target_distance=distance)
    motion.latest_vehicle_state.steering_rad = 0.2
    motion.commanded_steering_rad = 0.0
    motion.compute_limited_steering = lambda dt, scale=1.0: -0.3
    motion.update_steering_tracking_diagnostic = lambda: None
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.02)

    assert command.steering_target_rad == pytest.approx(0.2 if held else -0.3)


@pytest.mark.parametrize('distance, scale', [(0.0, 1.0), (-1.0, 1.0), (1.0, 0.5), (2.0, 1.0), (1.0e6, 1.0)])
def test_brake_now_distance_never_straightens_the_wheels(distance, scale) -> None:
    motion = make_motion(commanded_speed=1.0, vehicle_speed=1.0)
    assert motion.planned_stop_steering_scale(True, distance) == pytest.approx(scale)


def test_manual_mode_tracks_vehicle_without_requesting_autonomous_brake() -> None:
    motion = make_motion(commanded_speed=0.0, vehicle_speed=0.0)
    motion.max_steering_rad = 0.4363
    motion.last_drive_mode = None
    vehicle = VehicleState()
    vehicle.mode = VehicleState.MODE_MANUAL
    vehicle.speed_mps = 3.2
    vehicle.steering_rad = 0.18

    motion.on_vehicle_state(vehicle)
    command = ActuatorCommand()
    motion.apply_manual_standby(command)

    assert motion.commanded_speed_mps == 3.2
    assert motion.commanded_steering_rad == 0.18
    assert command.speed_target_mps == 0.0
    assert command.steering_target_rad == 0.18
    assert command.brake_engage is False


def test_auto_takeover_does_not_slow_for_steering_tracking_error() -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=4.0,
        stop_target_distance=1.0e6,
    )
    motion.max_steering_rad = 0.4363
    motion.last_drive_mode = None
    motion.latest_target_speed.need_stop = False

    manual = VehicleState()
    manual.mode = VehicleState.MODE_MANUAL
    manual.speed_mps = 2.5
    manual.steering_rad = 0.12
    motion.on_vehicle_state(manual)

    autonomous = VehicleState()
    autonomous.mode = VehicleState.MODE_AUTO
    autonomous.speed_mps = 2.4
    autonomous.steering_rad = 0.10
    motion.on_vehicle_state(autonomous)
    command = ActuatorCommand()
    motion.apply_nominal_motion(command, dt=0.1)

    assert command.speed_target_mps == pytest.approx(2.5)
    assert math.degrees(motion.steering_tracking_error_rad) == pytest.approx(
        math.degrees(0.10),
    )
    assert command.brake_engage is False


# ---- steering compensation (2026-10-22): neutral offset and encoder-error friction push ----
from pathlib import Path

from kaiev26_motion_control.motion_control_node import (
    latency_shifted_points, scheduled_stanley_gains, validate_steering_compensation,
)

MOTION_YAML = Path(__file__).resolve().parents[1] / "config" / "motion_control.yaml"
DT = 0.02


def compensation_config(**changes) -> SteeringCompensationConfig:
    values = dict(
        neutral_offset_rad=math.radians(-0.82), friction_speeds_mps=(3.0, 6.5, 12.0),
        friction_rad=tuple(math.radians(v) for v in (1.0, 0.85, 0.6)),
        error_start_rad=math.radians(0.2), error_full_rad=math.radians(0.5), error_ramp_s=0.1,
    )
    values.update(changes)
    return SteeringCompensationConfig(**values)


def run_compensator(commands_deg, encoder_deg=None, speed=6.5, config=None):
    """Feed the commands at 50 Hz; returns the friction push (deg) after each one."""
    compensator = SteeringCompensator(config or compensation_config())
    pushes = []
    for i, command in enumerate(commands_deg):
        encoder = None if encoder_deg is None else math.radians(
            encoder_deg[i] if isinstance(encoder_deg, list) else encoder_deg)
        compensator.apply(math.radians(command), encoder, speed, DT)
        pushes.append(math.degrees(compensator.friction_value_rad))
    return compensator, pushes


def steering_motion(steering_deg: float) -> MotionControlNode:
    """Steering the car with the rate limit out of the way."""
    motion = make_motion(commanded_speed=3.0, vehicle_speed=3.0, target_speed=3.0)
    motion.steering_rate_limit_radps = 0.0
    motion.compute_lateral_steering = lambda: math.radians(steering_deg)
    return motion


def test_the_node_starts_with_the_shipped_settings() -> None:
    import rclpy
    rclpy.init(args=["--ros-args", "--params-file", str(MOTION_YAML)])
    try:
        MotionControlNode().destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.parametrize("change", [
    dict(friction_speeds_mps=(6.5, 3.0, 12.0)),
    dict(friction_rad=(0.0, math.radians(4.0), 0.0)),
    dict(neutral_offset_rad=math.radians(5.0)),
    dict(error_full_rad=math.radians(0.1)),
    dict(error_ramp_s=-0.1),
    dict(friction_speeds_mps=(3.0,), friction_rad=()),
])
def test_a_compensation_config_that_would_misbehave_is_rejected(change) -> None:
    with pytest.raises(ValueError):
        validate_steering_compensation(compensation_config(**change))


@pytest.mark.parametrize(("speed", "expected"), [(1.0, 1.0), (3.0, 1.0), (4.75, 0.925), (6.5, 0.85),
                                                 (12.0, 0.6), (15.0, 0.6)])
def test_the_friction_push_follows_the_speed_table(speed, expected) -> None:
    compensator = SteeringCompensator(compensation_config())
    assert math.degrees(compensator.friction_at(speed)) == pytest.approx(expected)


@pytest.mark.parametrize(("encoder", "expected"), [
    (-0.82, 0.0),            # on target: nothing
    (-0.82 - 0.15, 0.0),     # inside the start error
    (-0.82 - 0.35, 0.425),   # half way between 0.2 and 0.5 deg
    (-0.82 - 1.0, 0.85),     # beyond the full error, to the left
    (-0.82 + 1.0, -0.85),    # encoder left of the target: push right
])
def test_encoder_error_pushes_by_the_shortfall(encoder, expected) -> None:
    _, pushes = run_compensator([0.0] * 50, encoder_deg=encoder)
    assert pushes[-1] == pytest.approx(expected)


def test_encoder_error_without_an_encoder_adds_only_the_offset() -> None:
    compensator, pushes = run_compensator([1.0] * 20, encoder_deg=None)
    assert pushes[-1] == 0.0
    assert math.degrees(compensator.added_rad) == pytest.approx(-0.82)


def test_node_adds_the_compensation_to_the_command() -> None:
    motion = steering_motion(-1.75)
    motion.steering_compensator = SteeringCompensator(compensation_config())
    motion.latest_vehicle_state.steering_rad = math.radians(-1.75)
    result = motion.compute_limited_steering(dt=DT)
    # encoder at -1.75 vs target -1.75-0.82: 0.82 deg short to the right, push starts ramping right
    assert math.degrees(result) < -1.75 - 0.82
    keys = {item.key: item.value for item in motion.steering_compensation_diagnostics()}
    assert float(keys["steering_compensation_deg"]) == pytest.approx(math.degrees(result) + 1.75)


def test_manual_sync_resets_the_compensator() -> None:
    motion = steering_motion(0.0)
    motion.steering_compensator = SteeringCompensator(compensation_config())
    for _ in range(50):
        motion.steering_compensator.apply(0.0, math.radians(-2.0), 6.5, DT)
    assert motion.steering_compensator.friction_value_rad != 0.0
    vehicle = VehicleState()
    vehicle.steering_rad = 0.0
    vehicle.speed_mps = 0.0
    motion.sync_command_state_to_vehicle(vehicle)
    assert motion.steering_compensator.friction_value_rad == 0.0


# ---- straight tracking (2026-10-22): scheduled gains, latency compensation, integral ----
L_M = 1.2991017929


def natural_frequency_and_damping(speed, heading_gain, cross_track_gain, softening=3.0):
    wn = math.sqrt(speed * speed * cross_track_gain / (L_M * (speed + softening)))
    damping = speed * (heading_gain + cross_track_gain * L_M / (speed + softening)) / L_M / (2.0 * wn)
    return wn, damping


@pytest.mark.parametrize("speed", [5.0, 7.0, 10.0, 15.0])
def test_scheduled_gains_hold_the_target_frequency_and_damping(speed) -> None:
    heading, cross_track = scheduled_stanley_gains(speed, 1.5, 0.95, L_M, 3.0, 4.0)
    wn, damping = natural_frequency_and_damping(speed, heading, cross_track)
    assert wn == pytest.approx(1.5)
    assert damping == pytest.approx(0.95)


def test_scheduled_gains_soften_with_speed_and_freeze_below_the_minimum() -> None:
    gains = [scheduled_stanley_gains(v, 1.5, 0.95, L_M, 3.0, 4.0) for v in (7.0, 10.0, 15.0)]
    assert gains[0][0] > gains[1][0] > gains[2][0]
    assert gains[0][1] > gains[1][1] > gains[2][1]
    assert scheduled_stanley_gains(1.0, 1.5, 0.95, L_M, 3.0, 4.0) == scheduled_stanley_gains(
        4.0, 1.5, 0.95, L_M, 3.0, 4.0)


def test_latency_shift_moves_the_path_back_by_the_travel() -> None:
    shifted = latency_shifted_points([path_point(float(x), 0.0) for x in range(10)], 0.1, 10.0, 0.0)
    assert [p.x for p in shifted] == pytest.approx([x - 1.0 for x in range(10)])
    assert all(p.y == pytest.approx(0.0) for p in shifted)


def test_latency_shift_turns_a_straight_path_against_the_yaw() -> None:
    # The car yawed left since the stamp, so the straight path now lies to its right.
    shifted = latency_shifted_points([path_point(float(x), 0.0) for x in range(10)], 0.1, 10.0, 0.5)
    assert shifted[-1].y < shifted[1].y < 0.0


def make_straight_motion(cross_track_error_m: float, phase: str = "straight") -> MotionControlNode:
    motion = make_motion(10.0, 10.0)
    motion.control_rate_hz = 50.0
    motion.wheelbase_m = L_M
    motion.stanley_softening_speed_mps = 3.0
    motion.straight_natural_frequency_radps = 1.5
    motion.straight_damping_ratio = 0.95
    motion.straight_gain_blend_start_mps = 3.0
    motion.straight_gain_full_speed_mps = 6.0
    motion.stanley_heading_gain = 0.30
    motion.stanley_cross_track_gain = 0.25
    motion.straight_integral_max_error_m = 0.6
    motion.straight_integral_time_s = 4.0
    motion.straight_integral_limit_rad = math.radians(1.5)
    motion.straight_integral_min_speed_mps = 4.0
    motion.straight_integral_rad = 0.0
    motion.lookahead_phase = phase
    motion.latest_vehicle_state.mode = VehicleState.MODE_AUTO
    path = Centerline()
    path.source = "global_route_local_path"
    path.points = [path_point(float(x), 0.0) for x in range(10)]
    motion.latest_target_path = path
    motion.stanley_decision = StanleyDecision(0.0, 0.0, cross_track_error_m, 0.0, 0.0, 0.0, True)
    motion.latency_compensated_path = lambda received: None
    motion.compute_pp_stanley_steering = lambda: 0.0
    return motion


def test_straight_integral_steers_toward_the_path_and_is_bounded() -> None:
    motion = make_straight_motion(0.5)          # path 0.5 m to the left
    first = motion.compute_lateral_steering()
    assert first > 0.0
    for _ in range(50 * 60):
        last = motion.compute_lateral_steering()
    assert last == pytest.approx(math.radians(1.5))


def test_straight_integral_holds_outside_straights_and_clears_in_manual() -> None:
    motion = make_straight_motion(0.5)
    for _ in range(100):
        motion.compute_lateral_steering()
    held = motion.straight_integral_rad
    motion.lookahead_phase = "turning"
    for _ in range(100):
        assert motion.compute_lateral_steering() == pytest.approx(held)
    motion.latest_target_path.source = "obstacle_bypass"
    motion.lookahead_phase = "straight"
    assert motion.compute_lateral_steering() == pytest.approx(held)
    vehicle = VehicleState()
    motion.steering_compensator = None
    motion.sync_command_state_to_vehicle(vehicle)
    assert motion.straight_integral_rad == 0.0


def test_trackers_see_the_latency_shifted_path_for_one_tick_only() -> None:
    motion = make_straight_motion(0.0)
    received = motion.latest_target_path
    shifted = SimpleNamespace(points=[path_point(1.0, 0.0)], source=received.source)
    seen = []
    motion.latency_compensated_path = lambda path: shifted
    motion.compute_pp_stanley_steering = lambda: seen.append(motion.latest_target_path) or 0.0
    motion.compute_lateral_steering()
    assert seen == [shifted]
    assert motion.latest_target_path is received


def test_straight_integral_holds_on_a_large_error_and_runs_with_a_stop_target() -> None:
    motion = make_straight_motion(1.2)
    for _ in range(100):
        assert motion.compute_lateral_steering() == 0.0
    motion = make_straight_motion(0.3)
    motion.latest_target_path.source = "longitudinal_stop_from_global_route_local_path"
    motion.compute_lateral_steering()
    assert motion.straight_integral_rad > 0.0


def test_straight_gains_return_to_the_fixed_gains_at_low_speed() -> None:
    motion = make_straight_motion(0.0)
    assert motion.straight_gains(2.0) == pytest.approx((0.30, 0.25))
    assert motion.straight_gains(8.0) == pytest.approx(
        scheduled_stanley_gains(8.0, 1.5, 0.95, L_M, 3.0, 6.0))
    low, mid, full = (motion.straight_gains(v) for v in (3.0, 4.5, 6.0))
    assert low[0] < mid[0] < full[0] and low[1] < mid[1] < full[1]


def test_latency_compensation_skips_avoidance_paths() -> None:
    motion = make_straight_motion(0.0)
    motion.latest_target_path.source = "forced_sequential_bypass_2_rows"
    motion.path_latency_s = 0.2
    assert MotionControlNode.latency_compensated_path(motion, motion.latest_target_path) is None
    assert motion.path_latency_s == 0.0


def test_stanley_only_paths_force_pure_stanley_in_the_hybrid_controller() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.lookahead_decision = LookaheadDecision(5.0, "turning", 0.0, 0.08, 4.0, 0.8)
    motion.lookahead_phase = "turning"
    motion.lookahead_curve_active = True
    motion.hybrid_pp_straight_weight = 0.0
    motion.hybrid_pp_approach_weight = 0.65
    motion.hybrid_pp_turning_weight = 0.35
    motion.hybrid_pp_exit_weight = 0.10
    motion.hybrid_blend_rate_per_s = 600.0
    motion.control_rate_hz = 50.0
    motion.hybrid_pp_weight = 0.35
    motion.stanley_only_path_source_prefixes = ("sequential_bypass",)
    motion.latest_target_path = Centerline()
    motion.latest_target_path.source = "sequential_bypass_2_rows_map_lane"
    motion.compute_pure_pursuit_steering = lambda: 0.30
    motion.compute_stanley_steering = lambda: -0.10

    steering = motion.compute_pp_stanley_steering()

    assert motion.stanley_only_path_active
    assert motion.hybrid_target_pp_weight == pytest.approx(0.0)
    assert motion.hybrid_pp_weight == pytest.approx(0.0)
    assert steering == pytest.approx(-0.10)

    motion.latest_target_path.source = "global_route_local_path"
    motion.hybrid_pp_weight = 0.0
    motion.compute_pp_stanley_steering()
    assert not motion.stanley_only_path_active
    assert motion.hybrid_target_pp_weight == pytest.approx(0.35)


def test_shipped_straight_tuning_keeps_the_high_speed_heading_gain() -> None:
    # 2026-10-23 tuning (reports 261023_dev추종기_보강_개발계획): damping 1.1 keeps the straight heading
    # gain near the old fixed 0.30 up to 13.6 m/s (0.295; 0.95 gave 0.25); 1.4 diverged with 0.08 s
    # more delay.
    import yaml
    params = yaml.safe_load(MOTION_YAML.read_text())["kaiev26_motion_control"]["ros__parameters"]
    damping = params["straight_damping_ratio"]
    assert 1.0 <= damping < 1.4

    def heading_gain(speed):
        return scheduled_stanley_gains(
            speed, params["straight_natural_frequency_radps"], damping, params["wheelbase_m"],
            params["stanley_softening_speed_mps"], params["straight_gain_full_speed_mps"])[0]

    assert heading_gain(10.0) >= 0.38
    assert heading_gain(11.0) >= 0.35
    assert heading_gain(13.6) >= 0.29


def test_shipped_friction_push_changes_only_the_high_speed_value() -> None:
    import yaml
    params = yaml.safe_load(MOTION_YAML.read_text())["kaiev26_motion_control"]["ros__parameters"]
    assert params["steering_friction_speed_mps"] == [3.0, 6.5, 12.0]
    # Slow avoidance, stops and restarts keep the 10/22 values; 12 m/s and above push 0.8 deg.
    assert params["steering_friction_comp_deg"] == [1.0, 0.85, 0.8]


def test_standstill_brake_release_lets_the_servo_go_and_brakes_again_when_rolling() -> None:
    # 2026-10-24: Decision tags a planned stop held at standstill for 2 s; the target stays 0.
    motion = make_motion(
        commanded_speed=0.0, vehicle_speed=0.0, stop_target_distance=0.0
    )
    command = ActuatorCommand()
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.brake_engage

    motion.latest_target_speed.constraints = ["FINISH_HELD", "STANDSTILL_BRAKE_RELEASE"]
    motion.apply_nominal_motion(command, dt=0.1)
    assert not command.brake_engage
    assert command.speed_target_mps == 0.0
    assert motion.standstill_brake_released

    # Rolling above the stopped speed brakes again before Decision drops the tag.
    motion.latest_vehicle_state.speed_mps = 0.3
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.brake_engage
    assert command.speed_target_mps == 0.0
    assert not motion.standstill_brake_released


def test_launch_after_a_standstill_release_needs_no_release_delay() -> None:
    motion = make_motion(
        commanded_speed=0.0, vehicle_speed=0.0, stop_target_distance=0.0
    )
    command = ActuatorCommand()
    motion.apply_nominal_motion(command, dt=0.1)
    motion.latest_target_speed.constraints = ["RED_LIGHT_HOLD", "STANDSTILL_BRAKE_RELEASE"]
    for _ in range(10):
        motion.apply_nominal_motion(command, dt=0.1)
    assert not command.brake_engage and not motion.brake_release_pending

    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.target_speed_mps = 4.0
    motion.latest_target_speed.constraints = []
    motion.apply_nominal_motion(command, dt=0.1)
    assert not command.brake_engage
    assert command.speed_target_mps == pytest.approx(0.1)
