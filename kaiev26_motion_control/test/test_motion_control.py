"""분할 전 `test_control_command_gate.py`의 계획정지 판정을 그대로 옮겼다.

이 시험이 고정하는 것은 판단 소유 절반이다 — 속도 ramp와 계획정지 제동 시점.
안전 override는 Control의 bridge 시험이 갖는다.
"""
import math

from geometry_msgs.msg import Point
import pytest

from kaiev26_motion_control.motion_control_node import (
    adaptive_lookahead_decision,
    adaptive_stanley_profile,
    blend_lateral_steering,
    curvature_speed_limit_mps,
    filtered_steering_rate,
    hybrid_pp_weight_for_phase,
    interpolated_lookahead_target,
    LookaheadDecision,
    MotionControlNode,
    smoothstep_scale,
    slew_lookahead,
    stanley_path_reference,
    stanley_preview_distance,
    stanley_steering_decision,
    straight_corridor_decision,
    sustained_curvature_profile,
    validate_lateral_controller,
)
from kaiev26_msgs.msg import ActuatorCommand, Centerline, TargetSpeed, VehicleState


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
    motion.curvature_speed_planning_enabled = False
    motion.curvature_speed_min_mps = 3.2
    motion.curvature_speed_limit_mps = math.inf
    motion.signal_release_accel_mps2 = 2.0
    motion.signal_release_boost_speed_mps = 3.8
    motion.signal_release_boost_active = False
    motion.max_decel_mps2 = 2.0
    motion.stop_speed_epsilon_mps = 0.05
    motion.planned_stop_brake_distance_m = 2.0
    motion.stopped_steering_hold_speed_mps = 0.10
    motion.planned_stop_steering_align_distance_m = 2.0
    motion.stop_release_steering_resume_speed_mps = 0.30
    motion.planned_stop_steering_hold_active = False
    motion.max_steering_rad = 0.4363
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
    motion.compute_pure_pursuit_steering = lambda: 0.0
    return motion


def test_smoothstep_scale_has_no_threshold_jump() -> None:
    assert smoothstep_scale(0.07, 0.08, 0.25) == 0.0
    assert 0.0 < smoothstep_scale(0.165, 0.08, 0.25) < 1.0
    assert smoothstep_scale(0.26, 0.08, 0.25) == 1.0


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


def test_straight_corridor_only_softens_small_global_route_errors() -> None:
    path = [path_point(float(x), 0.03) for x in range(20)]
    small = straight_corridor_decision(
        path,
        "global_route_local_path",
        "straight",
        3.2,
        1.2,
        4.0,
        9.0,
        0.08,
        0.25,
        math.radians(0.5),
        math.radians(2.0),
        8.0,
    )
    assert small.eligible
    assert small.scale == 0.0

    test_tracking = straight_corridor_decision(
        path,
        "global_route_constant_speed",
        "straight",
        3.2,
        1.2,
        4.0,
        9.0,
        0.08,
        0.25,
        math.radians(0.5),
        math.radians(2.0),
        8.0,
    )
    assert test_tracking.eligible

    large_heading_path = [
        path_point(float(x), math.tan(math.radians(3.0)) * x) for x in range(20)
    ]
    large = straight_corridor_decision(
        large_heading_path,
        "global_route_local_path",
        "straight",
        3.2,
        1.2,
        4.0,
        9.0,
        0.08,
        0.25,
        math.radians(0.5),
        math.radians(2.0),
        8.0,
    )
    assert large.eligible
    assert large.scale == 1.0

    rejoin = straight_corridor_decision(
        path,
        "route_rejoin",
        "straight",
        3.2,
        1.2,
        4.0,
        9.0,
        0.08,
        0.25,
        math.radians(0.5),
        math.radians(2.0),
        8.0,
    )
    assert not rejoin.eligible
    assert rejoin.scale == 1.0


def test_large_straight_error_keeps_full_steering_authority() -> None:
    motion = make_motion(commanded_speed=3.2, vehicle_speed=3.2, target_speed=3.2)
    motion.commanded_steering_rad = 0.0
    motion.latest_vehicle_state.steering_rad = 0.0
    motion.latest_target_path = Centerline()
    motion.latest_target_path.source = "global_route_local_path"
    motion.latest_target_path.points = [path_point(float(x), 0.4) for x in range(20)]
    motion.lookahead_phase = "straight"
    motion.straight_corridor_enabled = True
    motion.straight_corridor_preview_time_s = 1.2
    motion.straight_corridor_preview_min_m = 4.0
    motion.straight_corridor_preview_max_m = 9.0
    motion.straight_corridor_deadband_m = 0.08
    motion.straight_corridor_full_m = 0.25
    motion.straight_heading_deadband_rad = math.radians(0.8)
    motion.straight_heading_full_rad = math.radians(2.0)
    motion.stanley_heading_window_m = 8.0
    motion.compute_lateral_steering = lambda: math.radians(4.0)

    result = motion.compute_limited_steering(dt=0.1)

    assert motion.straight_corridor.scale == 1.0
    assert math.degrees(result) == pytest.approx(4.0)


def test_small_straight_correction_only_uses_corridor_scale() -> None:
    motion = make_motion(commanded_speed=3.2, vehicle_speed=3.2, target_speed=3.2)
    motion.commanded_steering_rad = 0.0
    motion.latest_vehicle_state.steering_rad = 0.0
    motion.latest_target_path = Centerline()
    motion.latest_target_path.source = "global_route_constant_speed"
    motion.latest_target_path.points = [path_point(float(x), 0.20) for x in range(20)]
    motion.lookahead_phase = "straight"
    motion.straight_corridor_enabled = True
    motion.straight_corridor_preview_time_s = 1.2
    motion.straight_corridor_preview_min_m = 4.0
    motion.straight_corridor_preview_max_m = 9.0
    motion.straight_corridor_deadband_m = 0.08
    motion.straight_corridor_full_m = 0.25
    motion.straight_heading_deadband_rad = math.radians(0.8)
    motion.straight_heading_full_rad = math.radians(2.0)
    motion.stanley_heading_window_m = 8.0
    motion.compute_lateral_steering = lambda: math.radians(4.0)

    result = motion.compute_limited_steering(dt=0.1)

    assert 0.0 < motion.straight_corridor.scale < 1.0
    assert result == pytest.approx(
        math.radians(4.0) * motion.straight_corridor.scale
    )


def test_all_lateral_controllers_are_implemented() -> None:
    assert validate_lateral_controller("PURE_PURSUIT") == "pure_pursuit"
    assert validate_lateral_controller("STANLEY") == "stanley"
    assert validate_lateral_controller("PP_STANLEY") == "pp_stanley"
    assert validate_lateral_controller("FF_STANLEY") == "ff_stanley"


def test_unknown_lateral_controller_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown lateral_controller"):
        validate_lateral_controller("unknown")


def test_lateral_dispatch_keeps_current_pure_pursuit_implementation() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.lateral_controller = "pure_pursuit"
    motion.compute_pure_pursuit_steering = lambda: 0.12

    assert motion.compute_lateral_steering() == pytest.approx(0.12)


def test_lateral_dispatch_runs_stanley_implementation() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.lateral_controller = "stanley"
    motion.compute_stanley_steering = lambda: -0.08

    assert motion.compute_lateral_steering() == pytest.approx(-0.08)


def test_lateral_dispatch_runs_pp_stanley_implementation() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.lateral_controller = "pp_stanley"
    motion.compute_pp_stanley_steering = lambda: 0.04

    assert motion.compute_lateral_steering() == pytest.approx(0.04)


def test_lateral_dispatch_runs_ff_stanley_implementation() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.lateral_controller = "ff_stanley"
    motion.compute_ff_stanley_steering = lambda: -0.03

    assert motion.compute_lateral_steering() == pytest.approx(-0.03)


def test_ff_stanley_combines_path_feedforward_and_scaled_feedback() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
    motion.latest_target_path = Centerline()
    motion.latest_target_path.points = left_arc(10.0)
    motion.latest_vehicle_state = VehicleState()
    motion.latest_vehicle_state.speed_mps = 6.0
    motion.commanded_speed_mps = 0.0
    motion.ff_preview_time_s = 0.20
    motion.ff_preview_min_m = 2.0
    motion.ff_preview_max_m = 4.0
    motion.ff_curvature_sample_half_width_m = 2.0
    motion.ff_curvature_gain = 1.0
    motion.ff_curvature_limit_1pm = 0.20
    motion.ff_steering_limit_rad = 0.30
    motion.ff_stanley_feedback_scale = 0.50
    motion.wheelbase_m = 1.2991017929
    motion.max_steering_rad = 0.4363
    motion.compute_stanley_steering = lambda: 0.10

    steering = motion.compute_ff_stanley_steering()

    assert motion.ff_reference_curvature_1pm == pytest.approx(0.10, rel=0.25)
    assert motion.ff_stanley_feedback_rad == pytest.approx(0.05)
    assert steering == pytest.approx(motion.ff_steering_rad + 0.05)


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


def test_hybrid_preserves_pp_phase_and_slews_to_its_fixed_weight() -> None:
    motion = MotionControlNode.__new__(MotionControlNode)
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

    assert motion.hybrid_phase == "turn_approach"
    assert motion.lookahead_phase == "turn_approach"
    assert motion.hybrid_target_pp_weight == pytest.approx(0.65)
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


def test_curvature_speed_limit_blends_only_when_cruise_exceeds_curve_floor() -> None:
    assert curvature_speed_limit_mps(5.0, 0.0, 3.2) == pytest.approx(5.0)
    assert curvature_speed_limit_mps(5.0, 0.5, 3.2) == pytest.approx(4.1)
    assert curvature_speed_limit_mps(5.0, 1.0, 3.2) == pytest.approx(3.2)
    assert curvature_speed_limit_mps(3.0, 1.0, 3.2) == pytest.approx(3.0)


def test_nominal_motion_decelerates_toward_curvature_speed_limit() -> None:
    motion = make_motion(commanded_speed=5.0, vehicle_speed=5.0, target_speed=5.0)
    motion.latest_target_speed.need_stop = False
    motion.curvature_speed_planning_enabled = True
    motion.compute_curvature_speed_limit = lambda _cruise: 3.2
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert command.speed_target_mps == pytest.approx(4.8)
    assert command.brake_engage is False


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


def test_green_release_uses_faster_acceleration_without_changing_normal_limit() -> None:
    motion = make_motion(
        commanded_speed=0.0,
        vehicle_speed=0.0,
        target_speed=4.0,
        stop_target_distance=1.0e6,
    )
    motion.latest_target_speed.need_stop = False
    motion.latest_target_speed.source_behavior = "TRAFFIC_LIGHT"
    motion.latest_target_speed.constraints = ["SIGNAL_PASS_COMMITTED"]
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.5)

    assert command.speed_target_mps == 1.0
    assert motion.signal_release_boost_active is True

    motion.latest_target_speed.source_behavior = "OBSTACLE"
    motion.latest_target_speed.constraints = []
    assert motion.select_accel_limit(2.2) == 1.0
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
    motion.compute_pure_pursuit_steering = lambda: (_ for _ in ()).throw(
        AssertionError("stopped vehicle must not recalculate steering")
    )
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)

    assert command.steering_target_rad == 0.08
    assert command.brake_engage is True


def test_planned_stop_straightens_steering_while_vehicle_is_still_rolling() -> None:
    motion = make_motion(
        commanded_speed=0.6,
        vehicle_speed=0.5,
        target_speed=0.4,
        stop_target_distance=0.5,
    )
    motion.commanded_steering_rad = 0.2
    motion.compute_pure_pursuit_steering = lambda: 0.2
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
    motion.compute_pure_pursuit_steering = lambda: -0.2
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


def test_global_route_slows_near_finish_then_brakes_and_can_restart() -> None:
    motion = make_motion(4.0, 4.0, target_speed=4.0, stop_target_distance=100.0)
    motion.latest_target_speed.source_behavior = 'GLOBAL_ROUTE'
    motion.planned_stop_brake_distance_m = 1.0
    command = ActuatorCommand()

    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == 4.0
    assert not command.brake_engage

    motion.latest_target_speed.stop_target_distance = 2.0
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(3.8)
    assert not command.brake_engage

    motion.latest_target_speed.stop_target_distance = 1.0
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == 0.0
    assert command.brake_engage

    motion.latest_vehicle_state.speed_mps = 0.0
    motion.commanded_speed_mps = 0.0
    motion.latest_target_speed.stop_target_distance = 100.0
    motion.latest_target_speed.target_speed_mps = 0.0
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.brake_engage

    motion.latest_target_speed.target_speed_mps = 4.0
    motion.apply_nominal_motion(command, dt=0.1)
    assert command.speed_target_mps == pytest.approx(0.1)
    assert not command.brake_engage


@pytest.mark.parametrize('speed', [0.0, 0.27, -0.27])
def test_low_speed_steering_tracks_feedback_until_control_can_actuate(speed) -> None:
    motion = make_motion(0.0, speed, target_speed=3.0, stop_target_distance=100.0)
    motion.latest_target_speed.need_stop = False
    motion.commanded_steering_rad = 0.4
    motion.latest_vehicle_state.steering_rad = 0.08
    motion.compute_pure_pursuit_steering = lambda: -0.2
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


def test_stale_planning_still_emits_a_stop_command() -> None:
    """Planning 입력이 stale이면 정지 명령을 낸다.

    발행을 멈추는 것이 아니다. 실차 STM_A는 `fw_current`에서 command deadman이
    없어 발행이 끊기면 마지막 목표를 계속 보낸다 — 그래서 stale에는 명시적
    정지가 나가야 하고, 이것은 분할 전부터 있던 동작이다.
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
