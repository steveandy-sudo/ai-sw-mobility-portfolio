#!/usr/bin/env python3
"""판단 결과를 최종 차량 액추에이터 명령으로 바꾸는 노드."""
from __future__ import annotations

from dataclasses import dataclass
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import ActuatorCommand, Centerline, RouteContext, TargetSpeed, VehicleState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RosTime
from rcl_interfaces.msg import ParameterDescriptor

from kaiev26_motion_control.input_loss import InputLossLatch


INF_DISTANCE = 1.0e6
# Match lateral_ctrl's minimum drive speed for steering actuation.
MIN_STEERING_SPEED_MPS = 1.0 / 3.6
# TargetSpeed constraint from Decision: a planned stop that has stood still long enough; the
# brake servo may release while the target stays speed 0 (main_planning_engine_node).
STANDSTILL_BRAKE_RELEASE = "STANDSTILL_BRAKE_RELEASE"
# Paths cut from the global route, including the one carrying a stop target (stop point).
GLOBAL_ROUTE_SOURCES = frozenset({
    "global_route_local_path",
    "global_route_constant_speed",
    "longitudinal_stop_from_global_route_local_path",
})


@dataclass(frozen=True)
class LookaheadDecision:
    distance_m: float
    phase: str
    near_curvature_1pm: float
    preview_curvature_1pm: float
    curve_distance_m: float
    curve_severity: float = 0.0


@dataclass(frozen=True)
class StanleyDecision:
    steering_rad: float
    heading_error_rad: float
    cross_track_error_m: float
    cross_track_correction_rad: float
    target_x_m: float
    target_y_m: float
    valid: bool = False
    heading_correction_rad: float = 0.0


@dataclass(frozen=True)
class StanleyProfile:
    heading_gain: float
    cross_track_gain: float
    cross_track_correction_limit_rad: float
    heading_correction_limit_rad: float
    heading_window_m: float
    curve_severity: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def interpolate_table(value: float, points, values) -> float:
    """Piecewise-linear lookup, flat outside the table."""
    if not points:
        return 0.0
    if value <= points[0]:
        return values[0]
    for (a, b), (x, y) in zip(zip(points, points[1:]), zip(values, values[1:])):
        if value < b:
            return x + (y - x) * (value - a) / (b - a)
    return values[-1]


@dataclass(frozen=True)
class SteeringCompensationConfig:
    """Friction compensation in front of the steering actuator (2026-10-22 analysis).

    The KEYA torque PD has no integral, so the wheel stops anywhere inside a friction band of
    about ±0.85° around the target, offset about 0.37° to the left; the car runs straight with
    the encoder about 0.45° right. The command gets neutral_offset (−0.82°) and, while the
    encoder falls short of it, a push toward the shortfall by the friction of the current speed
    (friction_speeds_mps → friction_rad, flat outside). Values from the 09/19~10/17 autonomous
    logs; see reports 261022_조향액추에이터_모델_직선흔들림. After the 10/25 wheel refit and
    alignment the car runs straight at about −0.20° and the offset is −0.52° (reports
    261025_정렬후_좌우90도_추종기_재적합).
    """

    neutral_offset_rad: float
    friction_speeds_mps: tuple[float, ...]
    friction_rad: tuple[float, ...]
    error_start_rad: float
    error_full_rad: float
    error_ramp_s: float


def validate_steering_compensation(config: SteeringCompensationConfig) -> None:
    speeds, values = config.friction_speeds_mps, config.friction_rad
    scalars = (
        config.neutral_offset_rad, config.error_start_rad, config.error_full_rad, config.error_ramp_s,
    )
    if (
        not speeds or len(speeds) != len(values)
        or not all(math.isfinite(v) for v in (*speeds, *values, *scalars))
        or not all(a < b for a, b in zip(speeds, speeds[1:]))
        or any(v < 0.0 or v > math.radians(3.0) for v in values)
        or abs(config.neutral_offset_rad) > math.radians(3.0)
        or min(config.error_ramp_s, config.error_start_rad) < 0.0
        or config.error_full_rad <= config.error_start_rad
    ):
        raise ValueError(
            "steering compensation needs rising friction speeds with 0..3 deg values, an offset "
            "within 3 deg, a non-negative start error and ramp time and error_full > error_start"
        )


class SteeringCompensator:
    """State of the encoder_error friction compensation."""

    def __init__(self, config: SteeringCompensationConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.friction_value_rad = 0.0
        self.added_rad = 0.0

    def friction_at(self, speed_mps: float) -> float:
        return interpolate_table(
            abs(speed_mps), self.config.friction_speeds_mps, self.config.friction_rad
        )

    def _ramp(self, goal: float, amount: float, ramp_s: float, dt: float) -> float:
        if ramp_s <= 0.0 or amount <= 0.0:
            return goal
        step = 2.0 * amount * dt / ramp_s
        return self.friction_value_rad + clamp(goal - self.friction_value_rad, -step, step)

    def apply(self, command_rad: float, encoder_rad: float | None, speed_mps: float, dt: float) -> float:
        """Command with the offset and friction push added; dt is the control period."""
        config = self.config
        dt = max(0.0, dt)
        amount = self.friction_at(speed_mps)
        # Push only while the encoder falls short of the target by more than the start error,
        # fully from the full error: jitter below it is left to the actuator.
        goal = 0.0
        if encoder_rad is not None and math.isfinite(encoder_rad):
            error = command_rad + config.neutral_offset_rad - encoder_rad
            share = clamp(
                (abs(error) - config.error_start_rad)
                / (config.error_full_rad - config.error_start_rad), 0.0, 1.0,
            )
            goal = math.copysign(amount * share, error)
        self.friction_value_rad = self._ramp(goal, amount, config.error_ramp_s, dt)
        self.added_rad = config.neutral_offset_rad + self.friction_value_rad
        return command_rad + self.added_rad


def validate_route_segments(boundaries, phases, length):
    boundaries = tuple(float(value) for value in boundaries)
    phases = tuple(phases)
    if not boundaries and not phases:
        return boundaries, phases
    if (len(boundaries) != len(phases) + 1 or not phases
            or any(not math.isfinite(v) for v in boundaries)
            or not math.isfinite(length) or length <= 0
            or abs(boundaries[0]) > 1e-6 or abs(boundaries[-1] - length) > 1e-6
            or any(b <= a for a, b in zip(boundaries, boundaries[1:]))
            or any(p not in {'straight', 'turn_approach', 'turning',
                             'chained_turn', 'turn_exit'} for p in phases)):
        raise ValueError('invalid route phase table')
    return boundaries, phases


def route_segment_index(progress, boundaries):
    if not math.isfinite(progress) or not boundaries or not boundaries[0] <= progress <= boundaries[-1]:
        return -1
    import bisect
    return min(len(boundaries) - 2, bisect.bisect_right(boundaries, progress) - 1)


def filtered_steering_rate(
    previous_angle_rad: float,
    current_angle_rad: float,
    dt: float,
    previous_rate_radps: float,
    time_constant_s: float,
    maximum_rate_radps: float,
) -> float:
    """Estimate wheel-angle rate while suppressing encoder quantization noise."""
    if dt <= 0.0 or not all(
        math.isfinite(value)
        for value in (previous_angle_rad, current_angle_rad, previous_rate_radps)
    ):
        return 0.0
    maximum_rate = max(0.0, float(maximum_rate_radps))
    raw_rate = (float(current_angle_rad) - float(previous_angle_rad)) / dt
    if maximum_rate > 0.0:
        raw_rate = clamp(raw_rate, -maximum_rate, maximum_rate)
    alpha = dt / (max(0.0, float(time_constant_s)) + dt)
    return float(previous_rate_radps) + alpha * (
        raw_rate - float(previous_rate_radps)
    )


def hybrid_pp_weight_for_phase(
    phase: str,
    straight_weight: float,
    approach_weight: float,
    turning_weight: float,
    exit_weight: float,
) -> float:
    """Return a fixed PP ratio for each debuggable route-shape phase."""
    weights = {
        "turn_approach": approach_weight,
        "turning": turning_weight,
        "chained_turn": turning_weight,
        "turn_exit": exit_weight,
    }
    return clamp(float(weights.get(phase, straight_weight)), 0.0, 1.0)


def blend_lateral_steering(
    pure_pursuit_rad: float,
    stanley_rad: float,
    pure_pursuit_weight: float,
) -> float:
    weight = clamp(float(pure_pursuit_weight), 0.0, 1.0)
    return (1.0 - weight) * float(stanley_rad) + weight * float(
        pure_pursuit_rad
    )


def scheduled_stanley_gains(
    speed_mps: float,
    natural_frequency_radps: float,
    damping_ratio: float,
    wheelbase_m: float,
    softening_speed_mps: float,
    minimum_speed_mps: float,
) -> tuple[float, float]:
    """Straight Stanley (heading, cross-track) gains that give the kinematic car the same
    natural frequency wn and damping z at every speed.

    Small angles: e'' = (v^2/L) d with d = kh*psi + ke*e_front/(v+vs) and e_front = e + L*psi,
    so wn^2 = v^2 ke / (L (v+vs)) and 2 z wn = v (kh + ke L/(v+vs)) / L. Below
    minimum_speed_mps the gains of that speed are kept.
    """
    v = max(float(minimum_speed_mps), abs(float(speed_mps)), 0.1)
    wn = max(0.0, float(natural_frequency_radps))
    denominator = v + max(0.0, float(softening_speed_mps))
    cross_track = wn * wn * wheelbase_m * denominator / (v * v)
    heading = 2.0 * float(damping_ratio) * wn * wheelbase_m / v - cross_track * wheelbase_m / denominator
    return max(0.05, heading), cross_track


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    z: float = 0.0


def latency_shifted_points(
    points, age_s: float, speed_mps: float, yaw_rate_radps: float
) -> list[PathPoint]:
    """Move a vehicle-frame path into the frame the car has reached age_s after its stamp
    (constant speed and yaw rate over that time)."""
    yaw = float(yaw_rate_radps) * age_s
    distance = float(speed_mps) * age_s
    dx = distance * math.cos(0.5 * yaw)
    dy = distance * math.sin(0.5 * yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for point in points:
        x = float(point.x) - dx
        y = float(point.y) - dy
        out.append(PathPoint(c * x + s * y, -s * x + c * y))
    return out


@dataclass(frozen=True)
class TrackingPath:
    """Target path seen by the trackers: the received message or its latency-shifted copy."""
    header: object
    points: list
    source: str


def stamp_age_sec(now, stamp) -> float:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return INF_DISTANCE
    age = now.nanoseconds - RosTime.from_msg(stamp).nanoseconds
    return max(0.0, age * 1.0e-9)


def _path_xy(points) -> list[tuple[float, float]]:
    cleaned: list[tuple[float, float]] = []
    for point in points:
        x = float(point.x)
        y = float(point.y)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if cleaned and math.hypot(x - cleaned[-1][0], y - cleaned[-1][1]) < 1.0e-6:
            continue
        cleaned.append((x, y))
    return cleaned


def _downstream_path(points) -> list[tuple[float, float]]:
    path = _path_xy(points)
    if len(path) < 2:
        return path

    best_distance2 = math.inf
    best_index = 0
    best_ratio = 0.0
    best_point = path[0]
    for index, (start, end) in enumerate(zip(path, path[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length2 = dx * dx + dy * dy
        if length2 <= 1.0e-12:
            continue
        ratio = clamp(-(start[0] * dx + start[1] * dy) / length2, 0.0, 1.0)
        projected = (start[0] + ratio * dx, start[1] + ratio * dy)
        distance2 = projected[0] * projected[0] + projected[1] * projected[1]
        if distance2 < best_distance2:
            best_distance2 = distance2
            best_index = index
            best_ratio = ratio
            best_point = projected

    downstream = [best_point]
    first_end = path[best_index + 1]
    if best_ratio < 1.0 - 1.0e-9:
        downstream.append(first_end)
    downstream.extend(path[best_index + 2:])
    deduplicated = [downstream[0]]
    for point in downstream[1:]:
        if math.hypot(
            point[0] - deduplicated[-1][0],
            point[1] - deduplicated[-1][1],
        ) >= 1.0e-6:
            deduplicated.append(point)
    return deduplicated


def _path_length(path: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(end[0] - start[0], end[1] - start[1])
        for start, end in zip(path, path[1:])
    )


def _sample_path(
    path: list[tuple[float, float]], distance_m: float
) -> tuple[float, float]:
    if not path:
        return (0.0, 0.0)
    remaining = max(0.0, distance_m)
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1.0e-9:
            continue
        if remaining <= length:
            ratio = remaining / length
            return (start[0] + ratio * dx, start[1] + ratio * dy)
        remaining -= length
    return path[-1]


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def stanley_path_reference(
    points,
    front_axle_offset_m: float,
    heading_window_m: float,
    preview_distance_m: float = 0.0,
) -> tuple[float, float, float, float] | None:
    """Return previewed front-reference heading, signed CTE and path point."""
    path = _path_xy(points)
    if len(path) < 2:
        return None

    front_x = max(0.0, float(front_axle_offset_m) + float(preview_distance_m))
    cumulative = [0.0]
    best_distance2 = math.inf
    best_index = 0
    best_ratio = 0.0
    best_point = path[0]
    for index, (start, end) in enumerate(zip(path, path[1:])):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length2 = dx * dx + dy * dy
        length = math.sqrt(length2)
        cumulative.append(cumulative[-1] + length)
        if length2 <= 1.0e-12:
            continue
        ratio = clamp(
            ((front_x - start[0]) * dx - start[1] * dy) / length2,
            0.0,
            1.0,
        )
        projected = (start[0] + ratio * dx, start[1] + ratio * dy)
        distance2 = (projected[0] - front_x) ** 2 + projected[1] ** 2
        if distance2 < best_distance2:
            best_distance2 = distance2
            best_index = index
            best_ratio = ratio
            best_point = projected

    segment_length = cumulative[best_index + 1] - cumulative[best_index]
    projection_s = cumulative[best_index] + best_ratio * segment_length
    half_window = max(0.0, float(heading_window_m)) * 0.5
    before = _sample_path(path, max(0.0, projection_s - half_window))
    after = _sample_path(
        path,
        min(cumulative[-1], projection_s + half_window),
    )
    tangent_x = after[0] - before[0]
    tangent_y = after[1] - before[1]
    if math.hypot(tangent_x, tangent_y) <= 1.0e-6:
        start = path[best_index]
        end = path[best_index + 1]
        tangent_x = end[0] - start[0]
        tangent_y = end[1] - start[1]
    tangent_length = math.hypot(tangent_x, tangent_y)
    if tangent_length <= 1.0e-6:
        return None

    tangent_x /= tangent_length
    tangent_y /= tangent_length
    heading_error = _normalize_angle(math.atan2(tangent_y, tangent_x))
    # The path frame is x-forward/y-left. Positive CTE therefore requests left steer.
    cross_track_error = (
        tangent_x * best_point[1]
        - tangent_y * (best_point[0] - front_x)
    )
    return heading_error, cross_track_error, best_point[0], best_point[1]


def stanley_steering_decision(
    points,
    speed_mps: float,
    front_axle_offset_m: float,
    heading_gain: float,
    cross_track_gain: float,
    softening_speed_mps: float,
    cross_track_correction_limit_rad: float,
    heading_window_m: float,
    heading_correction_limit_rad: float = math.inf,
    preview_distance_m: float = 0.0,
) -> StanleyDecision:
    reference = stanley_path_reference(
        points,
        front_axle_offset_m,
        heading_window_m,
        preview_distance_m,
    )
    if reference is None:
        return StanleyDecision(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)

    heading_error, cross_track_error, target_x, target_y = reference
    correction = math.atan2(
        max(0.0, cross_track_gain) * cross_track_error,
        max(1.0e-3, abs(float(speed_mps)) + max(0.0, softening_speed_mps)),
    )
    correction_limit = max(0.0, float(cross_track_correction_limit_rad))
    correction = clamp(correction, -correction_limit, correction_limit)
    heading_correction = max(0.0, heading_gain) * heading_error
    heading_limit = max(0.0, float(heading_correction_limit_rad))
    heading_correction = clamp(heading_correction, -heading_limit, heading_limit)
    steering = heading_correction + correction
    return StanleyDecision(
        steering,
        heading_error,
        cross_track_error,
        correction,
        target_x,
        target_y,
        True,
        heading_correction,
    )


def adaptive_stanley_profile(
    curve_severity: float,
    heading_gain: float,
    curve_heading_gain: float,
    cross_track_gain: float,
    curve_cross_track_gain: float,
    cross_track_correction_limit_rad: float,
    curve_cross_track_correction_limit_rad: float,
    heading_correction_limit_rad: float,
    curve_heading_correction_limit_rad: float,
    heading_window_m: float,
    curve_heading_window_m: float,
) -> StanleyProfile:
    """Blend straight and curve Stanley settings using the shared path severity."""
    severity = clamp(float(curve_severity), 0.0, 1.0)

    def blend(straight: float, curve: float) -> float:
        return float(straight) + severity * (float(curve) - float(straight))

    return StanleyProfile(
        heading_gain=max(0.0, blend(heading_gain, curve_heading_gain)),
        cross_track_gain=max(
            0.0, blend(cross_track_gain, curve_cross_track_gain)
        ),
        cross_track_correction_limit_rad=max(
            0.0,
            blend(
                cross_track_correction_limit_rad,
                curve_cross_track_correction_limit_rad,
            ),
        ),
        heading_correction_limit_rad=max(
            0.0,
            blend(
                heading_correction_limit_rad,
                curve_heading_correction_limit_rad,
            ),
        ),
        heading_window_m=max(
            0.0, blend(heading_window_m, curve_heading_window_m)
        ),
        curve_severity=severity,
    )


def stanley_preview_distance(
    speed_mps: float,
    curve_severity: float,
    preview_time_s: float,
    maximum_m: float,
) -> float:
    """Compensate steering delay only while approaching or driving a curve."""
    delay_distance = clamp(
        abs(float(speed_mps)) * max(0.0, float(preview_time_s)),
        0.0,
        max(0.0, float(maximum_m)),
    )
    return delay_distance * clamp(float(curve_severity), 0.0, 1.0)


def _signed_curvature(
    first: tuple[float, float],
    middle: tuple[float, float],
    last: tuple[float, float],
) -> float:
    first_leg = math.hypot(middle[0] - first[0], middle[1] - first[1])
    second_leg = math.hypot(last[0] - middle[0], last[1] - middle[1])
    chord = math.hypot(last[0] - first[0], last[1] - first[1])
    if min(first_leg, second_leg, chord) <= 1.0e-6:
        return 0.0
    cross = (middle[0] - first[0]) * (last[1] - first[1]) - (
        middle[1] - first[1]
    ) * (last[0] - first[0])
    return 2.0 * cross / (first_leg * second_leg * chord)


def path_curvature_profile(
    points,
    preview_distance_m: float,
    sample_half_width_m: float = 2.0,
) -> list[tuple[float, float]]:
    path = _downstream_path(points)
    total_length = _path_length(path)
    half_width = max(0.5, sample_half_width_m)
    end_distance = min(max(0.0, preview_distance_m), total_length - half_width)
    if end_distance < half_width:
        return []

    profile = []
    center = half_width
    while center <= end_distance + 1.0e-6:
        profile.append(
            (
                center,
                _signed_curvature(
                    _sample_path(path, center - half_width),
                    _sample_path(path, center),
                    _sample_path(path, center + half_width),
                ),
            )
        )
        center += 1.0
    return profile


def median_curvature_profile(
    profile: list[tuple[float, float]],
    window_size: int = 3,
) -> list[tuple[float, float]]:
    if window_size <= 1 or len(profile) < 2:
        return list(profile)
    radius = max(1, int(window_size) // 2)
    smoothed = []
    for index, (distance, _) in enumerate(profile):
        start = max(0, index - radius)
        end = min(len(profile), index + radius + 1)
        values = sorted(curvature for _, curvature in profile[start:end])
        midpoint = len(values) // 2
        if len(values) % 2:
            median = values[midpoint]
        else:
            median = 0.5 * (values[midpoint - 1] + values[midpoint])
        smoothed.append((distance, median))
    return smoothed


def sustained_curvature_profile(
    profile: list[tuple[float, float]],
    threshold_1pm: float,
    minimum_samples: int,
) -> list[tuple[float, float]]:
    required = max(1, int(minimum_samples))
    sustained: list[tuple[float, float]] = []
    run: list[tuple[float, float]] = []
    run_sign = 0

    def flush() -> None:
        if len(run) >= required:
            sustained.extend(run)

    for item in profile:
        curvature = item[1]
        if curvature >= threshold_1pm:
            sign = 1
        elif curvature <= -threshold_1pm:
            sign = -1
        else:
            sign = 0
        if sign == 0:
            flush()
            run = []
            run_sign = 0
        elif not run or sign == run_sign:
            run.append(item)
            run_sign = sign
        else:
            flush()
            run = [item]
            run_sign = sign
    flush()
    return sustained


def adaptive_lookahead_decision(
    points,
    speed_mps: float,
    base_m: float,
    gain_s: float,
    minimum_m: float,
    maximum_m: float,
    curve_start_1pm: float,
    curve_full_1pm: float,
    near_window_m: float,
    preview_distance_m: float,
    curve_exit_1pm: float | None = None,
    curve_speed_gain_s: float = 0.0,
    curve_minimum_samples: int = 1,
    curve_active: bool = False,
) -> LookaheadDecision:
    straight_distance = clamp(
        base_m + max(0.0, speed_mps) * gain_s,
        minimum_m,
        maximum_m,
    )
    profile = median_curvature_profile(
        path_curvature_profile(points, preview_distance_m)
    )
    if not profile:
        return LookaheadDecision(straight_distance, "straight", 0.0, 0.0, math.inf)

    exit_threshold = (
        curve_start_1pm
        if curve_exit_1pm is None
        else clamp(curve_exit_1pm, 0.0, curve_start_1pm)
    )
    active_threshold = exit_threshold if curve_active else curve_start_1pm
    profile = sustained_curvature_profile(
        profile,
        active_threshold,
        curve_minimum_samples,
    )
    if not profile:
        return LookaheadDecision(straight_distance, "straight", 0.0, 0.0, math.inf)

    near = [abs(curvature) for distance, curvature in profile if distance <= near_window_m]
    preview = [
        (distance, curvature)
        for distance, curvature in profile
        if distance <= preview_distance_m
    ]
    near_curvature = max(near, default=0.0)
    preview_curvature = max((abs(curvature) for _, curvature in preview), default=0.0)
    curve_distances = [
        distance
        for distance, curvature in preview
        if abs(curvature) >= active_threshold
    ]
    curve_distance = min(curve_distances, default=math.inf)
    curvature_span = max(1.0e-6, curve_full_1pm - active_threshold)

    def severity(curvature: float) -> float:
        return clamp((curvature - active_threshold) / curvature_span, 0.0, 1.0)

    near_severity = severity(near_curvature)
    preview_severity = severity(preview_curvature)
    if math.isfinite(curve_distance):
        preview_weight = clamp(
            (preview_distance_m - curve_distance)
            / max(1.0e-6, preview_distance_m - near_window_m),
            0.0,
            1.0,
        )
    else:
        preview_weight = 0.0
    effective_severity = max(near_severity, preview_severity * preview_weight)
    curve_floor = clamp(
        minimum_m + max(0.0, speed_mps) * max(0.0, curve_speed_gain_s),
        minimum_m,
        straight_distance,
    )
    distance = straight_distance + (curve_floor - straight_distance) * effective_severity

    curved_signs = {
        1 if curvature > 0.0 else -1
        for _, curvature in preview
        if abs(curvature) >= active_threshold
    }
    if near_severity > 0.0:
        phase = "chained_turn" if len(curved_signs) > 1 else "turning"
    elif effective_severity > 0.0:
        phase = "turn_approach"
    else:
        phase = "straight"
    return LookaheadDecision(
        clamp(distance, minimum_m, maximum_m),
        phase,
        near_curvature,
        preview_curvature,
        curve_distance,
        effective_severity,
    )


def slew_lookahead(
    current_m: float | None,
    target_m: float,
    dt: float,
    shorten_rate_mps: float,
    lengthen_rate_mps: float,
) -> float:
    if current_m is None or not math.isfinite(current_m):
        return target_m
    rate = shorten_rate_mps if target_m < current_m else lengthen_rate_mps
    max_delta = max(0.0, rate) * max(0.0, dt)
    return current_m + clamp(target_m - current_m, -max_delta, max_delta)


def interpolated_lookahead_target(points, lookahead_m: float) -> tuple[float, float] | None:
    path = _downstream_path(points)
    if len(path) < 2:
        return path[0] if path else None
    radius = max(0.5, lookahead_m)
    for start, end in zip(path, path[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        quadratic = dx * dx + dy * dy
        if quadratic <= 1.0e-12:
            continue
        linear = 2.0 * (start[0] * dx + start[1] * dy)
        constant = start[0] * start[0] + start[1] * start[1] - radius * radius
        discriminant = linear * linear - 4.0 * quadratic * constant
        if discriminant < 0.0:
            continue
        root = math.sqrt(discriminant)
        ratios = sorted(
            (
                (-linear - root) / (2.0 * quadratic),
                (-linear + root) / (2.0 * quadratic),
            )
        )
        for ratio in ratios:
            if -1.0e-9 <= ratio <= 1.0 + 1.0e-9:
                ratio = clamp(ratio, 0.0, 1.0)
                candidate = (start[0] + ratio * dx, start[1] + ratio * dy)
                if candidate[0] > 0.05:
                    return candidate
    fallback = _sample_path(path, min(radius, _path_length(path)))
    if fallback[0] > 0.05:
        return fallback
    forward = [point for point in path if point[0] > 0.1]
    return forward[-1] if forward else None


class MotionControlNode(Node):
    """Convert a planned local path and speed into bounded actuator commands."""

    def __init__(self) -> None:
        super().__init__("kaiev26_motion_control")
        self.declare_parameter("target_path_topic", "/planning/target_path")
        self.declare_parameter("target_speed_topic", "/planning/target_speed")
        self.declare_parameter("vehicle_state_topic", "/vehicle/state")
        self.declare_parameter("command_topic", "/planning/command")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("target_stale_sec", 0.35)
        self.declare_parameter("target_stale_brake_delay_sec", 1.0)
        self.declare_parameter("wheelbase_m", 1.2991017929)
        self.declare_parameter("max_steering_rad", 0.4363)
        self.declare_parameter("steering_neutral_offset_deg", -0.52)
        self.declare_parameter("steering_friction_speed_mps", [3.0, 6.5, 12.0])
        self.declare_parameter("steering_friction_comp_deg", [1.0, 0.85, 0.8])
        self.declare_parameter("steering_error_start_deg", 0.2)
        self.declare_parameter("steering_error_full_deg", 0.5)
        self.declare_parameter("steering_error_ramp_s", 0.1)
        self.declare_parameter("steering_rate_limit_radps", 0.0)
        self.declare_parameter("steering_rate_filter_time_constant_s", 0.15)
        self.declare_parameter("steering_rate_estimate_limit_radps", 2.0943951024)
        self.declare_parameter("straight_natural_frequency_radps", 1.5)
        self.declare_parameter("straight_damping_ratio", 1.1)
        self.declare_parameter("straight_gain_blend_start_mps", 3.0)
        self.declare_parameter("straight_gain_full_speed_mps", 6.0)
        self.declare_parameter("straight_integral_time_s", 4.0)
        self.declare_parameter("straight_integral_limit_deg", 1.5)
        self.declare_parameter("straight_integral_min_speed_mps", 4.0)
        self.declare_parameter("straight_integral_max_error_m", 0.6)
        self.declare_parameter("path_latency_max_s", 0.3)
        self.declare_parameter("straight_encoder_deg", -0.20)
        self.declare_parameter("lookahead_base_m", 4.0)
        self.declare_parameter("lookahead_gain_s", 0.9)
        self.declare_parameter("lookahead_min_m", 3.6)
        self.declare_parameter("lookahead_max_m", 9.5)
        self.declare_parameter("lookahead_curve_start_1pm", 0.015)
        self.declare_parameter("lookahead_curve_exit_1pm", 0.010)
        self.declare_parameter("lookahead_curve_full_1pm", 0.070)
        self.declare_parameter("lookahead_curve_speed_gain_s", 0.30)
        self.declare_parameter("lookahead_curve_minimum_samples", 3)
        self.declare_parameter("lookahead_near_window_m", 4.0)
        self.declare_parameter("lookahead_preview_distance_m", 12.0)
        self.declare_parameter("lookahead_shorten_rate_mps", 8.0)
        self.declare_parameter("lookahead_lengthen_rate_mps", 8.0)
        self.declare_parameter("stanley_heading_gain", 0.30)
        self.declare_parameter("stanley_curve_heading_gain", 0.45)
        self.declare_parameter("stanley_cross_track_gain", 0.25)
        self.declare_parameter("stanley_curve_cross_track_gain", 0.35)
        self.declare_parameter("stanley_softening_speed_mps", 3.0)
        self.declare_parameter("stanley_cross_track_correction_limit_rad", 0.05)
        self.declare_parameter(
            "stanley_curve_cross_track_correction_limit_rad", 0.07
        )
        self.declare_parameter("stanley_heading_correction_limit_rad", 0.10)
        self.declare_parameter("stanley_curve_heading_correction_limit_rad", 0.16)
        self.declare_parameter("stanley_heading_window_m", 8.0)
        self.declare_parameter("stanley_curve_heading_window_m", 6.0)
        self.declare_parameter("stanley_curve_preview_time_s", 0.18)
        self.declare_parameter("stanley_curve_preview_max_m", 3.5)
        self.declare_parameter("hybrid_pp_straight_weight", 0.0)
        self.declare_parameter("hybrid_pp_approach_weight", 0.65)
        self.declare_parameter("hybrid_pp_turning_weight", 0.35)
        self.declare_parameter("hybrid_pp_exit_weight", 0.10)
        self.declare_parameter("hybrid_blend_rate_per_s", 6.0)
        # Paths whose source starts with one of these use Stanley only: a pure pursuit
        # lookahead point beyond an obstacle would cut through it. None by default; the Midan
        # avoidance launch (case 17) names the bypass paths. [""] keeps the type a string array.
        self.declare_parameter("stanley_only_path_source_prefixes", [""])
        self.declare_parameter("hybrid_route_length_m", 0.0)
        self.declare_parameter("hybrid_route_segment_boundaries_m", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("hybrid_route_segment_phases", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("max_accel_mps2", 1.0)
        self.declare_parameter("signal_release_accel_mps2", 2.0)
        self.declare_parameter("signal_release_boost_speed_mps", 5.0)
        self.declare_parameter("max_decel_mps2", 2.0)
        self.declare_parameter("stop_pre_decel_mps2", 10.0)
        self.declare_parameter("obstacle_decel_mps2", 10.0)
        self.declare_parameter("stop_speed_epsilon_mps", 0.05)
        self.declare_parameter("planned_stop_brake_distance_m", 2.0)
        self.declare_parameter("brake_release_delay_sec", 0.8)
        self.declare_parameter("stopped_steering_hold_speed_mps", 0.10)
        self.declare_parameter("planned_stop_steering_align_distance_m", 2.0)
        self.declare_parameter("stop_release_steering_resume_speed_mps", 0.30)

        self.target_stale_sec = float(self.get_parameter("target_stale_sec").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.max_steering_rad = float(self.get_parameter("max_steering_rad").value)
        compensation_config = SteeringCompensationConfig(
            neutral_offset_rad=math.radians(float(self.get_parameter("steering_neutral_offset_deg").value)),
            friction_speeds_mps=tuple(
                float(v) for v in self.get_parameter("steering_friction_speed_mps").value or []
            ),
            friction_rad=tuple(
                math.radians(float(v))
                for v in self.get_parameter("steering_friction_comp_deg").value or []
            ),
            error_start_rad=math.radians(float(self.get_parameter("steering_error_start_deg").value)),
            error_full_rad=math.radians(float(self.get_parameter("steering_error_full_deg").value)),
            error_ramp_s=float(self.get_parameter("steering_error_ramp_s").value),
        )
        validate_steering_compensation(compensation_config)
        self.straight_natural_frequency_radps = float(
            self.get_parameter("straight_natural_frequency_radps").value
        )
        self.straight_damping_ratio = float(self.get_parameter("straight_damping_ratio").value)
        self.straight_gain_blend_start_mps = float(self.get_parameter("straight_gain_blend_start_mps").value)
        self.straight_gain_full_speed_mps = float(self.get_parameter("straight_gain_full_speed_mps").value)
        self.straight_integral_time_s = float(self.get_parameter("straight_integral_time_s").value)
        self.straight_integral_limit_rad = math.radians(
            float(self.get_parameter("straight_integral_limit_deg").value)
        )
        self.straight_integral_min_speed_mps = float(
            self.get_parameter("straight_integral_min_speed_mps").value
        )
        self.straight_integral_max_error_m = float(self.get_parameter("straight_integral_max_error_m").value)
        self.path_latency_max_s = float(self.get_parameter("path_latency_max_s").value)
        self.straight_encoder_rad = math.radians(
            float(self.get_parameter("straight_encoder_deg").value)
        )
        if not (
            0.3 <= self.straight_natural_frequency_radps <= 3.0
            and 0.5 <= self.straight_damping_ratio <= 1.5
            and 0.0 <= self.straight_gain_blend_start_mps < self.straight_gain_full_speed_mps
            and self.straight_integral_max_error_m > 0.0
            and self.straight_integral_time_s > 0.0
            and 0.0 <= self.straight_integral_limit_rad <= math.radians(3.0)
            and 0.0 <= self.path_latency_max_s <= 0.5
            and abs(self.straight_encoder_rad) <= math.radians(3.0)
        ):
            raise ValueError(
                "straight tracking needs wn 0.3..3 rad/s, damping 0.5..1.5, 0 <= gain blend start < full "
                "speed, integral max error > 0, "
                "integral time > 0, integral limit 0..3 deg, latency max 0..0.5 s, "
                "straight encoder within 3 deg"
            )
        self.steering_compensator = SteeringCompensator(compensation_config)
        self.steering_rate_limit_radps = float(
            self.get_parameter("steering_rate_limit_radps").value
        )
        self.steering_rate_filter_time_constant_s = max(
            0.0,
            float(
                self.get_parameter("steering_rate_filter_time_constant_s").value
            ),
        )
        self.steering_rate_estimate_limit_radps = max(
            0.0,
            float(self.get_parameter("steering_rate_estimate_limit_radps").value),
        )
        self.lookahead_base_m = float(self.get_parameter("lookahead_base_m").value)
        self.lookahead_gain_s = float(self.get_parameter("lookahead_gain_s").value)
        self.lookahead_min_m = max(
            0.5, float(self.get_parameter("lookahead_min_m").value)
        )
        self.lookahead_max_m = max(
            self.lookahead_min_m,
            float(self.get_parameter("lookahead_max_m").value),
        )
        self.lookahead_curve_start_1pm = max(
            0.0, float(self.get_parameter("lookahead_curve_start_1pm").value)
        )
        self.lookahead_curve_exit_1pm = clamp(
            float(self.get_parameter("lookahead_curve_exit_1pm").value),
            0.0,
            self.lookahead_curve_start_1pm,
        )
        self.lookahead_curve_full_1pm = max(
            self.lookahead_curve_start_1pm + 1.0e-6,
            float(self.get_parameter("lookahead_curve_full_1pm").value),
        )
        self.lookahead_curve_speed_gain_s = max(
            0.0, float(self.get_parameter("lookahead_curve_speed_gain_s").value)
        )
        self.lookahead_curve_minimum_samples = max(
            1, int(self.get_parameter("lookahead_curve_minimum_samples").value)
        )
        self.lookahead_near_window_m = max(
            1.0, float(self.get_parameter("lookahead_near_window_m").value)
        )
        self.lookahead_preview_distance_m = max(
            self.lookahead_near_window_m + 1.0,
            float(self.get_parameter("lookahead_preview_distance_m").value),
        )
        self.lookahead_shorten_rate_mps = max(
            0.0, float(self.get_parameter("lookahead_shorten_rate_mps").value)
        )
        self.lookahead_lengthen_rate_mps = max(
            0.0, float(self.get_parameter("lookahead_lengthen_rate_mps").value)
        )
        self.stanley_heading_gain = max(
            0.0, float(self.get_parameter("stanley_heading_gain").value)
        )
        self.stanley_curve_heading_gain = max(
            0.0, float(self.get_parameter("stanley_curve_heading_gain").value)
        )
        self.stanley_cross_track_gain = max(
            0.0, float(self.get_parameter("stanley_cross_track_gain").value)
        )
        self.stanley_curve_cross_track_gain = max(
            0.0,
            float(self.get_parameter("stanley_curve_cross_track_gain").value),
        )
        self.stanley_softening_speed_mps = max(
            0.0, float(self.get_parameter("stanley_softening_speed_mps").value)
        )
        self.stanley_cross_track_correction_limit_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "stanley_cross_track_correction_limit_rad"
                ).value
            ),
        )
        self.stanley_curve_cross_track_correction_limit_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "stanley_curve_cross_track_correction_limit_rad"
                ).value
            ),
        )
        self.stanley_heading_correction_limit_rad = max(
            0.0,
            float(self.get_parameter("stanley_heading_correction_limit_rad").value),
        )
        self.stanley_curve_heading_correction_limit_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "stanley_curve_heading_correction_limit_rad"
                ).value
            ),
        )
        self.stanley_heading_window_m = max(
            0.0, float(self.get_parameter("stanley_heading_window_m").value)
        )
        self.stanley_curve_heading_window_m = max(
            0.0,
            float(self.get_parameter("stanley_curve_heading_window_m").value),
        )
        self.stanley_curve_preview_time_s = max(
            0.0,
            float(self.get_parameter("stanley_curve_preview_time_s").value),
        )
        self.stanley_curve_preview_max_m = max(
            0.0,
            float(self.get_parameter("stanley_curve_preview_max_m").value),
        )
        self.hybrid_pp_straight_weight = clamp(
            float(self.get_parameter("hybrid_pp_straight_weight").value),
            0.0,
            1.0,
        )
        self.hybrid_pp_approach_weight = clamp(
            float(self.get_parameter("hybrid_pp_approach_weight").value),
            0.0,
            1.0,
        )
        self.hybrid_pp_turning_weight = clamp(
            float(self.get_parameter("hybrid_pp_turning_weight").value),
            0.0,
            1.0,
        )
        self.hybrid_pp_exit_weight = clamp(
            float(self.get_parameter("hybrid_pp_exit_weight").value),
            0.0,
            1.0,
        )
        self.hybrid_blend_rate_per_s = max(
            0.0,
            float(self.get_parameter("hybrid_blend_rate_per_s").value),
        )
        self.stanley_only_path_source_prefixes = tuple(
            str(prefix).strip()
            for prefix in self.get_parameter("stanley_only_path_source_prefixes").value
            if str(prefix).strip()
        )
        self.stanley_only_path_active = False
        self.max_accel_mps2 = float(self.get_parameter("max_accel_mps2").value)
        self.signal_release_accel_mps2 = float(
            self.get_parameter("signal_release_accel_mps2").value
        )
        self.signal_release_boost_speed_mps = float(
            self.get_parameter("signal_release_boost_speed_mps").value
        )
        self.max_decel_mps2 = float(self.get_parameter("max_decel_mps2").value)
        self.stop_pre_decel_mps2 = float(self.get_parameter("stop_pre_decel_mps2").value)
        self.obstacle_decel_mps2 = float(self.get_parameter("obstacle_decel_mps2").value)
        self.stop_speed_epsilon_mps = float(
            self.get_parameter("stop_speed_epsilon_mps").value
        )
        self.planned_stop_brake_distance_m = max(
            0.0,
            float(self.get_parameter("planned_stop_brake_distance_m").value),
        )
        self.brake_release_delay_sec = max(
            0.0,
            float(self.get_parameter("brake_release_delay_sec").value),
        )
        self.stopped_steering_hold_speed_mps = max(
            0.0,
            float(self.get_parameter("stopped_steering_hold_speed_mps").value),
        )
        self.planned_stop_steering_align_distance_m = max(
            0.0,
            float(
                self.get_parameter("planned_stop_steering_align_distance_m").value
            ),
        )
        self.stop_release_steering_resume_speed_mps = max(
            self.stopped_steering_hold_speed_mps,
            float(
                self.get_parameter("stop_release_steering_resume_speed_mps").value
            ),
        )
        self.control_rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.input_loss = InputLossLatch(
            float(self.get_parameter("target_stale_brake_delay_sec").value),
            self.stop_speed_epsilon_mps,
        )

        self.latest_target_path: Centerline | None = None
        self.latest_target_speed: TargetSpeed | None = None
        self.latest_vehicle_state: VehicleState | None = None
        self.latest_route_context: RouteContext | None = None
        self.hybrid_route_length_m = float(
            self.get_parameter("hybrid_route_length_m").value
        )
        self.hybrid_route_phase_active = False

        self.hybrid_route_segment_boundaries_m, self.hybrid_route_segment_phases = validate_route_segments(
            self.get_parameter("hybrid_route_segment_boundaries_m").value or [],
            self.get_parameter("hybrid_route_segment_phases").value or [],
            self.hybrid_route_length_m,
        )
        self.hybrid_route_segment_index = -1
        self.hybrid_route_fallback_reason = 'not_evaluated'

        self.commanded_speed_mps = 0.0
        self.commanded_steering_rad = 0.0
        self.current_lookahead_m: float | None = None
        self.lookahead_phase = "uninitialized"
        self.lookahead_curve_active = False
        self.stanley_preview_distance_m = 0.0
        self.hybrid_pp_weight = self.hybrid_pp_straight_weight
        self.hybrid_target_pp_weight = self.hybrid_pp_straight_weight
        self.hybrid_phase = "straight"
        self.hybrid_pp_steering_rad = 0.0
        self.hybrid_stanley_steering_rad = 0.0
        self.lookahead_decision = LookaheadDecision(
            0.0, "uninitialized", 0.0, 0.0, math.inf
        )
        self.stanley_decision = StanleyDecision(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False
        )
        self.stanley_profile = adaptive_stanley_profile(
            0.0,
            self.stanley_heading_gain,
            self.stanley_curve_heading_gain,
            self.stanley_cross_track_gain,
            self.stanley_curve_cross_track_gain,
            self.stanley_cross_track_correction_limit_rad,
            self.stanley_curve_cross_track_correction_limit_rad,
            self.stanley_heading_correction_limit_rad,
            self.stanley_curve_heading_correction_limit_rad,
            self.stanley_heading_window_m,
            self.stanley_curve_heading_window_m,
        )
        self.steering_tracking_error_rad = 0.0
        self.actual_steering_rate_radps = 0.0
        self.actual_steering_rate_sample_rad: float | None = None
        self.actual_steering_rate_sample_ns: int | None = None
        self.raw_steering_rad = 0.0
        self.straight_integral_rad = 0.0
        self.path_latency_s = 0.0
        self.signal_release_boost_active = False
        self.planned_stop_steering_hold_active = False
        self.brake_command_active = False
        self.brake_release_pending = False
        self.brake_release_elapsed_sec = 0.0
        self.standstill_brake_released = False
        self.last_drive_mode: int | None = None
        self.last_control_time = self.get_clock().now()

        self.command_pub = self.create_publisher(
            ActuatorCommand,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("target_path_topic").value),
            self.on_target_path,
            10,
        )
        self.create_subscription(
            TargetSpeed,
            str(self.get_parameter("target_speed_topic").value),
            self.on_target_speed,
            10,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle_state,
            10,
        )
        if self.hybrid_route_segment_phases:
            self.create_subscription(
                RouteContext,
                str(self.get_parameter("route_context_topic").value),
                self.on_route_context,
                10,
            )
        self.create_timer(1.0 / self.control_rate_hz, self.on_control_timer)
        self.create_timer(0.2, self.publish_lookahead_diagnostic)

    def on_target_path(self, msg: Centerline) -> None:
        self.latest_target_path = msg

    def on_target_speed(self, msg: TargetSpeed) -> None:
        self.latest_target_speed = msg

    def on_vehicle_state(self, msg: VehicleState) -> None:
        entering_auto = (
            self.last_drive_mode == VehicleState.MODE_MANUAL
            and msg.mode == VehicleState.MODE_AUTO
        )
        if msg.mode == VehicleState.MODE_MANUAL or entering_auto:
            self.sync_command_state_to_vehicle(msg)
            self.reset_steering_rate_estimate(msg)
        else:
            self.update_steering_rate_estimate(msg)
        self.last_drive_mode = int(msg.mode)
        self.latest_vehicle_state = msg

    @staticmethod
    def vehicle_state_stamp_ns(msg: VehicleState) -> int | None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        return stamp_ns if stamp_ns > 0 else None

    def reset_steering_rate_estimate(self, msg: VehicleState) -> None:
        self.actual_steering_rate_radps = 0.0
        self.actual_steering_rate_sample_rad = float(msg.steering_rad)
        self.actual_steering_rate_sample_ns = self.vehicle_state_stamp_ns(msg)

    def update_steering_rate_estimate(self, msg: VehicleState) -> None:
        current = float(msg.steering_rad)
        stamp_ns = self.vehicle_state_stamp_ns(msg)
        previous = getattr(self, "actual_steering_rate_sample_rad", None)
        previous_ns = getattr(self, "actual_steering_rate_sample_ns", None)
        if previous is None or stamp_ns is None or previous_ns is None:
            self.reset_steering_rate_estimate(msg)
            return
        dt = (stamp_ns - previous_ns) * 1.0e-9
        if dt <= 0.0 or dt > 0.5:
            self.reset_steering_rate_estimate(msg)
            return
        self.actual_steering_rate_radps = filtered_steering_rate(
            previous,
            current,
            dt,
            getattr(self, "actual_steering_rate_radps", 0.0),
            self.steering_rate_filter_time_constant_s,
            self.steering_rate_estimate_limit_radps,
        )
        self.actual_steering_rate_sample_rad = current
        self.actual_steering_rate_sample_ns = stamp_ns

    def on_control_timer(self) -> None:
        now = self.get_clock().now()
        dt = max(
            1.0 / self.control_rate_hz,
            (now.nanoseconds - self.last_control_time.nanoseconds) * 1.0e-9,
        )
        self.last_control_time = now

        command = ActuatorCommand()
        command.header.stamp = now.to_msg()

        if self.manual_mode_active():
            self.input_loss_latch().reset()
            self.apply_manual_standby(command)
        else:
            targets_ok = not self.targets_are_stale(now)
            loss = self.input_loss_latch().update(
                now.nanoseconds * 1.0e-9,
                targets_ok,
                self.measured_speed_mps(),
                brake_engaged=bool(getattr(self, "brake_command_active", False)),
            )
            if loss.brake:
                self.apply_stop_motion(command)
            elif loss.stale:
                self.apply_stale_coast(command)
            else:
                self.apply_nominal_motion(command, dt)

        self.command_pub.publish(command)

    def manual_mode_active(self) -> bool:
        return (
            self.latest_vehicle_state is not None
            and self.latest_vehicle_state.mode == VehicleState.MODE_MANUAL
        )

    def sync_command_state_to_vehicle(self, vehicle: VehicleState) -> None:
        self.commanded_speed_mps = abs(float(vehicle.speed_mps))
        self.commanded_steering_rad = clamp(
            float(vehicle.steering_rad),
            -self.max_steering_rad,
            self.max_steering_rad,
        )
        self.signal_release_boost_active = False
        self.planned_stop_steering_hold_active = False
        compensator = getattr(self, "steering_compensator", None)
        if compensator is not None:
            compensator.reset()
        self.straight_integral_rad = 0.0
        self.reset_brake_release_sequence()
        self.current_lookahead_m = None
        self.lookahead_phase = "uninitialized"
        self.lookahead_curve_active = False
        self.stanley_preview_distance_m = 0.0
        self.hybrid_pp_weight = getattr(self, "hybrid_pp_straight_weight", 0.0)
        self.hybrid_target_pp_weight = self.hybrid_pp_weight
        self.hybrid_phase = "straight"
        self.hybrid_route_phase_active = False
        self.hybrid_pp_steering_rad = 0.0
        self.hybrid_stanley_steering_rad = 0.0
        self.stanley_only_path_active = False
        self.steering_tracking_error_rad = 0.0
        self.raw_steering_rad = self.commanded_steering_rad

    def apply_manual_standby(self, command: ActuatorCommand) -> None:
        assert self.latest_vehicle_state is not None
        self.sync_command_state_to_vehicle(self.latest_vehicle_state)
        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0
        command.brake_engage = False

    def targets_are_stale(self, now) -> bool:
        if self.latest_target_path is None or self.latest_target_speed is None:
            return True
        path_age = stamp_age_sec(now, self.latest_target_path.header.stamp)
        speed_age = stamp_age_sec(now, self.latest_target_speed.header.stamp)
        if path_age > self.target_stale_sec or speed_age > self.target_stale_sec:
            return True
        if (
            len(self.latest_target_path.points) < 2
            or self.latest_target_path.confidence <= 0.0
        ):
            return True
        return False

    def input_loss_latch(self) -> InputLossLatch:
        latch = getattr(self, "input_loss", None)
        if latch is None:
            latch = self.input_loss = InputLossLatch(
                1.0, getattr(self, "stop_speed_epsilon_mps", 0.05)
            )
        return latch

    def measured_speed_mps(self) -> float | None:
        vehicle = self.latest_vehicle_state
        return None if vehicle is None else abs(float(vehicle.speed_mps))

    def hold_measured_steering(self) -> None:
        if self.latest_vehicle_state is not None:
            self.commanded_steering_rad = clamp(
                float(self.latest_vehicle_state.steering_rad),
                -self.max_steering_rad,
                self.max_steering_rad,
            )

    def apply_stale_coast(self, command: ActuatorCommand) -> None:
        """Stale targets, first second (D2): speed 0 without brake, steering held where it is."""
        self.commanded_speed_mps = 0.0
        self.hold_measured_steering()
        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0
        command.brake_engage = False

    def apply_stop_motion(self, command: ActuatorCommand) -> None:
        self.commanded_speed_mps = 0.0
        self.mark_brake_engaged()
        if self.latest_vehicle_state is not None:
            self.commanded_steering_rad = clamp(
                float(self.latest_vehicle_state.steering_rad),
                -self.max_steering_rad,
                self.max_steering_rad,
            )
        else:
            self.commanded_steering_rad = 0.0

        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0
        command.brake_engage = True

    def apply_nominal_motion(self, command: ActuatorCommand, dt: float) -> None:
        assert self.latest_target_speed is not None
        target_speed = max(0.0, float(self.latest_target_speed.target_speed_mps))
        planned_stop = bool(self.latest_target_speed.need_stop)
        stop_target_distance = float(self.latest_target_speed.stop_target_distance)

        actual_speed = (
            abs(float(self.latest_vehicle_state.speed_mps))
            if self.latest_vehicle_state is not None
            else abs(float(self.commanded_speed_mps))
        )
        accel_limit = self.select_accel_limit(target_speed)
        self.commanded_speed_mps = self.ramp_speed(
            self.commanded_speed_mps, target_speed, dt, accel_limit,
            self.select_decel_limit(),
        )
        hold_brake = (
            planned_stop
            and target_speed <= self.stop_speed_epsilon_mps
            and (
                (
                    abs(float(self.commanded_speed_mps))
                    <= self.stop_speed_epsilon_mps
                    and actual_speed <= self.stop_speed_epsilon_mps
                )
                or stop_target_distance <= self.planned_stop_brake_distance_m
            )
        )
        stopped_for_planned_hold = (
            planned_stop
            and target_speed <= self.stop_speed_epsilon_mps
            and actual_speed <= getattr(self, "stopped_steering_hold_speed_mps", 0.10)
        )
        # Standing at a planned stop, Decision lets the brake servo go to spare it: speed 0
        # without brake. Rolling above the stopped speed brakes again here at once.
        self.standstill_brake_released = (
            hold_brake
            and stopped_for_planned_hold
            and STANDSTILL_BRAKE_RELEASE in self.latest_target_speed.constraints
        )
        # The path keeps steering the car while it brakes until it has slowed below the
        # steerable speed: a hard signal brake from 10 m/s rolls 17-18 m, and holding the wheel
        # there let the car drift 2.3 m and yaw 14 deg off the route (2026-10-17 final, Z1).
        # Below the steerable speed the car holds the measured angle. The real car only follows
        # this once lateral_ctrl keeps KEYA enabled under brake_engage (steering_actuator_node.cpp
        # requestImmediateDisable(COMMAND_BRAKE)).
        braking_while_rolling = hold_brake and actual_speed >= MIN_STEERING_SPEED_MPS
        if hold_brake and actual_speed > self.stop_speed_epsilon_mps and not braking_while_rolling:
            # D7: the car brakes and is slower than the steerable speed: the target is the
            # measured steering (hold).
            self.hold_measured_steering()
        elif self.latest_vehicle_state is not None and actual_speed < MIN_STEERING_SPEED_MPS:
            self.commanded_steering_rad = clamp(
                float(self.latest_vehicle_state.steering_rad),
                -self.max_steering_rad,
                self.max_steering_rad,
            )
            if stopped_for_planned_hold:
                self.planned_stop_steering_hold_active = True
        elif stopped_for_planned_hold:
            self.planned_stop_steering_hold_active = True
        elif (
            getattr(self, "planned_stop_steering_hold_active", False)
            and actual_speed
            < getattr(self, "stop_release_steering_resume_speed_mps", 0.30)
        ):
            pass
        else:
            self.planned_stop_steering_hold_active = False
            # Distance 0 means "brake now", not "at the stop line": no stop-line straightening.
            steering_scale = (
                1.0
                if braking_while_rolling and stop_target_distance <= 0.0
                else self.planned_stop_steering_scale(planned_stop, stop_target_distance)
            )
            self.commanded_steering_rad = self.compute_limited_steering(dt, steering_scale)
        self.update_steering_tracking_diagnostic()
        brake = hold_brake and not self.standstill_brake_released
        release_only = self.apply_brake_release_sequence(brake, dt)

        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = (
            0.0 if hold_brake or release_only else float(self.commanded_speed_mps)
        )
        command.brake_engage = brake

    def reset_brake_release_sequence(self) -> None:
        self.brake_command_active = False
        self.brake_release_pending = False
        self.brake_release_elapsed_sec = 0.0

    def mark_brake_engaged(self) -> None:
        self.brake_command_active = True
        self.brake_release_pending = False
        self.brake_release_elapsed_sec = 0.0

    def apply_brake_release_sequence(self, hold_brake: bool, dt: float) -> bool:
        """Keep traction at zero while the physical brake servo releases."""
        if hold_brake:
            self.commanded_speed_mps = 0.0
            self.mark_brake_engaged()
            return False

        if self.brake_command_active:
            self.brake_command_active = False
            self.brake_release_pending = self.brake_release_delay_sec > 0.0
            self.brake_release_elapsed_sec = 0.0
            return self.brake_release_pending

        if not self.brake_release_pending:
            return False

        self.commanded_speed_mps = 0.0
        self.brake_release_elapsed_sec += max(0.0, dt)
        if self.brake_release_elapsed_sec + 1.0e-9 >= self.brake_release_delay_sec:
            self.brake_release_pending = False
        return True

    def select_accel_limit(self, target_speed: float) -> float:
        target = self.latest_target_speed
        # max_accel_mps2 0 lifts the acceleration limit, release boost included.
        if target is None or self.max_accel_mps2 <= 0.0:
            self.signal_release_boost_active = False
            return self.max_accel_mps2

        constraints = set(target.constraints)
        release_boost_requested = bool(
            constraints.intersection(
                {"SIGNAL_PASS_COMMITTED", "SIGNAL_RELEASE", "TIMED_STOP_RELEASE"}
            )
        )
        if release_boost_requested and not target.need_stop:
            self.signal_release_boost_active = True
        if target.need_stop or target.source_behavior in {
            "EMERGENCY",
            "RECOVERY",
            "OBSTACLE",
            "FINISH",
        }:
            self.signal_release_boost_active = False

        boost_speed = getattr(self, "signal_release_boost_speed_mps", 5.0)
        if (
            self.signal_release_boost_active
            and target_speed > self.commanded_speed_mps
            and self.commanded_speed_mps < boost_speed
        ):
            return max(
                self.max_accel_mps2,
                getattr(self, "signal_release_accel_mps2", 2.0),
            )

        if self.commanded_speed_mps >= boost_speed or target_speed <= self.commanded_speed_mps:
            self.signal_release_boost_active = False
        return self.max_accel_mps2

    def select_decel_limit(self) -> float:
        target = self.latest_target_speed
        if target is not None and not target.need_stop:
            if target.source_behavior == "OBSTACLE":
                return self.obstacle_decel_mps2
            constraint = {
                "STATIC_STOP": "BRAKE_TRIGGER_PRE_DECEL",
                "TRAFFIC_LIGHT": "SIGNAL_BRAKE_TRIGGER_PRE_DECEL",
            }.get(target.source_behavior)
            if constraint is not None and constraint in target.constraints:
                return self.stop_pre_decel_mps2
        return self.max_decel_mps2

    def ramp_speed(
        self,
        current: float,
        target: float,
        dt: float,
        accel_limit: float | None = None,
        decel_limit: float | None = None,
    ) -> float:
        if target >= current:
            limit = self.max_accel_mps2 if accel_limit is None else accel_limit
            if limit <= 0.0:
                return target
            return min(target, current + limit * dt)
        limit = self.max_decel_mps2 if decel_limit is None else decel_limit
        return max(target, current - limit * dt)

    def planned_stop_steering_scale(
        self,
        planned_stop: bool,
        stop_target_distance: float,
    ) -> float:
        align_distance = getattr(
            self,
            "planned_stop_steering_align_distance_m",
            0.0,
        )
        # Distance 0 means "brake now", not "at the line": it never straightens the wheels.
        if (
            not planned_stop
            or align_distance <= 0.0
            or not math.isfinite(stop_target_distance)
            or stop_target_distance <= 0.0
            or stop_target_distance >= INF_DISTANCE
        ):
            return 1.0
        return clamp(stop_target_distance / align_distance, 0.0, 1.0)

    def compute_limited_steering(self, dt: float, target_scale: float = 1.0) -> float:
        self.raw_steering_rad = self.compute_lateral_steering() * clamp(
            target_scale, 0.0, 1.0
        )
        raw = self.apply_steering_compensation(self.raw_steering_rad, dt)
        delta = raw - self.commanded_steering_rad
        if self.steering_rate_limit_radps > 0.0:
            max_delta = self.steering_rate_limit_radps * dt
            delta = clamp(delta, -max_delta, max_delta)
        return clamp(
            self.commanded_steering_rad + delta,
            -self.max_steering_rad,
            self.max_steering_rad,
        )

    def apply_steering_compensation(self, steering_rad: float, dt: float) -> float:
        vehicle = self.latest_vehicle_state
        encoder = float(vehicle.steering_rad) if vehicle is not None else None
        speed = (
            abs(float(vehicle.speed_mps)) if vehicle is not None
            else abs(float(self.commanded_speed_mps))
        )
        return self.steering_compensator.apply(steering_rad, encoder, speed, dt)

    def steering_compensation_diagnostics(self) -> list[KeyValue]:
        compensator = self.steering_compensator
        return [
            KeyValue(key="straight_integral_deg",
                     value=f"{math.degrees(getattr(self, 'straight_integral_rad', 0.0)):.3f}"),
            KeyValue(key="path_latency_s", value=f"{getattr(self, 'path_latency_s', 0.0):.3f}"),
            KeyValue(key="steering_compensation_deg", value=f"{math.degrees(compensator.added_rad):.3f}"),
            KeyValue(key="steering_friction_push_deg",
                     value=f"{math.degrees(compensator.friction_value_rad):.3f}"),
        ]

    def update_steering_tracking_diagnostic(self) -> None:
        if self.latest_vehicle_state is None:
            self.steering_tracking_error_rad = 0.0
            return
        actual = float(self.latest_vehicle_state.steering_rad)
        if not math.isfinite(actual):
            self.steering_tracking_error_rad = 0.0
            return
        self.steering_tracking_error_rad = abs(self.commanded_steering_rad - actual)

    def compute_lateral_steering(self) -> float:
        received = self.latest_target_path
        shifted = self.latency_compensated_path(received)
        if shifted is not None:
            # The trackers read latest_target_path; they see the shifted copy for this tick only.
            self.latest_target_path = shifted
        try:
            steering = self.compute_pp_stanley_steering()
        finally:
            self.latest_target_path = received
        return steering + self.update_straight_integral()

    def latency_compensated_path(self, path) -> TrackingPath | None:
        """The local path was cut from the pose at its stamp; move it by the travel since
        (vehicle speed, yaw rate from the encoder about its straight-running angle)."""
        self.path_latency_s = 0.0
        vehicle = self.latest_vehicle_state
        # Only global-route paths carry the route-context stamp of the pose they were cut from;
        # avoidance and held paths are stamped by the scene and would jump between sources.
        if path is None or vehicle is None or not path.points or path.source not in GLOBAL_ROUTE_SOURCES:
            return None
        stamp = path.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        age = (self.get_clock().now().nanoseconds
               - (stamp.sec * 1_000_000_000 + stamp.nanosec)) * 1.0e-9
        age = clamp(age, 0.0, self.path_latency_max_s)
        speed = float(vehicle.speed_mps)
        if age <= 0.0 or not math.isfinite(speed):
            return None
        yaw_rate = speed * math.tan(
            float(vehicle.steering_rad) - self.straight_encoder_rad
        ) / self.wheelbase_m
        self.path_latency_s = age
        return TrackingPath(path.header, latency_shifted_points(path.points, age, speed, yaw_rate),
                            path.source)

    def straight_gains(self, speed_mps: float) -> tuple[float, float]:
        """Straight gains: the scheduled ones from straight_gain_full_speed_mps up; below it the
        values of that speed blend back to the fixed straight gains by
        straight_gain_blend_start_mps, so stopping, restarting and slow avoidance keep them."""
        speed = abs(float(speed_mps))
        full = self.straight_gain_full_speed_mps
        heading, cross_track = scheduled_stanley_gains(
            max(speed, full), self.straight_natural_frequency_radps, self.straight_damping_ratio,
            self.wheelbase_m, self.stanley_softening_speed_mps, full,
        )
        start = self.straight_gain_blend_start_mps
        share = clamp((speed - start) / (full - start), 0.0, 1.0)
        return (
            self.stanley_heading_gain + share * (heading - self.stanley_heading_gain),
            self.stanley_cross_track_gain + share * (cross_track - self.stanley_cross_track_gain),
        )

    def update_straight_integral(self) -> float:
        """Slow integral of the front-axle CTE on global-route straights.

        The car needs a small steady right command to run straight (encoder about −0.3~−0.5°,
        differing by day); without it Stanley holds that command with a lateral offset that
        grows as (v+3)/ke. Held elsewhere (curves, avoidance, low speed, errors above
        straight_integral_max_error_m), cleared in manual.
        """
        vehicle = self.latest_vehicle_state
        path = self.latest_target_path
        decision = self.stanley_decision
        speed = abs(float(vehicle.speed_mps)) if vehicle is not None else 0.0
        if (
            vehicle is not None
            and vehicle.mode == VehicleState.MODE_AUTO
            and path is not None
            and path.source in GLOBAL_ROUTE_SOURCES
            and self.lookahead_phase == "straight"
            and decision.valid
            and speed >= self.straight_integral_min_speed_mps
            # A large error is a transient (start offset, rejoin) for the proportional terms.
            and abs(decision.cross_track_error_m) <= self.straight_integral_max_error_m
        ):
            _, cross_track_gain = self.straight_gains(speed)
            rate = (cross_track_gain * decision.cross_track_error_m
                    / (speed + self.stanley_softening_speed_mps) / self.straight_integral_time_s)
            self.straight_integral_rad = clamp(
                self.straight_integral_rad + rate / self.control_rate_hz,
                -self.straight_integral_limit_rad,
                self.straight_integral_limit_rad,
            )
        return self.straight_integral_rad

    def on_route_context(self, message: RouteContext) -> None:
        self.latest_route_context = message

    def select_hybrid_phase(self, geometry_phase: str) -> str:
        self.hybrid_route_phase_active = False
        self.hybrid_route_segment_index = -1
        self.hybrid_route_fallback_reason = 'no_valid_global_context'
        boundaries = getattr(self, "hybrid_route_segment_boundaries_m", ())
        phases = getattr(self, "hybrid_route_segment_phases", ())
        context = getattr(self, "latest_route_context", None)
        if (
            not phases
            or context is None
            or not context.route_projection_valid
            or self.latest_target_path is None
            or self.latest_target_path.source not in {
                "global_route_local_path", "global_route_constant_speed"
            }
        ):
            return geometry_phase
        stamp = context.header.stamp
        age = (self.get_clock().now().nanoseconds
               - (stamp.sec * 1_000_000_000 + stamp.nanosec)) * 1.0e-9
        route_length = float(context.progress_s) + float(context.distance_to_finish)
        # An old progress sample or a different map must not select a fixed corner.
        if (
            not 0.0 <= age <= self.target_stale_sec
            or not math.isfinite(route_length)
            or abs(route_length - self.hybrid_route_length_m) > 0.5
        ):
            self.hybrid_route_fallback_reason = 'stale_context_or_length_mismatch'
            return geometry_phase
        index = route_segment_index(float(context.progress_s), boundaries)
        phase = phases[index] if index >= 0 else None
        self.hybrid_route_segment_index = index
        if phase is None:
            self.hybrid_route_fallback_reason = 'outside_fixed_segments'
            return geometry_phase
        self.hybrid_route_phase_active = True
        self.hybrid_route_fallback_reason = ''
        return phase

    def path_requires_stanley_only(self) -> bool:
        path = getattr(self, "latest_target_path", None)
        prefixes = getattr(self, "stanley_only_path_source_prefixes", ())
        return path is not None and any(
            str(path.source).startswith(prefix) for prefix in prefixes
        )

    def compute_pp_stanley_steering(self) -> float:
        self.hybrid_pp_steering_rad = self.compute_pure_pursuit_steering()
        hybrid_decision = self.lookahead_decision
        hybrid_phase = self.lookahead_phase
        hybrid_curve_active = self.lookahead_curve_active
        self.hybrid_stanley_steering_rad = self.compute_stanley_steering()
        self.lookahead_decision = hybrid_decision
        self.lookahead_phase = hybrid_phase
        self.lookahead_curve_active = hybrid_curve_active
        self.hybrid_phase = self.select_hybrid_phase(hybrid_phase)
        self.hybrid_target_pp_weight = hybrid_pp_weight_for_phase(
            self.hybrid_phase,
            self.hybrid_pp_straight_weight,
            self.hybrid_pp_approach_weight,
            self.hybrid_pp_turning_weight,
            self.hybrid_pp_exit_weight,
        )
        self.stanley_only_path_active = self.path_requires_stanley_only()
        if self.stanley_only_path_active:
            self.hybrid_target_pp_weight = 0.0
        max_delta = self.hybrid_blend_rate_per_s / self.control_rate_hz
        self.hybrid_pp_weight += clamp(
            self.hybrid_target_pp_weight - self.hybrid_pp_weight,
            -max_delta,
            max_delta,
        )
        return blend_lateral_steering(
            self.hybrid_pp_steering_rad,
            self.hybrid_stanley_steering_rad,
            self.hybrid_pp_weight,
        )

    def compute_stanley_steering(self) -> float:
        if self.latest_target_path is None or not self.latest_target_path.points:
            return 0.0
        speed = (
            self.latest_vehicle_state.speed_mps
            if self.latest_vehicle_state is not None
            else self.commanded_speed_mps
        )
        path_decision = adaptive_lookahead_decision(
            self.latest_target_path.points,
            float(speed),
            self.lookahead_base_m,
            self.lookahead_gain_s,
            self.lookahead_min_m,
            self.lookahead_max_m,
            self.lookahead_curve_start_1pm,
            self.lookahead_curve_full_1pm,
            self.lookahead_near_window_m,
            self.lookahead_preview_distance_m,
            self.lookahead_curve_exit_1pm,
            self.lookahead_curve_speed_gain_s,
            self.lookahead_curve_minimum_samples,
            self.lookahead_curve_active,
        )
        self.lookahead_curve_active = path_decision.phase in {
            "turn_approach",
            "turning",
            "chained_turn",
        }
        self.lookahead_phase = path_decision.phase
        self.lookahead_decision = path_decision
        straight_heading_gain = self.stanley_heading_gain
        straight_cross_track_gain = self.stanley_cross_track_gain
        if self.latest_target_path.source in GLOBAL_ROUTE_SOURCES:
            # Curve values stay; the curve severity blends toward them as before. Avoidance
            # and held paths keep the fixed gains.
            straight_heading_gain, straight_cross_track_gain = self.straight_gains(float(speed))
        self.stanley_profile = adaptive_stanley_profile(
            path_decision.curve_severity,
            straight_heading_gain,
            self.stanley_curve_heading_gain,
            straight_cross_track_gain,
            self.stanley_curve_cross_track_gain,
            self.stanley_cross_track_correction_limit_rad,
            self.stanley_curve_cross_track_correction_limit_rad,
            self.stanley_heading_correction_limit_rad,
            self.stanley_curve_heading_correction_limit_rad,
            self.stanley_heading_window_m,
            self.stanley_curve_heading_window_m,
        )
        self.stanley_preview_distance_m = stanley_preview_distance(
            float(speed),
            path_decision.curve_severity,
            self.stanley_curve_preview_time_s,
            self.stanley_curve_preview_max_m,
        )
        self.stanley_decision = stanley_steering_decision(
            self.latest_target_path.points,
            float(speed),
            self.wheelbase_m,
            self.stanley_profile.heading_gain,
            self.stanley_profile.cross_track_gain,
            self.stanley_softening_speed_mps,
            self.stanley_profile.cross_track_correction_limit_rad,
            self.stanley_profile.heading_window_m,
            self.stanley_profile.heading_correction_limit_rad,
            self.stanley_preview_distance_m,
        )
        return self.stanley_decision.steering_rad

    def compute_pure_pursuit_steering(self) -> float:
        if self.latest_target_path is None or not self.latest_target_path.points:
            return 0.0
        speed = (
            self.latest_vehicle_state.speed_mps
            if self.latest_vehicle_state is not None
            else self.commanded_speed_mps
        )
        decision = adaptive_lookahead_decision(
            self.latest_target_path.points,
            float(speed),
            self.lookahead_base_m,
            self.lookahead_gain_s,
            self.lookahead_min_m,
            self.lookahead_max_m,
            self.lookahead_curve_start_1pm,
            self.lookahead_curve_full_1pm,
            self.lookahead_near_window_m,
            self.lookahead_preview_distance_m,
            self.lookahead_curve_exit_1pm,
            self.lookahead_curve_speed_gain_s,
            self.lookahead_curve_minimum_samples,
            self.lookahead_curve_active,
        )
        lookahead = slew_lookahead(
            self.current_lookahead_m,
            decision.distance_m,
            1.0 / self.control_rate_hz,
            self.lookahead_shorten_rate_mps,
            self.lookahead_lengthen_rate_mps,
        )
        phase = decision.phase
        self.lookahead_curve_active = phase in {
            "turn_approach",
            "turning",
            "chained_turn",
        }
        if (
            phase == "straight"
            and self.lookahead_phase in {"turning", "chained_turn", "turn_exit"}
            and lookahead < decision.distance_m - 0.05
        ):
            phase = "turn_exit"
        self.current_lookahead_m = lookahead
        self.lookahead_phase = phase
        self.lookahead_decision = LookaheadDecision(
            lookahead,
            phase,
            decision.near_curvature_1pm,
            decision.preview_curvature_1pm,
            decision.curve_distance_m,
            decision.curve_severity,
        )
        target = interpolated_lookahead_target(
            self.latest_target_path.points,
            lookahead,
        )
        if target is None:
            return 0.0

        ld2 = max(0.25, target[0] * target[0] + target[1] * target[1])
        curvature = 2.0 * target[1] / ld2
        return math.atan(self.wheelbase_m * curvature)

    def publish_lookahead_diagnostic(self) -> None:
        decision = self.lookahead_decision
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "kaiev26_motion_control/adaptive_lookahead"
        status.hardware_id = "motion_control"
        status.message = (
            decision.phase
        )
        status.values = [
            KeyValue(key="phase", value=decision.phase),
            KeyValue(key="lookahead_m", value=f"{decision.distance_m:.3f}"),
            KeyValue(
                key="near_curvature_1pm",
                value=f"{decision.near_curvature_1pm:.5f}",
            ),
            KeyValue(
                key="preview_curvature_1pm",
                value=f"{decision.preview_curvature_1pm:.5f}",
            ),
            KeyValue(
                key="curve_distance_m",
                value=(
                    f"{decision.curve_distance_m:.3f}"
                    if math.isfinite(decision.curve_distance_m)
                    else "inf"
                ),
            ),
            KeyValue(
                key="curve_severity",
                value=f"{decision.curve_severity:.3f}",
            ),
            KeyValue(
                key="raw_steering_deg",
                value=f"{math.degrees(self.raw_steering_rad):.3f}",
            ),
            *self.steering_compensation_diagnostics(),
            KeyValue(
                key="actual_steering_rate_degps",
                value=f"{math.degrees(self.actual_steering_rate_radps):.3f}",
            ),
            KeyValue(
                key="steering_tracking_error_deg",
                value=f"{math.degrees(self.steering_tracking_error_rad):.3f}",
            ),
            KeyValue(
                key="brake_release_pending",
                value=str(self.brake_release_pending).lower(),
            ),
            KeyValue(
                key="brake_release_elapsed_sec",
                value=f"{self.brake_release_elapsed_sec:.3f}",
            ),
            KeyValue(
                key="standstill_brake_released",
                value=str(getattr(self, "standstill_brake_released", False)).lower(),
            ),
            KeyValue(
                key="stanley_valid",
                value=str(self.stanley_decision.valid).lower(),
            ),
            KeyValue(
                key="stanley_heading_error_deg",
                value=f"{math.degrees(self.stanley_decision.heading_error_rad):.3f}",
            ),
            KeyValue(
                key="stanley_cross_track_error_m",
                value=f"{self.stanley_decision.cross_track_error_m:.3f}",
            ),
            KeyValue(
                key="stanley_cross_track_correction_deg",
                value=(
                    f"{math.degrees(self.stanley_decision.cross_track_correction_rad):.3f}"
                ),
            ),
            KeyValue(
                key="stanley_heading_correction_deg",
                value=(
                    f"{math.degrees(self.stanley_decision.heading_correction_rad):.3f}"
                ),
            ),
            KeyValue(
                key="stanley_heading_gain",
                value=f"{self.stanley_profile.heading_gain:.3f}",
            ),
            KeyValue(
                key="stanley_cross_track_gain",
                value=f"{self.stanley_profile.cross_track_gain:.3f}",
            ),
            KeyValue(
                key="stanley_heading_limit_deg",
                value=(
                    f"{math.degrees(self.stanley_profile.heading_correction_limit_rad):.3f}"
                ),
            ),
            KeyValue(
                key="stanley_cross_track_limit_deg",
                value=(
                    f"{math.degrees(self.stanley_profile.cross_track_correction_limit_rad):.3f}"
                ),
            ),
            KeyValue(
                key="stanley_heading_window_m",
                value=f"{self.stanley_profile.heading_window_m:.3f}",
            ),
            KeyValue(
                key="stanley_preview_distance_m",
                value=f"{self.stanley_preview_distance_m:.3f}",
            ),
            KeyValue(
                key="hybrid_phase",
                value=self.hybrid_phase,
            ),
            KeyValue(
                key="hybrid_route_phase_active",
                value=str(self.hybrid_route_phase_active).lower(),
            ),
            KeyValue(key="hybrid_route_segment_index", value=str(getattr(self, 'hybrid_route_segment_index', -1))),
            KeyValue(key="hybrid_route_fallback_reason", value=getattr(self, 'hybrid_route_fallback_reason', '')),
            KeyValue(
                key="hybrid_pp_weight",
                value=f"{self.hybrid_pp_weight:.3f}",
            ),
            KeyValue(
                key="hybrid_target_pp_weight",
                value=f"{self.hybrid_target_pp_weight:.3f}",
            ),
            KeyValue(
                key="stanley_only_path",
                value=str(getattr(self, "stanley_only_path_active", False)).lower(),
            ),
            KeyValue(
                key="hybrid_pp_steering_deg",
                value=f"{math.degrees(self.hybrid_pp_steering_rad):.3f}",
            ),
            KeyValue(
                key="hybrid_stanley_steering_deg",
                value=f"{math.degrees(self.hybrid_stanley_steering_rad):.3f}",
            ),
        ]
        message.status = [status]
        self.diagnostic_pub.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
