#!/usr/bin/env python3
"""판단 결과를 최종 차량 액추에이터 명령으로 바꾸는 노드."""
from __future__ import annotations

from dataclasses import dataclass
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import ActuatorCommand, Centerline, TargetSpeed, VehicleState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RosTime


INF_DISTANCE = 1.0e6
# Match lateral_ctrl's minimum drive speed for steering actuation.
MIN_STEERING_SPEED_MPS = 1.0 / 3.6
PURE_PURSUIT = "pure_pursuit"
STANLEY = "stanley"
PP_STANLEY = "pp_stanley"
FF_STANLEY = "ff_stanley"


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


@dataclass(frozen=True)
class StraightCorridorDecision:
    scale: float
    cross_track_error_m: float
    heading_error_rad: float
    predicted_error_m: float
    eligible: bool = False


def validate_lateral_controller(value: str) -> str:
    """Normalize and validate the selected lateral controller."""
    controller = value.strip().lower()
    if controller in {PURE_PURSUIT, STANLEY, PP_STANLEY, FF_STANLEY}:
        return controller
    choices = ", ".join((PURE_PURSUIT, STANLEY, PP_STANLEY, FF_STANLEY))
    raise ValueError(f"unknown lateral_controller '{value}'; expected one of: {choices}")


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def smoothstep_scale(value: float, lower: float, upper: float) -> float:
    """Return a continuous 0..1 correction scale between two thresholds."""
    if upper <= lower:
        return 1.0 if value >= upper else 0.0
    ratio = clamp((value - lower) / (upper - lower), 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


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


def straight_corridor_decision(
    points,
    path_source: str,
    phase: str,
    speed_mps: float,
    preview_time_s: float,
    preview_min_m: float,
    preview_max_m: float,
    error_deadband_m: float,
    error_full_m: float,
    heading_deadband_rad: float,
    heading_full_rad: float,
    heading_window_m: float,
) -> StraightCorridorDecision:
    """Scale small straight-route corrections using projected lateral error."""
    global_route_sources = {
        "global_route_local_path",
        "global_route_constant_speed",
    }
    if path_source not in global_route_sources or phase != "straight":
        return StraightCorridorDecision(1.0, 0.0, 0.0, 0.0, False)
    reference = stanley_path_reference(points, 0.0, heading_window_m)
    if reference is None:
        return StraightCorridorDecision(1.0, 0.0, 0.0, 0.0, False)
    heading_error, cross_track_error, _, _ = reference
    preview_distance = clamp(
        abs(float(speed_mps)) * max(0.0, float(preview_time_s)),
        max(0.0, float(preview_min_m)),
        max(float(preview_min_m), float(preview_max_m)),
    )
    predicted_error = cross_track_error + preview_distance * math.sin(heading_error)
    error_scale = smoothstep_scale(
        abs(predicted_error),
        max(0.0, float(error_deadband_m)),
        max(float(error_deadband_m), float(error_full_m)),
    )
    heading_scale = smoothstep_scale(
        abs(heading_error),
        max(0.0, float(heading_deadband_rad)),
        max(float(heading_deadband_rad), float(heading_full_rad)),
    )
    return StraightCorridorDecision(
        max(error_scale, heading_scale),
        cross_track_error,
        heading_error,
        predicted_error,
        True,
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


def reference_curvature_1pm(
    points,
    preview_distance_m: float,
    sample_half_width_m: float,
) -> float:
    """Return the smoothed signed path curvature at the preview distance."""
    half_width = max(0.5, float(sample_half_width_m))
    preview = max(half_width, float(preview_distance_m))
    profile = median_curvature_profile(
        path_curvature_profile(
            points,
            preview_distance_m=preview + half_width,
            sample_half_width_m=half_width,
        ),
        window_size=3,
    )
    if not profile:
        return 0.0
    _, curvature = min(profile, key=lambda item: abs(item[0] - preview))
    return float(curvature) if math.isfinite(curvature) else 0.0


def curvature_feedforward_steering(
    curvature_1pm: float,
    wheelbase_m: float,
    gain: float,
    curvature_limit_1pm: float,
    steering_limit_rad: float,
) -> float:
    """Convert path curvature to bicycle-model steering with explicit limits."""
    curvature_limit = max(0.0, float(curvature_limit_1pm))
    curvature = clamp(float(curvature_1pm), -curvature_limit, curvature_limit)
    steering = max(0.0, float(gain)) * math.atan(float(wheelbase_m) * curvature)
    steering_limit = max(0.0, float(steering_limit_rad))
    return clamp(steering, -steering_limit, steering_limit)


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


def curvature_speed_limit_mps(
    cruise_speed_mps: float,
    curve_severity: float,
    minimum_curve_speed_mps: float,
) -> float:
    """Blend the cruise limit toward the configured speed as a curve approaches."""
    cruise = max(0.0, float(cruise_speed_mps))
    floor = clamp(float(minimum_curve_speed_mps), 0.0, cruise)
    severity = clamp(float(curve_severity), 0.0, 1.0)
    return cruise + (floor - cruise) * severity


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
        self.declare_parameter("wheelbase_m", 1.2991017929)
        self.declare_parameter("max_steering_rad", 0.4363)
        self.declare_parameter("steering_rate_limit_radps", 1.0471975512)
        self.declare_parameter("steering_rate_filter_time_constant_s", 0.15)
        self.declare_parameter("steering_rate_estimate_limit_radps", 2.0943951024)
        self.declare_parameter("straight_corridor_enabled", True)
        self.declare_parameter("straight_corridor_preview_time_s", 1.2)
        self.declare_parameter("straight_corridor_preview_min_m", 4.0)
        self.declare_parameter("straight_corridor_preview_max_m", 9.0)
        self.declare_parameter("straight_corridor_deadband_m", 0.08)
        self.declare_parameter("straight_corridor_full_m", 0.25)
        self.declare_parameter("straight_heading_deadband_rad", 0.0139626340)
        self.declare_parameter("straight_heading_full_rad", 0.0349065850)
        self.declare_parameter("lateral_controller", PURE_PURSUIT)
        self.declare_parameter("adaptive_lookahead_enabled", True)
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
        self.declare_parameter("ff_stanley_feedback_scale", 0.50)
        self.declare_parameter("ff_curvature_gain", 1.0)
        self.declare_parameter("ff_preview_time_s", 0.20)
        self.declare_parameter("ff_preview_min_m", 2.0)
        self.declare_parameter("ff_preview_max_m", 4.0)
        self.declare_parameter("ff_curvature_sample_half_width_m", 2.0)
        self.declare_parameter("ff_curvature_limit_1pm", 0.20)
        self.declare_parameter("ff_steering_limit_rad", 0.30)
        self.declare_parameter("hybrid_pp_straight_weight", 0.0)
        self.declare_parameter("hybrid_pp_approach_weight", 0.65)
        self.declare_parameter("hybrid_pp_turning_weight", 0.35)
        self.declare_parameter("hybrid_pp_exit_weight", 0.10)
        self.declare_parameter("hybrid_blend_rate_per_s", 6.0)
        self.declare_parameter("curvature_speed_planning_enabled", True)
        self.declare_parameter("curvature_speed_min_mps", 3.2)
        self.declare_parameter("max_accel_mps2", 1.0)
        self.declare_parameter("signal_release_accel_mps2", 2.0)
        self.declare_parameter("signal_release_boost_speed_mps", 3.8)
        self.declare_parameter("max_decel_mps2", 2.0)
        self.declare_parameter("stop_speed_epsilon_mps", 0.05)
        self.declare_parameter("planned_stop_brake_distance_m", 2.0)
        self.declare_parameter("stopped_steering_hold_speed_mps", 0.10)
        self.declare_parameter("planned_stop_steering_align_distance_m", 2.0)
        self.declare_parameter("stop_release_steering_resume_speed_mps", 0.30)

        self.target_stale_sec = float(self.get_parameter("target_stale_sec").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.max_steering_rad = float(self.get_parameter("max_steering_rad").value)
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
        self.straight_corridor_enabled = bool(
            self.get_parameter("straight_corridor_enabled").value
        )
        self.straight_corridor_preview_time_s = max(
            0.0, float(self.get_parameter("straight_corridor_preview_time_s").value)
        )
        self.straight_corridor_preview_min_m = max(
            0.0, float(self.get_parameter("straight_corridor_preview_min_m").value)
        )
        self.straight_corridor_preview_max_m = max(
            self.straight_corridor_preview_min_m,
            float(self.get_parameter("straight_corridor_preview_max_m").value),
        )
        self.straight_corridor_deadband_m = max(
            0.0, float(self.get_parameter("straight_corridor_deadband_m").value)
        )
        self.straight_corridor_full_m = max(
            self.straight_corridor_deadband_m,
            float(self.get_parameter("straight_corridor_full_m").value),
        )
        self.straight_heading_deadband_rad = max(
            0.0, float(self.get_parameter("straight_heading_deadband_rad").value)
        )
        self.straight_heading_full_rad = max(
            self.straight_heading_deadband_rad,
            float(self.get_parameter("straight_heading_full_rad").value),
        )
        self.lateral_controller = validate_lateral_controller(
            str(self.get_parameter("lateral_controller").value)
        )
        self.adaptive_lookahead_enabled = bool(
            self.get_parameter("adaptive_lookahead_enabled").value
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
        self.ff_stanley_feedback_scale = clamp(
            float(self.get_parameter("ff_stanley_feedback_scale").value),
            0.0,
            1.0,
        )
        self.ff_curvature_gain = max(
            0.0, float(self.get_parameter("ff_curvature_gain").value)
        )
        self.ff_preview_time_s = max(
            0.0, float(self.get_parameter("ff_preview_time_s").value)
        )
        self.ff_preview_min_m = max(
            0.0, float(self.get_parameter("ff_preview_min_m").value)
        )
        self.ff_preview_max_m = max(
            self.ff_preview_min_m,
            float(self.get_parameter("ff_preview_max_m").value),
        )
        self.ff_curvature_sample_half_width_m = max(
            0.5,
            float(
                self.get_parameter("ff_curvature_sample_half_width_m").value
            ),
        )
        self.ff_curvature_limit_1pm = max(
            0.0, float(self.get_parameter("ff_curvature_limit_1pm").value)
        )
        self.ff_steering_limit_rad = clamp(
            float(self.get_parameter("ff_steering_limit_rad").value),
            0.0,
            self.max_steering_rad,
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
        self.curvature_speed_planning_enabled = bool(
            self.get_parameter("curvature_speed_planning_enabled").value
        )
        self.curvature_speed_min_mps = max(
            0.0, float(self.get_parameter("curvature_speed_min_mps").value)
        )
        self.max_accel_mps2 = float(self.get_parameter("max_accel_mps2").value)
        self.signal_release_accel_mps2 = float(
            self.get_parameter("signal_release_accel_mps2").value
        )
        self.signal_release_boost_speed_mps = float(
            self.get_parameter("signal_release_boost_speed_mps").value
        )
        self.max_decel_mps2 = float(self.get_parameter("max_decel_mps2").value)
        self.stop_speed_epsilon_mps = float(
            self.get_parameter("stop_speed_epsilon_mps").value
        )
        self.planned_stop_brake_distance_m = max(
            0.0,
            float(self.get_parameter("planned_stop_brake_distance_m").value),
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

        self.latest_target_path: Centerline | None = None
        self.latest_target_speed: TargetSpeed | None = None
        self.latest_vehicle_state: VehicleState | None = None

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
        self.ff_reference_curvature_1pm = 0.0
        self.ff_preview_distance_m = self.ff_preview_min_m
        self.ff_steering_rad = 0.0
        self.ff_stanley_feedback_rad = 0.0
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
        self.curvature_speed_limit_mps = math.inf
        self.steering_tracking_error_rad = 0.0
        self.actual_steering_rate_radps = 0.0
        self.actual_steering_rate_sample_rad: float | None = None
        self.actual_steering_rate_sample_ns: int | None = None
        self.raw_steering_rad = 0.0
        self.corridor_scaled_steering_rad = 0.0
        self.straight_corridor = StraightCorridorDecision(
            1.0, 0.0, 0.0, 0.0, False
        )
        self.signal_release_boost_active = False
        self.planned_stop_steering_hold_active = False
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
        self.create_timer(1.0 / self.control_rate_hz, self.on_control_timer)
        self.create_timer(0.2, self.publish_lookahead_diagnostic)
        self.get_logger().info(
            f"lateral controller: {self.lateral_controller}"
        )

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
            self.apply_manual_standby(command)
        elif self.targets_are_stale(now):
            self.apply_stop_motion(command)
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
        self.current_lookahead_m = None
        self.lookahead_phase = "uninitialized"
        self.lookahead_curve_active = False
        self.stanley_preview_distance_m = 0.0
        self.hybrid_pp_weight = getattr(self, "hybrid_pp_straight_weight", 0.0)
        self.hybrid_target_pp_weight = self.hybrid_pp_weight
        self.hybrid_phase = "straight"
        self.hybrid_pp_steering_rad = 0.0
        self.hybrid_stanley_steering_rad = 0.0
        self.steering_tracking_error_rad = 0.0
        self.raw_steering_rad = self.commanded_steering_rad
        self.corridor_scaled_steering_rad = self.commanded_steering_rad
        self.straight_corridor = StraightCorridorDecision(
            1.0, 0.0, 0.0, 0.0, False
        )

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

    def apply_stop_motion(self, command: ActuatorCommand) -> None:
        self.commanded_speed_mps = 0.0
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

        self.curvature_speed_limit_mps = self.compute_curvature_speed_limit(target_speed)
        target_speed = min(target_speed, self.curvature_speed_limit_mps)
        # Mission planners already shape speed; route-only tracking supplies its finish distance.
        if planned_stop and self.latest_target_speed.source_behavior == "GLOBAL_ROUTE":
            if not math.isfinite(stop_target_distance):
                self.apply_stop_motion(command)
                return
            if stop_target_distance <= self.planned_stop_brake_distance_m:
                target_speed = 0.0
            elif stop_target_distance < INF_DISTANCE:
                target_speed = min(
                    target_speed,
                    math.sqrt(max(0.0, 2.0 * self.max_decel_mps2 * stop_target_distance)),
                )

        actual_speed = (
            abs(float(self.latest_vehicle_state.speed_mps))
            if self.latest_vehicle_state is not None
            else abs(float(self.commanded_speed_mps))
        )
        stopped_for_planned_hold = (
            planned_stop
            and target_speed <= self.stop_speed_epsilon_mps
            and actual_speed <= getattr(self, "stopped_steering_hold_speed_mps", 0.10)
        )
        if self.latest_vehicle_state is not None and actual_speed < MIN_STEERING_SPEED_MPS:
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
            self.commanded_steering_rad = self.compute_limited_steering(
                dt,
                self.planned_stop_steering_scale(
                    planned_stop,
                    stop_target_distance,
                ),
            )
        self.update_steering_tracking_diagnostic()
        accel_limit = self.select_accel_limit(target_speed)
        self.commanded_speed_mps = self.ramp_speed(
            self.commanded_speed_mps, target_speed, dt, accel_limit
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

        command.steering_target_rad = float(self.commanded_steering_rad)
        command.speed_target_mps = 0.0 if hold_brake else float(self.commanded_speed_mps)
        command.brake_engage = hold_brake

    def select_accel_limit(self, target_speed: float) -> float:
        target = self.latest_target_speed
        if target is None:
            self.signal_release_boost_active = False
            return self.max_accel_mps2

        constraints = set(target.constraints)
        if "SIGNAL_PASS_COMMITTED" in constraints and not target.need_stop:
            self.signal_release_boost_active = True
        if target.need_stop or target.source_behavior in {
            "EMERGENCY",
            "RECOVERY",
            "OBSTACLE",
            "FINISH",
        }:
            self.signal_release_boost_active = False

        boost_speed = getattr(self, "signal_release_boost_speed_mps", 3.8)
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

    def compute_curvature_speed_limit(self, cruise_speed_mps: float) -> float:
        if (
            not getattr(self, "curvature_speed_planning_enabled", False)
            or self.latest_target_path is None
            or len(self.latest_target_path.points) < 2
        ):
            return max(0.0, cruise_speed_mps)

        speed = (
            abs(float(self.latest_vehicle_state.speed_mps))
            if self.latest_vehicle_state is not None
            else abs(float(self.commanded_speed_mps))
        )
        decision = adaptive_lookahead_decision(
            self.latest_target_path.points,
            speed,
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
            getattr(self, "lookahead_curve_active", False),
        )
        return curvature_speed_limit_mps(
            cruise_speed_mps,
            decision.curve_severity,
            self.curvature_speed_min_mps,
        )

    def ramp_speed(
        self,
        current: float,
        target: float,
        dt: float,
        accel_limit: float | None = None,
    ) -> float:
        if target >= current:
            limit = self.max_accel_mps2 if accel_limit is None else accel_limit
            return min(target, current + limit * dt)
        return max(target, current - self.max_decel_mps2 * dt)

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
        if (
            not planned_stop
            or align_distance <= 0.0
            or not math.isfinite(stop_target_distance)
            or stop_target_distance >= INF_DISTANCE
        ):
            return 1.0
        return clamp(stop_target_distance / align_distance, 0.0, 1.0)

    def compute_limited_steering(self, dt: float, target_scale: float = 1.0) -> float:
        self.raw_steering_rad = self.compute_lateral_steering() * clamp(
            target_scale, 0.0, 1.0
        )
        raw = self.apply_straight_corridor(self.raw_steering_rad)
        self.corridor_scaled_steering_rad = raw
        max_delta = self.steering_rate_limit_radps * dt
        delta = clamp(raw - self.commanded_steering_rad, -max_delta, max_delta)
        return clamp(
            self.commanded_steering_rad + delta,
            -self.max_steering_rad,
            self.max_steering_rad,
        )

    def apply_straight_corridor(self, steering_rad: float) -> float:
        if (
            not getattr(self, "straight_corridor_enabled", False)
            or self.latest_target_path is None
            or self.latest_vehicle_state is None
        ):
            self.straight_corridor = StraightCorridorDecision(
                1.0, 0.0, 0.0, 0.0, False
            )
            return steering_rad
        self.straight_corridor = straight_corridor_decision(
            self.latest_target_path.points,
            self.latest_target_path.source,
            self.lookahead_phase,
            float(self.latest_vehicle_state.speed_mps),
            self.straight_corridor_preview_time_s,
            self.straight_corridor_preview_min_m,
            self.straight_corridor_preview_max_m,
            self.straight_corridor_deadband_m,
            self.straight_corridor_full_m,
            self.straight_heading_deadband_rad,
            self.straight_heading_full_rad,
            self.stanley_heading_window_m,
        )
        return steering_rad * self.straight_corridor.scale

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
        controller = getattr(self, "lateral_controller", PURE_PURSUIT)
        if controller == PURE_PURSUIT:
            return self.compute_pure_pursuit_steering()
        if controller == STANLEY:
            return self.compute_stanley_steering()
        if controller == PP_STANLEY:
            return self.compute_pp_stanley_steering()
        if controller == FF_STANLEY:
            return self.compute_ff_stanley_steering()
        raise RuntimeError(f"lateral controller '{controller}' is not implemented")

    def compute_ff_stanley_steering(self) -> float:
        """Use route curvature as feedforward and Stanley for error feedback."""
        if self.latest_target_path is None or not self.latest_target_path.points:
            self.ff_reference_curvature_1pm = 0.0
            self.ff_steering_rad = 0.0
            self.ff_stanley_feedback_rad = 0.0
            return 0.0

        speed_mps = (
            abs(float(self.latest_vehicle_state.speed_mps))
            if self.latest_vehicle_state is not None
            else abs(float(self.commanded_speed_mps))
        )
        self.ff_preview_distance_m = clamp(
            speed_mps * self.ff_preview_time_s,
            self.ff_preview_min_m,
            self.ff_preview_max_m,
        )
        self.ff_reference_curvature_1pm = reference_curvature_1pm(
            self.latest_target_path.points,
            self.ff_preview_distance_m,
            self.ff_curvature_sample_half_width_m,
        )
        self.ff_steering_rad = curvature_feedforward_steering(
            self.ff_reference_curvature_1pm,
            self.wheelbase_m,
            self.ff_curvature_gain,
            self.ff_curvature_limit_1pm,
            self.ff_steering_limit_rad,
        )
        self.ff_stanley_feedback_rad = (
            self.ff_stanley_feedback_scale * self.compute_stanley_steering()
        )
        return clamp(
            self.ff_steering_rad + self.ff_stanley_feedback_rad,
            -self.max_steering_rad,
            self.max_steering_rad,
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
        self.hybrid_phase = hybrid_phase
        self.hybrid_target_pp_weight = hybrid_pp_weight_for_phase(
            hybrid_phase,
            self.hybrid_pp_straight_weight,
            self.hybrid_pp_approach_weight,
            self.hybrid_pp_turning_weight,
            self.hybrid_pp_exit_weight,
        )
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
        self.stanley_profile = adaptive_stanley_profile(
            path_decision.curve_severity,
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
        if self.adaptive_lookahead_enabled:
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
        else:
            decision = LookaheadDecision(
                clamp(
                    self.lookahead_base_m
                    + max(0.0, float(speed)) * self.lookahead_gain_s,
                    self.lookahead_min_m,
                    self.lookahead_max_m,
                ),
                "speed_only",
                0.0,
                0.0,
                math.inf,
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
            KeyValue(key="lateral_controller", value=self.lateral_controller),
            KeyValue(
                key="ff_reference_curvature_1pm",
                value=f"{self.ff_reference_curvature_1pm:.5f}",
            ),
            KeyValue(
                key="ff_preview_distance_m",
                value=f"{self.ff_preview_distance_m:.3f}",
            ),
            KeyValue(
                key="ff_steering_deg",
                value=f"{math.degrees(self.ff_steering_rad):.3f}",
            ),
            KeyValue(
                key="ff_stanley_feedback_deg",
                value=f"{math.degrees(self.ff_stanley_feedback_rad):.3f}",
            ),
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
                key="curvature_speed_limit_mps",
                value=(
                    f"{self.curvature_speed_limit_mps:.3f}"
                    if math.isfinite(self.curvature_speed_limit_mps)
                    else "inf"
                ),
            ),
            KeyValue(
                key="raw_steering_deg",
                value=f"{math.degrees(self.raw_steering_rad):.3f}",
            ),
            KeyValue(
                key="straight_corridor_eligible",
                value=str(self.straight_corridor.eligible).lower(),
            ),
            KeyValue(
                key="straight_corridor_scale",
                value=f"{self.straight_corridor.scale:.3f}",
            ),
            KeyValue(
                key="straight_predicted_error_m",
                value=f"{self.straight_corridor.predicted_error_m:.3f}",
            ),
            KeyValue(
                key="actual_steering_rate_degps",
                value=f"{math.degrees(self.actual_steering_rate_radps):.3f}",
            ),
            KeyValue(
                key="steering_tracking_error_deg",
                value=f"{math.degrees(self.steering_tracking_error_rad):.3f}",
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
                key="hybrid_pp_weight",
                value=f"{self.hybrid_pp_weight:.3f}",
            ),
            KeyValue(
                key="hybrid_target_pp_weight",
                value=f"{self.hybrid_target_pp_weight:.3f}",
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
