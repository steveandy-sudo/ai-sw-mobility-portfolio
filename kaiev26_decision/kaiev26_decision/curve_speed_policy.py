"""Offline route-station speed limits, independent of ROS and actuator state."""
from __future__ import annotations

import bisect
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Literal

import yaml


@dataclass(frozen=True)
class CurveSpeedZone:
    zone_id: str
    start_s: float
    end_s: float
    # Tightest radius inside the zone; only read when the policy has a lateral acceleration cap.
    radius_m: float = math.inf


@dataclass(frozen=True)
class CurveSpeedLimit:
    speed_limit_mps: float
    zone_id: str
    phase: Literal["approach", "zone"]


@dataclass(frozen=True)
class CurveSpeedPolicy:
    speed_scale: float
    approach_decel_mps2: float
    zones: tuple[CurveSpeedZone, ...]
    route_length_m: float
    # Above 0 each zone is also capped at sqrt(lateral_accel * radius) (decision_pipeline.yaml).
    lateral_accel_mps2: float = 0.0

    def limit_for_progress(
        self, progress_s: float, cruise_speed_mps: float
    ) -> CurveSpeedLimit | None:
        """Limit the original cruise target, never a previously reduced target.

        The corner speed is cruise * speed_scale, and with a lateral acceleration cap no more
        than sqrt(lateral_accel * zone radius).
        Before a zone, its envelope is sqrt(v_corner**2 + 2*a*distance).
        The slowest active/upcoming envelope wins; completed zones do not apply.
        Zone endpoints are inclusive. Stop and fault handling belongs to callers.
        """
        progress = _finite_number(progress_s, "progress_s")
        cruise = _finite_number(cruise_speed_mps, "cruise_speed_mps")
        if not 0.0 <= progress <= self.route_length_m:
            raise ValueError("progress_s must be within the route length")
        if cruise < 0.0:
            raise ValueError("cruise_speed_mps must be nonnegative")

        tightest: CurveSpeedLimit | None = None
        for zone in self.zones:
            if progress > zone.end_s:
                continue
            corner_speed = cruise * self.speed_scale
            if self.lateral_accel_mps2 > 0.0 and math.isfinite(zone.radius_m):
                corner_speed = min(corner_speed, math.sqrt(self.lateral_accel_mps2 * zone.radius_m))
            if progress >= zone.start_s:
                limit = corner_speed
                phase = "zone"
            else:
                distance = zone.start_s - progress
                # hypot avoids squaring a large but finite cruise-speed value.
                braking_component = (
                    math.sqrt(2.0)
                    * math.sqrt(self.approach_decel_mps2)
                    * math.sqrt(distance)
                )
                limit = min(cruise, math.hypot(corner_speed, braking_component))
                phase = "approach"
            if limit >= cruise:
                continue
            if tightest is None or limit < tightest.speed_limit_mps:
                tightest = CurveSpeedLimit(limit, zone.zone_id, phase)
        return tightest


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{name} must be a finite number") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def validate_curve_speed_scale(value: object) -> float:
    """Validate an explicit curve-speed multiplier, without sentinel values."""
    scale = _finite_number(value, "curve_speed_scale")
    if not 0.0 < scale <= 1.0:
        raise ValueError("curve_speed_scale must be greater than zero and at most one")
    return scale


def resolve_curve_speed_scale_override(value: object) -> float | None:
    """Interpret the startup parameter: -1 retains the selected route YAML."""
    scale = _finite_number(value, "curve_speed_scale")
    return None if scale == -1.0 else validate_curve_speed_scale(scale)


def resolve_curve_lateral_accel(value: object) -> float:
    """Startup parameter: 0 turns the radius cap off; otherwise 1..10 m/s^2."""
    accel = _finite_number(value, "curve_lateral_accel_mps2")
    if accel != 0.0 and not 1.0 <= accel <= 10.0:
        raise ValueError("curve_lateral_accel_mps2 must be 0 (off) or within 1..10")
    return accel


def resolve_curve_approach_decel_override(value: object) -> float | None:
    """Startup parameter: -1 keeps the route YAML approach_decel_mps2."""
    decel = _finite_number(value, "curve_approach_decel_mps2")
    if decel == -1.0:
        return None
    if not 0.1 <= decel <= 5.0:
        raise ValueError("curve_approach_decel_mps2 must be -1 (route YAML) or within 0.1..5")
    return decel


def _require_fields(value: object, fields: set[str], name: str) -> dict:
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError(f"{name} must contain exactly {', '.join(sorted(fields))}")
    return value


def _route_points(document: dict) -> list[tuple[float, float]]:
    waypoints = document.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise ValueError("curve speed policy requires at least two route waypoints")
    points = []
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict) or not {"x", "y"}.issubset(waypoint):
            raise ValueError(f"waypoints[{index}] requires x and y")
        points.append((
            _finite_number(waypoint["x"], f"waypoints[{index}].x"),
            _finite_number(waypoint["y"], f"waypoints[{index}].y"),
        ))
    return points


def _route_length(points: list[tuple[float, float]]) -> float:
    length = sum(math.dist(start, end) for start, end in zip(points, points[1:]))
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError("curve speed policy requires a finite positive route length")
    return length


def _tightest_radius(points: list[tuple[float, float]], start_s: float, end_s: float) -> float:
    """Smallest radius in [start_s, end_s]: three points 2 m apart every 0.5 m, median of five,
    the same smoothing Motion Control uses for the path curvature."""
    stations = [0.0]
    for a, b in zip(points, points[1:]):
        stations.append(stations[-1] + math.dist(a, b))

    def at(s: float) -> tuple[float, float]:
        s = min(max(s, 0.0), stations[-1])
        index = min(len(points) - 2, max(0, bisect.bisect_right(stations, s) - 1))
        span = stations[index + 1] - stations[index]
        ratio = 0.0 if span <= 0.0 else (s - stations[index]) / span
        a, b = points[index], points[index + 1]
        return (a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1]))

    curvatures = []
    s = start_s - 1.0
    while s <= end_s + 1.0:
        a, b, c = at(s - 2.0), at(s), at(s + 2.0)
        sides = math.dist(a, b) * math.dist(b, c) * math.dist(a, c)
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        curvatures.append(abs(2.0 * cross / sides) if sides > 1.0e-9 else 0.0)
        s += 0.5
    smoothed = [sorted(curvatures[max(0, i - 2):i + 3])[len(curvatures[max(0, i - 2):i + 3]) // 2]
                for i in range(len(curvatures))]
    peak = max(smoothed, default=0.0)
    return 1.0 / peak if peak > 1.0e-6 else math.inf


# A route captured in the test TUI has no curve_speed_policy; its zones come from the corners
# the capture found (route_capture tracking_profile), with the K-City route YAML defaults.
CAPTURED_ROUTE_SPEED_SCALE = 0.8
CAPTURED_ROUTE_APPROACH_DECEL_MPS2 = 1.4


def captured_route_curve_policy(document: dict) -> dict | None:
    """curve_speed_policy for a manually captured route: one zone per detected corner."""
    profile = document.get("tracking_profile")
    if document.get("capture_mode") != "manual_tui" or not isinstance(profile, dict):
        return None
    length = _route_length(_route_points(document))
    zones = []
    for corner in profile.get("corners") or []:
        start = max(0.0, _finite_number(corner.get("start_s_m"), "corner start_s_m"))
        end = min(length, _finite_number(corner.get("end_s_m"), "corner end_s_m"))
        if end <= start:
            continue
        if zones and start <= zones[-1]["end_s"]:
            zones[-1]["end_s"] = max(zones[-1]["end_s"], end)
            continue
        zones.append({"id": f"C{len(zones) + 1}", "start_s": start, "end_s": end})
    if not zones:
        return None
    return {
        "speed_scale": CAPTURED_ROUTE_SPEED_SCALE,
        "approach_decel_mps2": CAPTURED_ROUTE_APPROACH_DECEL_MPS2,
        "zones": zones,
    }


def load_curve_speed_policy(
    route_config: str,
    *,
    speed_scale_override: float | None = None,
    lateral_accel_mps2: float = 0.0,
    approach_decel_override_mps2: float | None = None,
) -> CurveSpeedPolicy | None:
    """Read the same route YAML as the route manager; reject malformed policies.

    lateral_accel_mps2 > 0 adds a radius cap per zone; approach_decel_override_mps2 replaces
    the YAML approach deceleration (both from decision_pipeline.yaml).
    """
    if speed_scale_override is not None:
        speed_scale_override = validate_curve_speed_scale(speed_scale_override)
    lateral_accel_mps2 = resolve_curve_lateral_accel(lateral_accel_mps2)
    if approach_decel_override_mps2 is not None:
        approach_decel_override_mps2 = resolve_curve_approach_decel_override(
            approach_decel_override_mps2
        )
    if not isinstance(route_config, str):
        raise ValueError("route_config must be a string")
    if not route_config.strip():
        return None
    try:
        with Path(route_config).open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot load curve speed route config: {route_config}") from error
    if not isinstance(document, dict):
        raise ValueError("route config must be a mapping")
    raw_policy = document.get("curve_speed_policy")
    if raw_policy is None:
        raw_policy = captured_route_curve_policy(document)
    if raw_policy is None:
        return None

    policy = _require_fields(
        raw_policy,
        {"speed_scale", "approach_decel_mps2", "zones"},
        "curve_speed_policy",
    )
    scale = validate_curve_speed_scale(policy["speed_scale"])
    decel = _finite_number(policy["approach_decel_mps2"], "approach_decel_mps2")
    if decel <= 0.0:
        raise ValueError("approach_decel_mps2 must be positive")
    if not isinstance(policy["zones"], list) or not policy["zones"]:
        raise ValueError("curve speed zones must be a nonempty list")

    points = _route_points(document)
    route_length = _route_length(points)
    zones: list[CurveSpeedZone] = []
    zone_ids: set[str] = set()
    for index, item in enumerate(policy["zones"]):
        zone = _require_fields(item, {"id", "start_s", "end_s"}, f"zones[{index}]")
        zone_id = zone["id"]
        if not isinstance(zone_id, str) or not zone_id.strip():
            raise ValueError(f"zones[{index}].id must be a nonempty string")
        if zone_id != zone_id.strip() or zone_id in zone_ids:
            raise ValueError("curve speed zone IDs must be unique without outer whitespace")
        start = _finite_number(zone["start_s"], f"zones[{index}].start_s")
        end = _finite_number(zone["end_s"], f"zones[{index}].end_s")
        if not 0.0 <= start < end <= route_length:
            raise ValueError(f"zone {zone_id} must satisfy 0 <= start_s < end_s <= route length")
        if zones and start <= zones[-1].end_s:
            raise ValueError("curve speed zones must be sorted and nonoverlapping")
        radius = _tightest_radius(points, start, end) if lateral_accel_mps2 > 0.0 else math.inf
        zones.append(CurveSpeedZone(zone_id, start, end, radius))
        zone_ids.add(zone_id)
    if speed_scale_override is not None:
        scale = speed_scale_override
    if approach_decel_override_mps2 is not None:
        decel = approach_decel_override_mps2
    return CurveSpeedPolicy(scale, decel, tuple(zones), route_length, lateral_accel_mps2)
