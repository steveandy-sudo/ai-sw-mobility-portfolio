"""Route-specific cruise ceilings (curves, speed-limit zones) as speed caps.

They only lower caps; they never stop the car.
"""
from __future__ import annotations

import math

from kaiev26_msgs.msg import RouteContext

from kaiev26_decision.curve_speed_policy import CurveSpeedPolicy
from kaiev26_decision.zone_policy import SpeedZoneCap


def curve_speed_cap(
    route: RouteContext | None,
    policy: CurveSpeedPolicy | None,
    cruise_speed_mps: float,
    *,
    route_is_fresh: bool = True,
) -> tuple[float | None, list[str]]:
    """Cap and tags for the current route progress.

    RouteContext has no route ID. Both producers must load the same route_config as the route
    manager; the length check rejects a mismatched course or malformed progress. When the route
    cannot be located the cap is 0 (speed 0 without brake); a stop is the input state's job.
    """
    if policy is None:
        return None, []
    try:
        if route is None or not route_is_fresh or not route.route_projection_valid:
            raise ValueError("unavailable route projection")
        progress = float(route.progress_s)
        remaining = float(route.distance_to_finish)
        # Float32 RouteContext fields round independently at the route endpoint.
        tolerance_m = 0.1
        if (
            not math.isfinite(progress)
            or not math.isfinite(remaining)
            or progress < 0.0
            or remaining < 0.0
            or abs(progress + remaining - policy.route_length_m) > tolerance_m
        ):
            raise ValueError("route progress does not match configured route")
        limit = policy.limit_for_progress(min(progress, policy.route_length_m), cruise_speed_mps)
    except ValueError:
        return 0.0, ["CURVE_SPEED_ROUTE_INVALID"]
    if limit is None:
        return None, []
    return float(limit.speed_limit_mps), [
        f"CURVE_SPEED_{limit.phase.upper()}:{limit.zone_id}",
        f"CURVE_SPEED_SCALE:{float(policy.speed_scale)}",
        f"CURVE_SPEED_LIMIT_MPS:{float(limit.speed_limit_mps)}",
    ]


def speed_zone_cap(
    route: RouteContext | None,
    zones: tuple[SpeedZoneCap, ...],
    cruise_speed_mps: float,
    front_overhang_m: float,
    *,
    route_is_fresh: bool = True,
) -> tuple[float | None, list[str]]:
    """Course speed-limit zones (qualifying school zone) with the obstacle-zone pre-slow.

    Each cap holds from the front bumper at start_route_s until the rear axle (progress_s) is
    past end_route_s; before it the cap falls at pre_slow_decel_mps2. A cap at or above cruise
    is left out. An unlocated route gives no cap: that stop is the input state's job.
    """
    if not zones or route is None or not route_is_fresh or not route.route_projection_valid:
        return None, []
    progress = float(route.progress_s)
    if not math.isfinite(progress):
        return None, []
    best: tuple[float, str] | None = None
    for zone in zones:
        if progress > zone.end_route_s:
            continue
        distance = zone.start_route_s - (progress + front_overhang_m)
        cap = zone.speed_cap_mps
        tag = f"SPEED_ZONE_CAP:{zone.zone_id}"
        if distance > 0.0:
            cap = math.sqrt(cap * cap + 2.0 * zone.pre_slow_decel_mps2 * distance)
            tag = f"SPEED_ZONE_PRE_SLOW:{zone.zone_id}"
        if cap < cruise_speed_mps and (best is None or cap < best[0]):
            best = (cap, tag)
    if best is None:
        return None, []
    return float(best[0]), [best[1]]
