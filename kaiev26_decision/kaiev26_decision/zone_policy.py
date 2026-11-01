from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path

import yaml


SUPPORTED_EVENT_TYPES = {"TIMED_STOP", "SIGNAL_OBEY", "PASS_THROUGH", "FINISH"}
SUPPORTED_ARROW_MANEUVERS = {"NONE", "LEFT", "RIGHT"}


@dataclass(frozen=True)
class EventPolicy:
    event_id: str
    event_type: str
    stop_line_id: str = ""
    traffic_light_ids: tuple[str, ...] = ()
    approach_distance_m: float = 18.0
    hold_sec: float = 0.0
    commit_distance_m: float = 8.0
    arrow_maneuver: str = "NONE"
    brake_trigger_waypoint: int | None = None
    brake_trigger_route_s: float | None = None
    # Front-bumper margin added to d_stop(v) for this event; None uses the node parameter.
    brake_margin_m: float | None = None
    # FINISH only: route s where the route crosses the surveyed painted line, and the surveyed
    # (latitude_deg, longitude_deg) points on it. None keeps the route end as the finish line.
    finish_line_route_s: float | None = None
    finish_line_points: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class StagePolicy:
    stage_id: str
    traffic_light_enabled: bool = False
    obstacle_enabled: bool = False
    boundary_stop_line_id: str = ""
    boundary_route_s: float | None = None
    event: EventPolicy | None = None
    # Course speed for this stage in place of the launch base speed (the TUI v), or None. The
    # curve, zone and stop caps still apply below it.
    course_speed_mps: float | None = None


@dataclass(frozen=True)
class ObstacleZonePolicy:
    """Map-frame polygon of the obstacle zone, the surveyed lane lines bounding the lane and the
    zone speed cap with the deceleration that reaches it at the entry (decisions D10, D14).
    start_route_s, when given, is the zone entry for the speed cap; otherwise the polygon's.

    lane_offsets_m (right, left) instead fixes the lane at constant route offsets with no polygon
    or map: the zone runs from start_route_s to the route end and every object counts, so a test
    route gets the final's bypass search (Midan: the final lane at the drums, -1.96 / +2.14 m)."""

    polygon: tuple[tuple[float, float], ...]
    map_semantics: str
    right_boundary_ids: tuple[str, ...]
    left_boundary_ids: tuple[str, ...]
    speed_cap_mps: float
    pre_slow_decel_mps2: float
    boundary_max_offset_m: float = 3.0
    start_route_s: float | None = None
    lane_offsets_m: tuple[float, float] | None = None


@dataclass(frozen=True)
class SpeedZoneCap:
    """Course speed-limit zone (1017 scenario: qualifying school zone 30 km/h).

    The cap holds from the front bumper at start_route_s until the rear axle is past
    end_route_s. Before it the cap falls at pre_slow_decel_mps2 as for the obstacle zone.
    """

    zone_id: str
    start_route_s: float
    end_route_s: float
    speed_cap_mps: float
    pre_slow_decel_mps2: float


@dataclass(frozen=True)
class ZonePolicy:
    course_id: str
    ready: bool
    stages: tuple[StagePolicy, ...]
    pending_data: tuple[str, ...] = ()
    obstacle_zone: ObstacleZonePolicy | None = None
    speed_zones: tuple[SpeedZoneCap, ...] = ()

    def stage_index(self, stage_id: str) -> int | None:
        for index, stage in enumerate(self.stages):
            if stage.stage_id == stage_id:
                return index
        return None

    def zone_ranges(
        self,
        stopline_progress: dict[str, float],
        route_length_m: float,
    ) -> list[dict[str, float | str]]:
        if not self.ready:
            raise ValueError(f"zone policy [{self.course_id}] is not ready")

        ranges: list[dict[str, float | str]] = []
        start_s = 0.0
        for index, stage in enumerate(self.stages):
            is_last = index == len(self.stages) - 1
            if is_last:
                end_s = route_length_m
            elif stage.boundary_route_s is not None:
                end_s = float(stage.boundary_route_s)
            else:
                stop_line_id = stage.boundary_stop_line_id
                if stop_line_id not in stopline_progress:
                    raise ValueError(
                        f"stage [{stage.stage_id}] boundary stop line is not on the route: "
                        f"{stop_line_id}"
                    )
                end_s = float(stopline_progress[stop_line_id])

            if end_s <= start_s:
                raise ValueError(
                    f"stage [{stage.stage_id}] ends at {end_s:.2f} m, "
                    f"not after {start_s:.2f} m"
                )
            ranges.append(
                {
                    "zone": stage.stage_id,
                    "start_s": start_s,
                    "end_s": end_s,
                }
            )
            start_s = end_s
        return ranges


@dataclass
class ZonePolicyRuntime:
    policy: ZonePolicy
    stage_index_value: int | None = None
    completed_event_ids: set[str] = field(default_factory=set)

    def stage_for(self, route_zone: str) -> StagePolicy | None:
        candidate = self.policy.stage_index(route_zone)
        if candidate is not None:
            if self.stage_index_value is None or candidate > self.stage_index_value:
                self.stage_index_value = candidate
        if self.stage_index_value is None:
            return None
        return self.policy.stages[self.stage_index_value]

    def event_for(self, route_zone: str, next_stop_line_id: str) -> EventPolicy | None:
        stage = self.stage_for(route_zone)
        if stage is None or stage.event is None:
            return None
        event = stage.event
        if event.event_id in self.completed_event_ids:
            return None
        if event.stop_line_id and event.stop_line_id != next_stop_line_id:
            return None
        return event

    def complete(self, event_id: str) -> None:
        if event_id:
            self.completed_event_ids.add(event_id)


def _required_text(mapping: dict, key: str, context: str) -> str:
    value = str(mapping.get(key, "")).strip()
    if not value:
        raise ValueError(f"{context} requires [{key}]")
    return value


def _parse_event(value: object, context: str) -> EventPolicy | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError(f"{context}.event must be a mapping")
    event_id = _required_text(value, "id", f"{context}.event")
    event_type = _required_text(value, "type", f"{context}.event").upper()
    if event_type not in SUPPORTED_EVENT_TYPES:
        raise ValueError(f"{context}.event has unsupported type [{event_type}]")
    traffic_light_ids_value = value.get("traffic_light_ids", [])
    if not isinstance(traffic_light_ids_value, list):
        raise ValueError(f"{context}.event.traffic_light_ids must be a list")
    arrow_maneuver = str(value.get("arrow_maneuver", "NONE")).strip().upper()
    if arrow_maneuver not in SUPPORTED_ARROW_MANEUVERS:
        raise ValueError(
            f"{context}.event.arrow_maneuver must be one of "
            f"{sorted(SUPPORTED_ARROW_MANEUVERS)}"
        )
    waypoint_value = value.get("brake_trigger_waypoint")
    route_s_value = value.get("brake_trigger_route_s")
    margin_value = value.get("brake_margin_m")
    if margin_value is not None and not 0.0 <= float(margin_value) <= 10.0:
        raise ValueError(f"{context}.event.brake_margin_m must be within [0, 10] m")
    line_s_value = value.get("finish_line_route_s")
    line_points_value = value.get("finish_line_points", [])
    if not isinstance(line_points_value, list):
        raise ValueError(f"{context}.event.finish_line_points must be a list")
    line_points = tuple(
        (float(point["latitude_deg"]), float(point["longitude_deg"])) for point in line_points_value
    )
    if (line_s_value is not None or line_points) and event_type != "FINISH":
        raise ValueError(f"{context}.event finish_line_* is for FINISH events only")
    if line_s_value is not None and not (math.isfinite(float(line_s_value)) and float(line_s_value) > 0.0):
        raise ValueError(f"{context}.event.finish_line_route_s must be positive")
    return EventPolicy(
        event_id=event_id,
        event_type=event_type,
        stop_line_id=str(value.get("stop_line_id", "")).strip(),
        traffic_light_ids=tuple(str(item).strip() for item in traffic_light_ids_value),
        approach_distance_m=max(0.1, float(value.get("approach_distance_m", 18.0))),
        hold_sec=max(0.0, float(value.get("hold_sec", 0.0))),
        commit_distance_m=max(0.1, float(value.get("commit_distance_m", 8.0))),
        arrow_maneuver=arrow_maneuver,
        brake_trigger_waypoint=(
            None if waypoint_value is None else int(waypoint_value)
        ),
        brake_trigger_route_s=(
            None if route_s_value is None else float(route_s_value)
        ),
        brake_margin_m=None if margin_value is None else float(margin_value),
        finish_line_route_s=None if line_s_value is None else float(line_s_value),
        finish_line_points=line_points,
    )


def _parse_obstacle_zone(value: object) -> ObstacleZonePolicy | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("obstacle_zone must be a mapping")
    speeds = {}
    for key in ("speed_cap_mps", "pre_slow_decel_mps2"):
        speeds[key] = float(value.get(key, math.nan))
        if not math.isfinite(speeds[key]) or speeds[key] <= 0.0:
            raise ValueError(f"obstacle_zone.{key} must be positive")
    start = value.get("start_route_s")
    if start is not None:
        start = float(start)
        if not math.isfinite(start) or start <= 0.0:
            raise ValueError("obstacle_zone.start_route_s must be positive")
    offsets = value.get("lane_offsets_m")
    if offsets is not None:
        if not isinstance(offsets, dict):
            raise ValueError("obstacle_zone.lane_offsets_m must be a mapping")
        if value.get("polygon") or value.get("lane_boundaries"):
            raise ValueError("obstacle_zone takes lane_offsets_m or a polygon with lane_boundaries, not both")
        if start is None:
            raise ValueError("obstacle_zone.lane_offsets_m needs start_route_s")
        lane_offsets = (float(offsets.get("right", math.nan)), float(offsets.get("left", math.nan)))
        if not all(math.isfinite(v) for v in lane_offsets) or not lane_offsets[0] < 0.0 < lane_offsets[1]:
            raise ValueError("obstacle_zone.lane_offsets_m needs right < 0 < left")
        return ObstacleZonePolicy(
            polygon=(), map_semantics="", right_boundary_ids=(), left_boundary_ids=(),
            **speeds, start_route_s=start, lane_offsets_m=lane_offsets,
        )
    points = value.get("polygon")
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("obstacle_zone.polygon needs at least three map points")
    polygon = tuple((float(point["x"]), float(point["y"])) for point in points)
    boundaries = value.get("lane_boundaries") or {}
    right = tuple(str(item).strip() for item in boundaries.get("right", []))
    left = tuple(str(item).strip() for item in boundaries.get("left", []))
    if not right or not left:
        raise ValueError("obstacle_zone.lane_boundaries needs right and left lane marking ids")
    return ObstacleZonePolicy(
        polygon=polygon,
        map_semantics=_required_text(value, "map_semantics", "obstacle_zone"),
        right_boundary_ids=right,
        left_boundary_ids=left,
        **speeds,
        boundary_max_offset_m=max(0.5, float(value.get("boundary_max_offset_m", 3.0))),
        start_route_s=start,
    )


def _parse_speed_zones(value: object) -> tuple[SpeedZoneCap, ...]:
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ValueError("zone policy speed_zones must be a list")
    zones = []
    for index, item in enumerate(value):
        context = f"speed_zones[{index}]"
        if not isinstance(item, dict):
            raise ValueError(f"{context} must be a mapping")
        zone_id = _required_text(item, "id", context)
        start_s = float(item.get("start_route_s", math.nan))
        end_s = float(item.get("end_route_s", math.nan))
        cap = float(item.get("speed_cap_mps", math.nan))
        decel = float(item.get("pre_slow_decel_mps2", math.nan))
        if not math.isfinite(start_s) or start_s < 0.0:
            raise ValueError(f"{context}.start_route_s must be non-negative")
        if not math.isfinite(end_s) or end_s <= start_s:
            raise ValueError(f"{context}.end_route_s must be after start_route_s")
        if not math.isfinite(cap) or cap <= 0.0:
            raise ValueError(f"{context}.speed_cap_mps must be positive")
        if not math.isfinite(decel) or decel <= 0.0:
            raise ValueError(f"{context}.pre_slow_decel_mps2 must be positive")
        zones.append(SpeedZoneCap(zone_id, start_s, end_s, cap, decel))
    if len({zone.zone_id for zone in zones}) != len(zones):
        raise ValueError("zone policy speed_zones IDs must be unique")
    return tuple(zones)


def _parse_course_speed(value: object, context: str) -> float | None:
    if value is None:
        return None
    speed = float(value)
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError(f"{context}.course_speed_mps must be positive")
    return speed


def load_zone_policy(path: Path | str, *, require_ready: bool = True) -> ZonePolicy:
    policy_path = Path(path).expanduser()
    with policy_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"zone policy must be a mapping: {policy_path}")

    stages_value = document.get("stages", [])
    if not isinstance(stages_value, list) or not stages_value:
        raise ValueError("zone policy requires a non-empty stages list")

    stages = []
    for index, value in enumerate(stages_value):
        context = f"stages[{index}]"
        if not isinstance(value, dict):
            raise ValueError(f"{context} must be a mapping")
        stages.append(
            StagePolicy(
                stage_id=_required_text(value, "id", context),
                traffic_light_enabled=bool(value.get("traffic_light_enabled", False)),
                obstacle_enabled=bool(value.get("obstacle_enabled", False)),
                boundary_stop_line_id=str(
                    value.get("boundary_stop_line_id", "")
                ).strip(),
                boundary_route_s=(
                    None
                    if value.get("boundary_route_s") is None
                    else float(value["boundary_route_s"])
                ),
                event=_parse_event(value.get("event"), context),
                course_speed_mps=_parse_course_speed(value.get("course_speed_mps"), context),
            )
        )

    pending_value = document.get("pending_data", [])
    if not isinstance(pending_value, list):
        raise ValueError("zone policy pending_data must be a list")
    policy = ZonePolicy(
        course_id=_required_text(document, "course_id", "zone policy"),
        ready=bool(document.get("ready", False)),
        stages=tuple(stages),
        pending_data=tuple(str(item).strip() for item in pending_value),
        obstacle_zone=_parse_obstacle_zone(document.get("obstacle_zone")),
        speed_zones=_parse_speed_zones(document.get("speed_zones")),
    )
    _validate_policy(policy)
    if require_ready and not policy.ready:
        details = "; ".join(policy.pending_data) or "course data is incomplete"
        raise ValueError(f"zone policy [{policy.course_id}] is not ready: {details}")
    return policy


def _validate_policy(policy: ZonePolicy) -> None:
    stage_ids = [stage.stage_id for stage in policy.stages]
    if len(stage_ids) != len(set(stage_ids)):
        raise ValueError("zone policy stage IDs must be unique")

    events = [stage.event for stage in policy.stages if stage.event is not None]
    event_ids = [event.event_id for event in events]
    if len(event_ids) != len(set(event_ids)):
        raise ValueError("zone policy event IDs must be unique")

    if not policy.ready:
        return

    for index, stage in enumerate(policy.stages):
        is_last = index == len(policy.stages) - 1
        if (
            not is_last
            and not stage.boundary_stop_line_id
            and stage.boundary_route_s is None
        ):
            raise ValueError(
                f"stage [{stage.stage_id}] requires a stop-line or route-s boundary"
            )
        if stage.boundary_route_s is not None and stage.boundary_route_s <= 0.0:
            raise ValueError(
                f"stage [{stage.stage_id}] boundary_route_s must be positive"
            )
        event = stage.event
        if event is None:
            continue
        if event.event_type in {"TIMED_STOP", "SIGNAL_OBEY", "PASS_THROUGH"}:
            if not event.stop_line_id:
                raise ValueError(
                    f"event [{event.event_id}] requires stop_line_id"
                )
        if event.stop_line_id and stage.boundary_stop_line_id:
            if event.stop_line_id != stage.boundary_stop_line_id:
                raise ValueError(
                    f"event [{event.event_id}] stop line must match its stage boundary"
                )
        if event.event_type == "TIMED_STOP" and event.hold_sec <= 0.0:
            raise ValueError(f"event [{event.event_id}] requires hold_sec > 0")
        if (event.brake_trigger_waypoint is None) != (
            event.brake_trigger_route_s is None
        ):
            raise ValueError(
                f"event [{event.event_id}] must set brake trigger waypoint and route_s together"
            )
        if (
            event.brake_trigger_waypoint is not None
            and event.brake_trigger_waypoint < 0
        ):
            raise ValueError(
                f"event [{event.event_id}] brake trigger waypoint must be non-negative"
            )
        if event.brake_trigger_route_s is not None and event.brake_trigger_route_s < 0.0:
            raise ValueError(
                f"event [{event.event_id}] brake trigger route_s must be non-negative"
            )
        if event.event_type == "SIGNAL_OBEY":
            if not stage.traffic_light_enabled:
                raise ValueError(
                    f"stage [{stage.stage_id}] must enable traffic lights"
                )
            if not event.traffic_light_ids:
                raise ValueError(
                    f"event [{event.event_id}] requires traffic_light_ids"
                )
        elif event.arrow_maneuver != "NONE":
            raise ValueError(
                f"event [{event.event_id}] may set arrow_maneuver only for SIGNAL_OBEY"
            )
