from __future__ import annotations

from dataclasses import dataclass, field
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
    commit_distance_m: float = 6.0
    arrow_maneuver: str = "NONE"
    brake_trigger_waypoint: int | None = None
    brake_trigger_route_s: float | None = None


@dataclass(frozen=True)
class StagePolicy:
    stage_id: str
    traffic_light_enabled: bool = False
    obstacle_enabled: bool = False
    boundary_stop_line_id: str = ""
    boundary_route_s: float | None = None
    event: EventPolicy | None = None


@dataclass(frozen=True)
class ZonePolicy:
    course_id: str
    ready: bool
    stages: tuple[StagePolicy, ...]
    pending_data: tuple[str, ...] = ()

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
    return EventPolicy(
        event_id=event_id,
        event_type=event_type,
        stop_line_id=str(value.get("stop_line_id", "")).strip(),
        traffic_light_ids=tuple(str(item).strip() for item in traffic_light_ids_value),
        approach_distance_m=max(0.1, float(value.get("approach_distance_m", 18.0))),
        hold_sec=max(0.0, float(value.get("hold_sec", 0.0))),
        commit_distance_m=max(0.1, float(value.get("commit_distance_m", 6.0))),
        arrow_maneuver=arrow_maneuver,
        brake_trigger_waypoint=(
            None if waypoint_value is None else int(waypoint_value)
        ),
        brake_trigger_route_s=(
            None if route_s_value is None else float(route_s_value)
        ),
    )


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
