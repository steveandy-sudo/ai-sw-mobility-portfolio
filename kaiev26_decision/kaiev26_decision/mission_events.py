"""Stop events of a run, built once at start from the course policy, the landmarks and the mode.

stopline mode turns signal events into timed stops of test_stop_hold_sec without lights;
avoidance and tracking keep no stop events. Every mode ends with the finish event at the route end.
"""
from __future__ import annotations

from dataclasses import dataclass

from kaiev26_decision.stop_point import (
    KIND_FINISH,
    KIND_SIGNAL,
    KIND_TIMED,
    StopEvent,
    validate_stop_events,
)


MISSION_MODES = ("all", "tracking", "stopline", "stopline_signal", "avoidance")


@dataclass(frozen=True)
class EventSettings:
    default_brake_margin_m: float = 1.0
    test_stop_hold_sec: float = 3.0
    finish_approach_m: float = 20.0
    finish_slow_speed_mps: float = 0.6
    finish_stop_distance_m: float = 1.0
    finish_overrun_m: float = 5.0
    # Surveyed finish line (policy finish_line_route_s): the line was surveyed with this point of
    # the car on it, this far ahead of the rear axle, and the car brakes once that point is this
    # margin past the line.
    finish_line_reference_forward_m: float = 0.0
    finish_line_pass_margin_m: float = 2.0


# The finish watch starts this far before a fixed brake station (covers a planning tick).
FINISH_LINE_WATCH_LEAD_M = 2.0


def build_stop_events(
    policy,
    stopline_s: dict[str, float],
    route_length_m: float,
    mission_mode: str,
    settings: EventSettings = EventSettings(),
) -> tuple[StopEvent, ...]:
    if mission_mode not in MISSION_MODES:
        raise ValueError(f"unsupported mission_mode [{mission_mode}]")
    events: list[StopEvent] = []
    finish_id = "FINISH"
    finish_line_s = None
    for stage in policy.stages if policy is not None else ():
        event = stage.event
        if event is None:
            continue
        if event.event_type == "FINISH":
            finish_id = event.event_id
            finish_line_s = event.finish_line_route_s
            continue
        if event.event_type not in {KIND_SIGNAL, KIND_TIMED}:
            continue
        if mission_mode in {"tracking", "avoidance"}:
            continue
        if event.stop_line_id not in stopline_s:
            raise ValueError(
                f"event [{event.event_id}] stop line [{event.stop_line_id}] is not on the route"
            )
        kind, lights, arrow, hold = event.event_type, event.traffic_light_ids, event.arrow_maneuver, event.hold_sec
        if mission_mode == "stopline":
            kind, lights, arrow, hold = KIND_TIMED, (), "NONE", max(0.1, settings.test_stop_hold_sec)
        events.append(StopEvent(
            event_id=event.event_id,
            kind=kind,
            line_s=float(stopline_s[event.stop_line_id]),
            approach_m=event.approach_distance_m,
            brake_margin_m=(
                settings.default_brake_margin_m if event.brake_margin_m is None else event.brake_margin_m
            ),
            stop_line_id=event.stop_line_id,
            light_ids=tuple(lights),
            arrow=arrow,
            hold_sec=hold,
            commit_m=event.commit_distance_m,
            brake_wp=event.brake_trigger_waypoint,
            brake_wp_s=event.brake_trigger_route_s,
        ))
    if finish_line_s is None:
        events.append(StopEvent(
            event_id=finish_id,
            kind=KIND_FINISH,
            line_s=float(route_length_m),
            approach_m=settings.finish_approach_m,
            brake_margin_m=-settings.finish_overrun_m,
            hard_stop_rear_m=settings.finish_stop_distance_m,
            watch_cap_mps=settings.finish_slow_speed_mps,
        ))
    else:
        # Surveyed line: no slow-down and no d_stop(v) brake point before it. The course speed
        # holds until the surveyed point of the car is past the line, then the car brakes and
        # stops on the route past it (the route end stays the event line).
        brake_s = (
            float(finish_line_s) - settings.finish_line_reference_forward_m + settings.finish_line_pass_margin_m)
        if not brake_s < float(route_length_m):
            raise ValueError(
                f"finish brake station {brake_s:.1f} m (line {finish_line_s:.1f} m) is not before "
                f"the route end {float(route_length_m):.1f} m"
            )
        events.append(StopEvent(
            event_id=finish_id,
            kind=KIND_FINISH,
            line_s=float(route_length_m),
            approach_m=float(route_length_m) - brake_s + FINISH_LINE_WATCH_LEAD_M,
            brake_margin_m=0.0,
            brake_s=brake_s,
        ))
    validate_stop_events(events)
    return tuple(sorted(events, key=lambda item: item.line_s))


def obstacle_zone_finish_check(zone_exit_s: float, return_length_m: float, finish_brake_s: float) -> str:
    """D18 launch check: the zone end plus the bypass return length must lie before the finish
    brake point. Returns the problem, or an empty string."""
    end = zone_exit_s + return_length_m
    if end < finish_brake_s:
        return ""
    return (
        f"obstacle zone end {zone_exit_s:.1f} m + return {return_length_m:.1f} m = {end:.1f} m "
        f"is not before the finish brake point {finish_brake_s:.1f} m"
    )
