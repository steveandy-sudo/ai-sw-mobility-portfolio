"""One stop-point machine for signal, timed stop and finish events, and the event tracker.

Route stations are rear-axle route s (RouteContext.progress_s). A stop event's line is the
painted line's rear-axle crossing station; the finish line is the route end. Brake points are
front-bumper distances: brake when front_gap <= d_stop(v) + brake_margin_m. The finish targets
an overrun past its line, so its margin is negative (minus the overrun). A finish with a
surveyed painted line instead brakes at a fixed rear-axle station past that line (brake_s),
whatever the speed, and keeps the course speed up to it.

The machine never computes permission. Green confirmation, the yellow dilemma, the arrival
verdict, the timed-stop timer, the 1 s UNKNOWN escape and the 110 s wait are computed outside and
arrive as booleans.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from kaiev26_decision.common import StoppingModel


FREE = "FREE"
WATCH = "WATCH"
BRAKING = "BRAKING"
HELD = "HELD"
LAUNCH = "LAUNCH"
COMMITTED = "COMMITTED"
CLEARED = "CLEARED"
PHASES = (FREE, WATCH, BRAKING, HELD, LAUNCH, COMMITTED, CLEARED)

KIND_SIGNAL = "SIGNAL_OBEY"
KIND_TIMED = "TIMED_STOP"
KIND_FINISH = "FINISH"
BEHAVIOR_OF_KIND = {
    KIND_SIGNAL: "TRAFFIC_LIGHT",
    KIND_TIMED: "STATIC_STOP",
    KIND_FINISH: "FINISH",
}
# Phase tags per kind. motion_control reads the PRE_DECEL, RELEASE and COMMITTED values
# (kaiev26_msgs/TargetSpeed CONSTRAINT_*).
TAGS = {
    KIND_SIGNAL: {
        "pre_decel": "SIGNAL_BRAKE_TRIGGER_PRE_DECEL",
        BRAKING: "SIGNAL_HARD_BRAKE",
        HELD: "RED_LIGHT_HOLD",
        LAUNCH: "SIGNAL_RELEASE",
        COMMITTED: "SIGNAL_PASS_COMMITTED",
    },
    KIND_TIMED: {
        "pre_decel": "BRAKE_TRIGGER_PRE_DECEL",
        BRAKING: "TIMED_STOP_CONFIRM",
        HELD: "TIMED_STOP_HOLD",
        LAUNCH: "TIMED_STOP_RELEASE",
        COMMITTED: "TIMED_STOP_RELEASE",
    },
    KIND_FINISH: {
        WATCH: "FINISH_DECEL",
        BRAKING: "FINISH_STOP",
        HELD: "FINISH_HELD",
    },
}
# fsm_state shown to people (docs/ARCHITECTURE.md). Only the BehaviorDecision label; control reads
# the tags above.
STATE_PREFIX = {KIND_SIGNAL: "SIGNAL", KIND_TIMED: "TIMED_STOP", KIND_FINISH: "FINISH"}


def state_name(kind: str, phase: str, *, go: bool = False, pre_decel: bool = False) -> str:
    """WATCH splits by what the machine already knows: pre-deceleration, then a signal's permission."""
    prefix = STATE_PREFIX[kind]
    if phase == WATCH:
        if pre_decel:
            return f"{prefix}_PRE_BRAKE"
        if kind == KIND_SIGNAL:
            return "SIGNAL_APPROACH_GO" if go else "SIGNAL_APPROACH_STOP"
        return f"{prefix}_APPROACH"
    if phase == BRAKING:
        return f"{prefix}_BRAKE"
    if phase == HELD:
        return "FINISH_STOPPED" if kind == KIND_FINISH else f"{prefix}_WAIT"
    if phase in {LAUNCH, COMMITTED}:
        return f"{prefix}_PASS"
    return phase


@dataclass(frozen=True)
class StopEvent:
    event_id: str
    kind: str
    line_s: float
    approach_m: float
    brake_margin_m: float
    stop_line_id: str = ""
    light_ids: tuple[str, ...] = ()
    arrow: str = "NONE"
    hold_sec: float = 0.0
    commit_m: float = 8.0
    brake_wp: int | None = None
    brake_wp_s: float | None = None
    hard_stop_rear_m: float | None = None
    watch_cap_mps: float | None = None
    # Fixed rear-axle station where braking starts, in place of the d_stop(v) and hard-stop rules.
    brake_s: float | None = None

    @property
    def behavior(self) -> str:
        return BEHAVIOR_OF_KIND[self.kind]

    @property
    def watch_start_s(self) -> float:
        return self.line_s - self.approach_m


@dataclass(frozen=True)
class Permission:
    """Booleans computed outside the machine for the current event."""

    go: bool = False
    dilemma: bool = False
    release: bool = False
    # The pass is decided (event_permission arrival verdict): commit now, wherever the car is.
    commit: bool = False
    reason: str = ""
    tags: tuple[str, ...] = ()


@dataclass(frozen=True)
class StopPointOutput:
    event: StopEvent
    phase: str
    cap_mps: float | None = None
    stop: bool = False
    tags: tuple[str, ...] = ()
    reason: str = ""
    state: str = ""


class StopPointMachine:
    def __init__(
        self,
        event: StopEvent,
        *,
        front_overhang_m: float,
        stopping_model: StoppingModel,
        stopped_speed_mps: float = 0.1,
        pre_decel_window_m: float = 8.0,
        pre_decel_cap_mps: float = 2.0,
    ) -> None:
        self.event = event
        self.front_overhang_m = max(0.0, front_overhang_m)
        self.stopping_model = stopping_model
        self.stopped_speed_mps = max(0.0, stopped_speed_mps)
        self.pre_decel_window_m = max(0.1, pre_decel_window_m)
        self.pre_decel_cap_mps = max(0.0, pre_decel_cap_mps)
        self.phase = FREE

    def reset(self) -> None:
        self.phase = FREE

    def brake_point_reached(self, rear_gap: float, speed: float) -> bool:
        event = self.event
        if event.brake_s is not None:
            return event.line_s - rear_gap >= event.brake_s
        front_gap = rear_gap - self.front_overhang_m
        if front_gap <= self.stopping_model.distance_m(speed) + event.brake_margin_m:
            return True
        return event.hard_stop_rear_m is not None and rear_gap <= event.hard_stop_rear_m

    def brake_point_s(self, speed_mps: float) -> float:
        """Rear-axle station where brake_point_reached first holds at this speed."""
        event = self.event
        if event.brake_s is not None:
            return event.brake_s
        rear_gap = self.front_overhang_m + self.stopping_model.distance_m(speed_mps) + event.brake_margin_m
        if event.hard_stop_rear_m is not None:
            rear_gap = max(rear_gap, event.hard_stop_rear_m)
        return event.line_s - rear_gap

    def update(self, s_rear: float, speed_mps: float, permission: Permission) -> StopPointOutput:
        """Advance with a finite rear-axle station and measured speed magnitude."""
        event = self.event
        speed = abs(float(speed_mps))
        rear_gap = event.line_s - float(s_rear)
        cap = None
        tags: list[str] = []
        reason = ""
        pre_decel = False

        if self.phase == FREE:
            if rear_gap < 0.0:
                self.phase = CLEARED
            elif rear_gap <= event.approach_m:
                self.phase = WATCH

        if self.phase == WATCH:
            if rear_gap < 0.0:
                self.phase = CLEARED
                reason = "line passed without a stop"
            elif event.kind == KIND_SIGNAL and permission.commit:
                self.phase = COMMITTED
                reason = permission.reason or "pass committed"
            elif event.kind == KIND_SIGNAL and permission.go and rear_gap <= event.commit_m:
                self.phase = COMMITTED
                reason = "permitted signal confirmed inside the commit distance"
            elif (
                event.kind == KIND_SIGNAL
                and permission.dilemma
                and rear_gap - self.front_overhang_m < self.stopping_model.distance_m(speed)
            ):
                self.phase = COMMITTED
                reason = "permission changed with the stop line inside the stopping distance"
            elif not permission.go and self.brake_point_reached(rear_gap, speed):
                self.phase = BRAKING
                reason = "brake point reached" + (f" ({permission.reason})" if permission.reason else "")
            else:
                pre_decel = (
                    not permission.go
                    and event.brake_wp_s is not None
                    and event.brake_wp_s - float(s_rear) <= self.pre_decel_window_m
                )
                if pre_decel:
                    cap = self.pre_decel_cap_mps
                    tags.append(TAGS[event.kind]["pre_decel"])
                    reason = "pre-deceleration before the brake waypoint"
                else:
                    reason = "approaching"

        if self.phase == BRAKING and speed <= self.stopped_speed_mps:
            self.phase = HELD
            reason = "stopped"
        elif self.phase == HELD and permission.release:
            self.phase = LAUNCH
            reason = permission.reason or "released"
        elif self.phase == HELD:
            # The wait shown to people, such as "holding timed stop 1.2/3.0s" (label only).
            reason = permission.reason
        elif self.phase == LAUNCH:
            self.phase = COMMITTED

        if self.phase == COMMITTED and rear_gap < 0.0:
            self.phase = CLEARED
            reason = "line passed"

        if self.phase == WATCH and event.watch_cap_mps is not None:
            cap = event.watch_cap_mps if cap is None else min(cap, event.watch_cap_mps)
        phase_tag = TAGS[event.kind].get(self.phase)
        if phase_tag is not None:
            tags.append(phase_tag)
        tags.extend(permission.tags)
        return StopPointOutput(
            event=event,
            phase=self.phase,
            cap_mps=cap,
            stop=self.phase in {BRAKING, HELD},
            tags=tuple(dict.fromkeys(tags)),
            reason=reason or self.phase.lower(),
            state=state_name(event.kind, self.phase, go=permission.go, pre_decel=pre_decel),
        )

    def hold_output(self, speed_mps: float | None = None, reason: str = "inputs stale; phase kept") -> StopPointOutput:
        """Output without advancing, used while the inputs the machine needs are stale.

        A BRAKING or HELD machine keeps requesting the brake, so staleness never releases it.
        The one step taken is BRAKING -> HELD when a fresh measured speed shows the car stopped:
        it needs no route or permission, and without it a finish stop that ends where the route
        projection runs out would never report FINISH_HELD (D11).
        """
        if (
            self.phase == BRAKING
            and speed_mps is not None
            and abs(float(speed_mps)) <= self.stopped_speed_mps
        ):
            self.phase = HELD
        phase_tag = TAGS[self.event.kind].get(self.phase)
        return StopPointOutput(
            event=self.event,
            phase=self.phase,
            stop=self.phase in {BRAKING, HELD},
            tags=(phase_tag,) if phase_tag else (),
            reason=reason,
            state=state_name(self.event.kind, self.phase),
        )


@dataclass
class EventTracker:
    """Selection, commit and completion of the ordered stop events in one place.

    Only the current event has a machine. The tracker advances only from CLEARED, so it never
    leaves a BRAKING or HELD event, also when the car stopped past the line.
    """

    events: tuple[StopEvent, ...]
    machine_options: dict = field(default_factory=dict)
    index: int = 0
    cleared_ids: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.events = tuple(sorted(self.events, key=lambda event: event.line_s))
        self.machine = self._machine_for(self.index)

    def _machine_for(self, index: int) -> StopPointMachine | None:
        if index >= len(self.events):
            return None
        return StopPointMachine(self.events[index], **self.machine_options)

    @property
    def current(self) -> StopEvent | None:
        return None if self.machine is None else self.machine.event

    @property
    def phase(self) -> str:
        return CLEARED if self.machine is None else self.machine.phase

    def reset_current(self) -> None:
        if self.machine is not None:
            self.machine.reset()

    def update(self, s_rear: float, speed_mps: float, permission_for) -> StopPointOutput | None:
        """permission_for(event, phase) -> Permission, evaluated before each machine step."""
        while self.machine is not None:
            machine = self.machine
            output = machine.update(s_rear, speed_mps, permission_for(machine.event, machine.phase))
            if machine.phase != CLEARED:
                return output
            self.cleared_ids.append(machine.event.event_id)
            self.index += 1
            self.machine = self._machine_for(self.index)
        return None

    def hold_output(self, speed_mps: float | None = None, reason: str = "inputs stale; phase kept") -> StopPointOutput | None:
        return None if self.machine is None else self.machine.hold_output(speed_mps, reason)


def validate_stop_events(events) -> None:
    """Refuse event lists whose approach windows overlap (one machine serves one event)."""
    ordered = sorted(events, key=lambda event: event.line_s)
    for event in ordered:
        if not math.isfinite(event.line_s) or event.approach_m <= 0.0:
            raise ValueError(f"stop event [{event.event_id}] needs a finite line and approach")
        if event.brake_s is not None and not event.watch_start_s < event.brake_s < event.line_s:
            raise ValueError(
                f"stop event [{event.event_id}] brake station {event.brake_s:.1f} m must lie inside "
                f"its approach ({event.watch_start_s:.1f}-{event.line_s:.1f} m)"
            )
    for first, second in zip(ordered, ordered[1:]):
        if second.watch_start_s < first.line_s:
            raise ValueError(
                f"approach of [{second.event_id}] starts at {second.watch_start_s:.1f} m, "
                f"before the line of [{first.event_id}] at {first.line_s:.1f} m"
            )
