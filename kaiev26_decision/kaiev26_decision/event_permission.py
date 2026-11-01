"""Permission booleans for the stop-point machine, computed outside it.

A signal event reads only its own lights (StopEvent.light_ids), also after it overshot its line.
Stopped past its line with its own light UNKNOWN, the signal phase tracker's permitted_now
stands in for the light when the tracker reports that light aligned; otherwise the 1 s UNKNOWN
escape applies (D5, the same scope as kgm bf2cc01/8f503ac).
Stopped (BRAKING or HELD), before or past its line, an own-light reading that would grant permission is dropped
while the signal input forbids the launch (signal_schedule.SignalInput.schedule_forbids: near a
scheduled change the aligned schedule does not permit, or its permission ends within
launch_min_remaining_sec). There the schedule only withholds the launch (owner kgm, 2026-10-16:
pv26 reads a false arrow on red).
A timed stop releases after hold_sec stopped; speed above the stopped threshold restarts it.
The finish never gets permission.

Arrival verdict (2026-10-20 Gazebo F3 g98.5: the arrow had 4 s left 29 m out at 7 m/s, the car
kept going and crossed the line on yellow). Approaching on a confirmed permission whose scheduled
end is known (Arrival.permission_left_sec, from the aligned phase estimate), the pass or the stop
is decided once, on the first approach tick at the decision point: the brake point, the last
place a normal stop is still possible, or the commit distance when a slow car reaches it first:

- PASS when the front bumper reaches the line arrival_margin_sec before the permission ends
  (front gap / speed + margin <= time left). The margin covers the K-City controller clock, locked
  to the UTC 100 s grid within about 1 s, and the theta error of the estimate (+-0.5 s between
  runs). The machine commits at once (Permission.commit).
- STOP otherwise: the permission no longer counts for this approach and the machine brakes at
  its brake point, as for a red. A stop that follows reads the light again from the stop.

The verdict is not taken back: PASS commits the machine and STOP starts its brake. Without a
known permission end (no aligned schedule, a distrusted or camera-only one) or without a
confirmed permission at the brake point no verdict is made and the approach behaves as before.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from kaiev26_decision.stop_point import BRAKING, HELD, KIND_SIGNAL, KIND_TIMED, WATCH, Permission, StopEvent


# Permission is computed before the machine steps. A BRAKING machine that measures the stop turns
# HELD in this same step, so stopped-time timers already count from that tick.
STOPPED_PHASES = frozenset({BRAKING, HELD})


UNKNOWN = "UNKNOWN"
RED = "RED"
YELLOW = "YELLOW"
GREEN = "GREEN"
ARROW = "ARROW"

VERDICT_PASS = "PASS"
VERDICT_STOP = "STOP"


@dataclass(frozen=True)
class OwnLight:
    """Perception's reading of the event's own light; state is UNKNOWN when none is fresh."""

    state: str = UNKNOWN
    arrow_active: bool = False


@dataclass(frozen=True)
class PhaseHint:
    """The signal phase tracker's estimate of one of the event's lights."""

    aligned: bool
    permitted_now: bool


@dataclass(frozen=True)
class Arrival:
    """The approach against the scheduled end of the own light's permission (module docstring)."""

    # Front bumper to the line [m] and measured speed [m/s].
    front_gap_m: float
    speed_mps: float
    # The stop-point machine would start its brake here on a red, or commit here on a permission
    # (commit distance): the last station where a normal stop is still possible.
    decision_point_reached: bool
    # Scheduled permission left now [s]; NaN without a trusted aligned schedule that permits.
    permission_left_sec: float = math.nan


def light_allows(light: OwnLight, arrow_maneuver: str) -> bool:
    """Arrow events go on a lit arrow that is not yellow; other events go on green."""
    if str(arrow_maneuver).upper() in {"LEFT", "RIGHT"}:
        arrow = light.arrow_active or light.state == ARROW
        return arrow and light.state != YELLOW
    return light.state == GREEN


class EventPermission:
    def __init__(
        self,
        *,
        confirm_sec: float = 0.4,
        lapse_hold_sec: float = 0.3,
        unknown_grace_sec: float = 0.3,
        escape_sec: float = 1.0,
        unknown_wait_limit_sec: float = 110.0,
        stopped_speed_mps: float = 0.1,
        arrival_margin_sec: float = 1.0,
    ) -> None:
        self.confirm_sec = max(0.0, confirm_sec)
        self.lapse_hold_sec = max(0.0, lapse_hold_sec)
        # Before the confirmation, an UNKNOWN this short pauses it instead of restarting it;
        # UNKNOWN itself never permits (2026-10-17 145703 F2: two UNKNOWN frames at 10 m/s
        # restarted the 0.4 s confirmation and the car braked on green). An arrow-off is a read
        # stop signal and still restarts it, since red is misread as red+arrow (F3 305).
        self.unknown_grace_sec = max(0.0, unknown_grace_sec)
        self.escape_sec = max(0.0, escape_sec)
        # Zero or less disables the stopped UNKNOWN wait limit.
        self.unknown_wait_limit_sec = float(unknown_wait_limit_sec)
        self.stopped_speed_mps = max(0.0, stopped_speed_mps)
        self.arrival_margin_sec = max(0.0, arrival_margin_sec)
        self.event_id: str | None = None
        self.reset()

    def reset(self) -> None:
        self.event_id = None
        self.green_since: float | None = None
        self.green_paused_since: float | None = None
        self.confirmed = False
        self.confirmed_this_approach = False
        self.lapse_since: float | None = None
        self.escape_since: float | None = None
        self.unknown_wait_since: float | None = None
        self.hold_since: float | None = None
        self.unknown_waited_sec: float | None = None
        # Phase of the previous signal tick, to see BRAKING turn HELD.
        self.last_phase: str | None = None
        # Arrival verdict of this approach (VERDICT_*), made on the first decision-point tick.
        self.decision_point_seen = False
        self.verdict: str | None = None
        self.verdict_reason = ""

    def update(
        self,
        now_sec: float,
        event: StopEvent,
        phase: str,
        *,
        s_rear: float,
        speed_mps: float,
        light: OwnLight | None = None,
        hint: PhaseHint | None = None,
        phase_forbids: bool = False,
        arrival: Arrival | None = None,
    ) -> Permission:
        """phase_forbids: the camera decides near a scheduled change and the aligned schedule
        of the own light does not permit (signal_schedule.SignalInput.schedule_forbids).
        arrival: the approach for the arrival verdict (module docstring); None makes none."""
        if event.event_id != self.event_id:
            self.reset()
            self.event_id = event.event_id
        stopped = math.isfinite(speed_mps) and abs(speed_mps) <= self.stopped_speed_mps
        if event.kind == KIND_TIMED:
            return self.timed(now_sec, event, phase, stopped)
        if event.kind == KIND_SIGNAL:
            permission = self.signal(
                now_sec, event, phase, s_rear, stopped, light or OwnLight(), hint, phase_forbids
            )
            return self.apply_verdict(phase, permission, arrival)
        return Permission()

    def apply_verdict(self, phase: str, permission: Permission, arrival: Arrival | None) -> Permission:
        """The arrival verdict on top of the light's permission, while approaching only."""
        if phase != WATCH:
            return permission
        if not self.decision_point_seen and arrival is not None and arrival.decision_point_reached:
            # The first decision-point tick decides, or never: later the stop is no longer normal.
            self.decision_point_seen = True
            if permission.go:
                self.verdict, self.verdict_reason = self.arrival_verdict(arrival)
        if self.verdict == VERDICT_PASS:
            return Permission(
                go=True, commit=True, reason=self.verdict_reason,
                tags=(*permission.tags, "SIGNAL_ARRIVAL_PASS"),
            )
        if self.verdict == VERDICT_STOP:
            return Permission(reason=self.verdict_reason, tags=(*permission.tags, "SIGNAL_ARRIVAL_STOP"))
        return permission

    def arrival_verdict(self, arrival: Arrival) -> tuple[str | None, str]:
        """PASS or STOP from the arrival time against the permission left; None without the end."""
        left = float(arrival.permission_left_sec)
        if not math.isfinite(left):
            return None, ""
        speed = abs(float(arrival.speed_mps))
        gap = max(0.0, float(arrival.front_gap_m))
        arrive = gap / speed if math.isfinite(speed) and speed > 1.0e-3 else math.inf
        if arrive + self.arrival_margin_sec <= left + 1.0e-6:
            return VERDICT_PASS, (
                f"arrival verdict pass: line in {arrive:.1f}s + margin {self.arrival_margin_sec:.1f}s, "
                f"permission left {left:.1f}s"
            )
        return VERDICT_STOP, (
            f"arrival verdict stop: line in {arrive:.1f}s + margin {self.arrival_margin_sec:.1f}s, "
            f"permission left only {left:.1f}s"
        )

    def timed(self, now_sec: float, event: StopEvent, phase: str, stopped: bool) -> Permission:
        if phase not in STOPPED_PHASES or not stopped:
            self.hold_since = None
            return Permission()
        if self.hold_since is None or now_sec < self.hold_since:
            self.hold_since = now_sec
        elapsed = now_sec - self.hold_since
        if elapsed + 1.0e-6 >= event.hold_sec:
            return Permission(release=True, reason=f"{event.hold_sec:.1f}s stop completed")
        return Permission(reason=f"holding timed stop {elapsed:.1f}/{event.hold_sec:.1f}s")

    def signal(
        self,
        now_sec: float,
        event: StopEvent,
        phase: str,
        s_rear: float,
        stopped: bool,
        light: OwnLight,
        hint: PhaseHint | None,
        phase_forbids: bool = False,
    ) -> Permission:
        raw_unknown = light.state == UNKNOWN
        stopped_past_line = phase in STOPPED_PHASES and stopped and s_rear >= event.line_s
        hint_used = raw_unknown and stopped_past_line and hint is not None and hint.aligned
        is_go = hint.permitted_now if hint_used else light_allows(light, event.arrow)
        # Stopped in BRAKING or HELD only: the approach, the yellow dilemma and a committed pass
        # are not changed. The go reading is dropped, so the confirm time starts again once the
        # tracker permits.
        phase_blocked = (
            phase in STOPPED_PHASES and stopped and not hint_used and is_go and phase_forbids
        )
        if phase_blocked:
            is_go = False
        # An arrow event whose arrow alone went dark while the circle lamp is still read.
        arrow_dropped = (
            str(event.arrow).upper() in {"LEFT", "RIGHT"}
            and light.state in {RED, GREEN}
            and not light.arrow_active
        )
        lapsed = (raw_unknown and not hint_used) or arrow_dropped
        if not lapsed:
            self.lapse_since = None
        elif self.lapse_since is None or now_sec < self.lapse_since:
            self.lapse_since = now_sec
        if phase == HELD and self.last_phase == BRAKING:
            # The hard brake ended in a stop: a permission confirmed while braking does not launch
            # the car; standing, it is confirmed again from this tick (2026-10-17 14:57 F3).
            self.green_since = None
            self.confirmed = False
            self.confirmed_this_approach = False
        self.last_phase = phase
        permission_before = self.confirmed_this_approach
        held_as_go = (
            lapsed
            and permission_before
            and now_sec - self.lapse_since < self.lapse_hold_sec - 1.0e-6
            and not phase_blocked
        )
        pending_go_held = (
            raw_unknown
            and not hint_used
            and not permission_before
            and self.green_since is not None
            and now_sec - self.lapse_since < self.unknown_grace_sec - 1.0e-6
        )
        if pending_go_held and self.green_paused_since is None:
            self.green_paused_since = self.lapse_since
        dilemma = permission_before and light.state in {YELLOW, UNKNOWN} and not is_go

        if is_go:
            paused, self.green_paused_since = self.green_paused_since, None
            if paused is not None and now_sec - paused >= self.unknown_grace_sec - 1.0e-6:
                self.green_since = None  # the pause outlasted the grace between ticks
            if self.green_since is None or now_sec < self.green_since:
                self.green_since = now_sec
            self.confirmed = now_sec - self.green_since >= self.confirm_sec - 1.0e-6
            if self.confirmed:
                self.confirmed_this_approach = True
        elif not held_as_go and not pending_go_held:
            self.green_since = None
            self.green_paused_since = None
            self.confirmed = False
            if not raw_unknown:
                self.confirmed_this_approach = False
        go = self.confirmed and (is_go or held_as_go)

        tags = ["SIGNAL_PHASE_TRACKER_USED"] if hint_used else []
        release, reason = go, "permitted signal confirmed" if go else ""
        if phase_blocked:
            tags.append("SIGNAL_PHASE_TRACKER_BLOCK")
            reason = "own light reads go; the schedule does not permit near a change or its permission ends too soon"
        escape = self.escape_ready(now_sec, event, phase, s_rear, stopped, raw_unknown and not hint_used)
        waited = self.unknown_wait(now_sec, phase, stopped, raw_unknown)
        if not release and escape:
            release, reason = True, "stopped beyond the stop line with the own light unavailable"
        if not release and waited is not None and waited >= self.unknown_wait_limit_sec - 1.0e-6:
            release = True
            reason = f"own light UNKNOWN for {waited:.1f}s while stopped; departing at wait limit"
            tags.append("SIGNAL_UNKNOWN_TIMEOUT")
            self.unknown_waited_sec = waited
        return Permission(go=go, dilemma=dilemma, release=release, reason=reason, tags=tuple(tags))

    def escape_ready(self, now_sec, event, phase, s_rear, stopped, unknown) -> bool:
        if phase not in STOPPED_PHASES or not unknown or not stopped or s_rear < event.line_s:
            self.escape_since = None
            return False
        if self.escape_since is None or now_sec < self.escape_since:
            self.escape_since = now_sec
        return now_sec - self.escape_since >= self.escape_sec - 1.0e-6

    def unknown_wait(self, now_sec, phase, stopped, raw_unknown) -> float | None:
        """Seconds stopped at the line seeing only UNKNOWN; any valid signal restarts it."""
        if (
            self.unknown_wait_limit_sec <= 0.0
            or phase not in STOPPED_PHASES
            or not stopped
            or not raw_unknown
        ):
            self.unknown_wait_since = None
            return None
        if self.unknown_wait_since is None or now_sec < self.unknown_wait_since:
            self.unknown_wait_since = now_sec
        return now_sec - self.unknown_wait_since
