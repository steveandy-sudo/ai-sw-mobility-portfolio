"""Signal input of a SIGNAL_OBEY event: the wall-clock phase schedule, with the camera as a veto.

K-City runs every signal controller on the UTC 100 s grid (docs/reports/2026-10-17
_signal_phase_wallclock), so an aligned, fresh phase estimate of the event's own light says what
the light shows. The owner (kgm, 2026-10-17) chose to drive on it: the signal logic reads the
schedule state in place of the camera reading and keeps its own confirmation rules. The camera
only rejects the schedule:

- Near a scheduled change (transition_margin_sec before or after a lamp or permission change of
  the own light) the camera decides as before, and a schedule that does not permit, or permits
  for less than launch_min_remaining_sec, still withholds a launch from a stop (schedule_forbids).
- The schedule permits but the camera reads a sure opposite for reject_frames frames and
  reject_sec within that permission: the camera decides until the scheduled permission changes,
  which also takes back a confirmed permission that has not committed the pass.
- The camera keeps contradicting the schedule (a sure opposite for distrust_opposite_sec while it
  permits, or a sure go for distrust_go_sec while it does not): the schedule is dropped for the
  rest of the approach, in case the signal operation changed on the day.

The time since the last scheduled change comes from the tracker's estimate stream
(ScheduleChangeClock, ScheduleState.elapsed_sec), so an approach that first sees the schedule in
the middle of a phase drives on it at once. Without that time the first sight counts as a change
(2026-10-17 14:57 F2: green with 7.7 s left, the camera decided the first second, UNKNOWN frames
broke the 0.4 s confirmation and the car braked hard on green).

A reading is sure only when its circle is one the scheduled lamps do not light. A sure opposite
is a red or yellow circle, read with the arrow off at an arrow event; a sure go is a reading that
permits the event (green, or the arrow at an arrow event, whose circle is red when none is read).
A missing or an extra arrow over the scheduled circle is never sure: at K-City 375 the red stays
lit through the arrow phase and 36 m away pv26 read red with the arrow off for 5.6 s on end while
the arrow was lit (2026-10-17 103503), and pv26 reads false arrows over a lit red (Gazebo 375) or
green (305 close up). UNKNOWN frames neither extend nor end a run; any other frame ends it.

Two rules hold while the car stands at the event (stopped_at_line), from the real-car change of
2026-10-17 (kty 17d49de) that first let the estimate supply a permission pv26 missed:

- A scheduled permission launches the car only when it is sure: permission probability at least
  launch_min_probability (0.9) and the permission lasting at least launch_min_remaining_sec (4 s).
  Otherwise the camera decides (UNSURE); a schedule that does not permit still holds the car,
  and so does a permission ending within launch_min_remaining_sec, also near a change and on a
  rejected schedule (2026-10-17 14:57 F3: the arrow had 0.4 s left at the stop, and the launch
  takes 3.2 s to the line).
- The schedule holds back a permission the camera reads for at most withhold_limit_sec (100 s,
  one program cycle) per stop, counted from the first such tick; then the camera decides for the
  rest of the approach (WITHHOLD_LIMIT), as with distrust.

Runs are counted on distinct observation stamps. Nothing here stops the car; it only chooses the
input the existing permission logic reads. No rclpy here, so the logic is unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from kaiev26_decision.signal_phase_tracker import (
    LAMP_GREEN,
    LAMP_RED,
    LAMP_YELLOW,
    TRAFFIC_ARROW,
    TRAFFIC_GREEN,
    TRAFFIC_RED,
    TRAFFIC_UNKNOWN,
    TRAFFIC_YELLOW,
    lamps_permit,
    lamps_scene_state,
)


SOURCE_SCHEDULE = "SCHEDULE"
SOURCE_CAMERA = "CAMERA"
SOURCE_MARGIN = "MARGIN"
SOURCE_REJECT = "REJECT"
SOURCE_DISTRUST = "DISTRUST"
SOURCE_UNSURE = "UNSURE"
SOURCE_LIMIT = "WITHHOLD_LIMIT"

FRAME_GO = "GO"
FRAME_OPPOSITE = "OPPOSITE"
FRAME_UNKNOWN = "UNKNOWN"
FRAME_OTHER = "OTHER"

# Circle lamp of a read state; ARROW is red and arrow (or an arrow alone), so red.
_CIRCLE_LAMP = {
    TRAFFIC_RED: LAMP_RED,
    TRAFFIC_YELLOW: LAMP_YELLOW,
    TRAFFIC_GREEN: LAMP_GREEN,
    TRAFFIC_ARROW: LAMP_RED,
}


@dataclass(frozen=True)
class ScheduleConfig:
    # No schedule this close to a scheduled change; theta varies about +-0.5 s between runs.
    transition_margin_sec: float = 1.0
    # A sure opposite rejects a scheduled permission after both; 0.5 s fits inside the margin, so a
    # wrong permission at a stop is rejected before the 0.4 s confirmation can launch.
    reject_frames: int = 10
    reject_sec: float = 0.5
    distrust_opposite_sec: float = 3.0
    distrust_go_sec: float = 2.0
    # Stopped at the event, a scheduled permission launches only this sure and lasting this long.
    launch_min_probability: float = 0.9
    # 4 s: the real launch takes 3.2 s to the line (2026-10-17 14:57) plus 0.5 s of theta error.
    launch_min_remaining_sec: float = 4.0
    # Longest the schedule holds back a permission the camera reads, per stop. Zero: no limit.
    withhold_limit_sec: float = 100.0

    def __post_init__(self) -> None:
        for name in ("transition_margin_sec", "reject_sec", "distrust_opposite_sec", "distrust_go_sec",
                     "launch_min_remaining_sec", "withhold_limit_sec"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and not negative")
        if self.reject_frames < 1:
            raise ValueError("reject_frames must be at least 1")


@dataclass(frozen=True)
class ScheduleState:
    """An aligned, fresh phase estimate of the event's own light."""

    permitted: bool
    # Predicted lamps (LAMP_* bits); 0 when unknown.
    lamps: int = 0
    # Time to the next scheduled lamp or permission change [s]; NaN when unknown.
    until_change_sec: float = math.nan
    # Probability that the light permits now, and time to the next permission change [s]; NaN
    # when unknown (then a scheduled permission never launches the car from a stop).
    probability: float = math.nan
    permission_change_sec: float = math.nan
    # Time since the tracker's estimate took this permission and these lamps [s]; NaN when
    # unknown (then the first sight counts as a change).
    elapsed_sec: float = math.nan


class ScheduleChangeClock:
    """When each light's aligned estimate took its present (permitted, lamps), from the stream of
    estimates. Unknown until a change is seen: on the first estimate, after a gap longer than
    gap_sec or a clock jump back, and while the estimate is not aligned."""

    def __init__(self, gap_sec: float) -> None:
        self.gap_sec = float(gap_sec)
        # light id -> (key, since or None, last stamp)
        self.lights: dict[str, tuple[tuple[bool, int], float | None, float]] = {}

    def observe(self, stamp_sec: float, light_id: str, aligned: bool, key: tuple[bool, int]) -> None:
        if not aligned:
            self.lights.pop(light_id, None)
            return
        previous = self.lights.get(light_id)
        if (
            previous is None
            or stamp_sec < previous[2]
            or stamp_sec - previous[2] > self.gap_sec
        ):
            self.lights[light_id] = (key, None, stamp_sec)
            return
        since = previous[1] if key == previous[0] else stamp_sec
        self.lights[light_id] = (key, since, stamp_sec)

    def elapsed(self, light_id: str, key: tuple[bool, int], now_sec: float) -> float:
        entry = self.lights.get(light_id)
        if entry is None or entry[0] != key or entry[1] is None:
            return math.nan
        return max(0.0, now_sec - entry[1])


@dataclass(frozen=True)
class SignalInput:
    state: int
    arrow_active: bool
    source: str
    # Camera input while the schedule does not permit near a scheduled change, or permits for
    # less than launch_min_remaining_sec at a stop: a launch from a stop is withheld.
    schedule_forbids: bool = False
    reason: str = ""
    # REJECT or DISTRUST on the tick it began, for one log line.
    started: str = ""

    @property
    def tag(self) -> str:
        return f"SIGNAL_INPUT_{self.source}"


def is_arrow_event(maneuver: str) -> bool:
    return str(maneuver).upper() in {"LEFT", "RIGHT"}


def camera_permits(state: int, arrow_active: bool, maneuver: str) -> bool:
    """The camera reading permits the event: a lit arrow that is not yellow at an arrow event,
    green otherwise."""
    state = int(state)
    if is_arrow_event(maneuver):
        return (bool(arrow_active) or state == TRAFFIC_ARROW) and state != TRAFFIC_YELLOW
    return state == TRAFFIC_GREEN


def sure_permission(schedule: ScheduleState, config: ScheduleConfig) -> bool:
    """A scheduled permission sure and lasting enough to launch the car from a stop."""
    probability = float(schedule.probability)
    remaining = float(schedule.permission_change_sec)
    return (
        bool(schedule.permitted)
        and math.isfinite(probability)
        and probability >= config.launch_min_probability
        and math.isfinite(remaining)
        and remaining >= config.launch_min_remaining_sec
    )


def frame_kind(state: int, arrow_active: bool, arrow_event: bool, scheduled_lamps: int = 0) -> str:
    """A sure GO or a sure OPPOSITE against scheduled_lamps (module docstring), UNKNOWN or OTHER."""
    state = int(state)
    if state == TRAFFIC_UNKNOWN:
        return FRAME_UNKNOWN
    circle = _CIRCLE_LAMP.get(state, 0)
    if int(scheduled_lamps) & circle:
        return FRAME_OTHER
    if arrow_event:
        if (arrow_active or state == TRAFFIC_ARROW) and state != TRAFFIC_YELLOW:
            return FRAME_GO
        if state in {TRAFFIC_RED, TRAFFIC_YELLOW}:
            return FRAME_OPPOSITE
        return FRAME_OTHER
    if state == TRAFFIC_GREEN:
        return FRAME_GO
    if state in {TRAFFIC_RED, TRAFFIC_YELLOW}:
        return FRAME_OPPOSITE
    return FRAME_OTHER


def schedule_scene_state(schedule: ScheduleState, maneuver: str) -> tuple[int, bool]:
    """SceneSummary state and arrow flag the schedule stands for at this event."""
    if schedule.permitted:
        return (TRAFFIC_ARROW, True) if is_arrow_event(maneuver) else (TRAFFIC_GREEN, False)
    lamps = int(schedule.lamps)
    if lamps and not lamps_permit(lamps, maneuver):
        state, arrow = lamps_scene_state(lamps)
        if state != TRAFFIC_UNKNOWN:
            return state, arrow
    return TRAFFIC_RED, False


class _Run:
    """Frames of one kind with only UNKNOWN between them, counted from max(first frame, last
    schedule change)."""

    def __init__(self) -> None:
        self.since: float | None = None
        self.frames = 0

    def clear(self) -> None:
        self.since = None
        self.frames = 0

    def add(self, now_sec: float) -> None:
        if self.since is None:
            self.since = now_sec
        self.frames += 1

    def restart_at(self, now_sec: float) -> None:
        if self.since is not None:
            self.since = now_sec
            self.frames = 0

    def seconds(self, now_sec: float) -> float:
        return now_sec - self.since if self.since is not None else 0.0


class SignalSourceSelector:
    def __init__(self, config: ScheduleConfig | None = None) -> None:
        self.config = config or ScheduleConfig()
        self.reset()

    def reset(self) -> None:
        self.event_id: str | None = None
        self.last_frame_stamp: float | None = None
        self.opposite = _Run()
        self.go = _Run()
        self.schedule_key: tuple[bool, int] | None = None
        self.schedule_changed_sec: float | None = None
        self.rejected = False
        self.distrusted = False
        self.distrust_reason = ""
        self.dropped_source = SOURCE_DISTRUST
        # First tick of this stop on which the schedule held back a permission the camera read.
        self.withhold_since: float | None = None

    def _frame(self, now_sec: float, kind: str, frame_stamp: float | None) -> None:
        new_frame = frame_stamp is None or frame_stamp != self.last_frame_stamp
        self.last_frame_stamp = frame_stamp
        if kind == FRAME_UNKNOWN:
            return
        if kind != FRAME_OPPOSITE:
            self.opposite.clear()
        if kind != FRAME_GO:
            self.go.clear()
        if not new_frame:
            return
        if kind == FRAME_OPPOSITE:
            self.opposite.add(now_sec)
        elif kind == FRAME_GO:
            self.go.add(now_sec)

    def update(
        self,
        now_sec: float,
        event_id: str,
        maneuver: str,
        camera_state: int,
        camera_arrow: bool,
        schedule: ScheduleState | None,
        frame_stamp: float | None = None,
        stopped_at_line: bool = False,
    ) -> SignalInput:
        """Input for this tick. schedule: an aligned, fresh estimate of the own light, else None.
        frame_stamp: stamp of the camera observation behind camera_state (None counts every tick).
        stopped_at_line: the car stands at this event waiting for its release (module docstring)."""
        if event_id != self.event_id:
            self.reset()
            self.event_id = event_id
        config = self.config

        def camera(source: str, reason: str, forbids: bool = False, started: str = "") -> SignalInput:
            return SignalInput(int(camera_state), bool(camera_arrow), source, forbids, reason, started)

        if schedule is None:
            self.schedule_key = None
            return camera(SOURCE_CAMERA, "signal input: camera (no aligned signal schedule)")
        if self.distrusted:
            return camera(self.dropped_source, f"signal input: camera, schedule distrusted ({self.distrust_reason})")

        key = (bool(schedule.permitted), int(schedule.lamps))
        if key != self.schedule_key:
            if self.schedule_key is None or key[0] != self.schedule_key[0]:
                self.rejected = False
            self.schedule_key = key
            # The tracker's time since the change; unknown, the first sight counts as a change.
            elapsed = float(schedule.elapsed_sec)
            self.schedule_changed_sec = now_sec - (elapsed if math.isfinite(elapsed) and elapsed > 0.0 else 0.0)
            self.opposite.restart_at(now_sec)
            self.go.restart_at(now_sec)
        kind = frame_kind(camera_state, camera_arrow, is_arrow_event(maneuver), schedule.lamps)
        self._frame(now_sec, kind, frame_stamp)
        since_change = now_sec - self.schedule_changed_sec
        until_change = float(schedule.until_change_sec)
        margin = config.transition_margin_sec
        near_change = since_change < margin or not (until_change > margin)

        started = ""
        if not near_change:
            if schedule.permitted and self.opposite.seconds(now_sec) > config.distrust_opposite_sec:
                self.distrust_reason = (
                    f"camera read the opposite for {self.opposite.seconds(now_sec):.1f}s "
                    "while the schedule permits"
                )
            elif not schedule.permitted and self.go.seconds(now_sec) > config.distrust_go_sec:
                self.distrust_reason = (
                    f"camera read go for {self.go.seconds(now_sec):.1f}s "
                    "while the schedule does not permit"
                )
            if self.distrust_reason:
                self.distrusted = True
                return camera(
                    SOURCE_DISTRUST,
                    f"signal input: camera, schedule distrusted ({self.distrust_reason})",
                    started=SOURCE_DISTRUST,
                )
            if (
                schedule.permitted
                and not self.rejected
                and self.opposite.frames >= config.reject_frames
                and self.opposite.seconds(now_sec) >= config.reject_sec - 1.0e-6
            ):
                self.rejected = True
                started = SOURCE_REJECT
        if not stopped_at_line:
            self.withhold_since = None
        elif (
            self.withhold_since is None
            and not schedule.permitted
            and camera_permits(camera_state, camera_arrow, maneuver)
        ):
            self.withhold_since = now_sec
        limit = config.withhold_limit_sec
        if (
            limit > 0.0
            and self.withhold_since is not None
            and now_sec - self.withhold_since >= limit - 1.0e-6
        ):
            self.distrusted = True
            self.dropped_source = SOURCE_LIMIT
            self.distrust_reason = f"it held back the camera's permission for {limit:.0f}s at this stop"
            return camera(
                SOURCE_LIMIT,
                f"signal input: camera, schedule distrusted ({self.distrust_reason})",
                started=SOURCE_LIMIT,
            )
        # Stopped at the event, a scheduled permission ending within launch_min_remaining_sec
        # withholds the launch whoever decides: the car would reach the line after it ends.
        remaining = float(schedule.permission_change_sec)
        permission_ending = (
            stopped_at_line
            and schedule.permitted
            and math.isfinite(remaining)
            and remaining < config.launch_min_remaining_sec
        )
        if schedule.permitted and self.rejected:
            return camera(
                SOURCE_REJECT,
                "signal input: camera, a sure opposite reading rejected the scheduled permission",
                forbids=permission_ending,
                started=started,
            )
        if near_change:
            return camera(
                SOURCE_MARGIN,
                "signal input: camera near a scheduled change",
                forbids=not schedule.permitted or permission_ending,
            )
        if stopped_at_line and schedule.permitted and not sure_permission(schedule, config):
            return camera(
                SOURCE_UNSURE,
                "signal input: camera, the scheduled permission is not sure enough to launch",
                forbids=permission_ending,
            )
        state, arrow = schedule_scene_state(schedule, maneuver)
        return SignalInput(
            state,
            arrow,
            SOURCE_SCHEDULE,
            reason=(
                "signal input: schedule "
                + ("permits" if schedule.permitted else "does not permit")
            ),
        )
