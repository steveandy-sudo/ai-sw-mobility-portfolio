"""Timed world script of a scenario: light states, dropouts, obstacles.

A timeline is a list of steps. The first step applies at the start unless it has a trigger;
every later step waits for its trigger, armed when the previous step fired. Triggers (exactly
one key per step, all optional for the first step):

  at_s: T             scenario time T
  after_s: T          T seconds after the previous step fired
  stopped_for_s: T    the vehicle has stood still (|v| <= 0.05 m/s) for T seconds
  moving_for_s: T     the vehicle has moved (|v| > 0.3 m/s) for T seconds
  front_gap_le: D     front bumper at most D metres before the line of `event` (the scenario event
                      unless the step names one); negative D means past the line
  route_s_ge: S       rear axle route progress at least S

Light step values: `lamps: [red|yellow|green|arrow, ...]` (an empty list is a dark head, which
pv26 reports as UNKNOWN), `unseen: true` (the light is not in the observation array), or
`cycle: [{lamps: [...], s: T} | {unseen: true, s: T}, ...]` repeated until the next step.

Phase step values (`event_phases`, what the signal phase tracker reports for every own light of
the scenario event): `{aligned: true|false, permitted: true|false}`, optionally with
`until_change_s: T` (phase_remaining_sec and permission_change_sec, default 0: always within the
schedule margin, so the camera decides; a permission with less than 4 s left never launches a
stopped car, so a permitted step that should launch one gives a longer time; with `counting: true`
it counts down from the step's firing to 0, as the tracker's estimate does), `lamps: [...]`
(predicted_lamps) and `probability: P`
(permission_probability, default 1 when permitted and 0 otherwise), or
`unpublished: true` (no phase message). The harness publishes it in place of the tracker node,
which it leaves out. A `dropouts` entry on topic `signal_phases` also stops it.
"""
from __future__ import annotations

from dataclasses import dataclass, field

TRIGGER_KEYS = ("at_s", "after_s", "stopped_for_s", "moving_for_s", "front_gap_le", "route_s_ge")
LAMP_NAMES = ("red", "yellow", "green", "arrow")
STOPPED_MPS = 0.05
MOVING_MPS = 0.3


@dataclass
class Truth:
    """What the triggers and checks may know about the vehicle at one instant."""

    t: float
    speed: float
    route_s: float
    front_s: float
    stopped_since: float | None
    moving_since: float | None

    def front_gap(self, line_s: float) -> float:
        return line_s - self.front_s


class Trigger:
    def __init__(self, step: dict, default_event: str | None, course) -> None:
        keys = [key for key in TRIGGER_KEYS if key in step]
        if len(keys) > 1:
            raise ValueError(f"a step takes one trigger, got {keys}: {step}")
        self.key = keys[0] if keys else None
        self.value = float(step[self.key]) if self.key else 0.0
        self.line_s = None
        if self.key == "front_gap_le":
            event_id = step.get("event", default_event)
            if event_id is None:
                raise ValueError(f"front_gap_le needs an event: {step}")
            self.line_s = course.event(event_id).line_s

    def fired(self, truth: Truth, armed_at: float) -> bool:
        key, value = self.key, self.value
        if key is None:
            return True
        if key == "at_s":
            return truth.t >= value - 1.0e-9
        if key == "after_s":
            return truth.t - armed_at >= value - 1.0e-9
        if key == "stopped_for_s":
            return truth.stopped_since is not None and truth.t - truth.stopped_since >= value - 1.0e-9
        if key == "moving_for_s":
            return truth.moving_since is not None and truth.t - truth.moving_since >= value - 1.0e-9
        if key == "front_gap_le":
            return truth.front_gap(self.line_s) <= value
        if key == "route_s_ge":
            return truth.route_s >= value
        raise AssertionError(key)


@dataclass
class LightView:
    visible: bool
    lamps: frozenset


def _view(item: dict) -> LightView:
    if item.get("unseen"):
        return LightView(False, frozenset())
    lamps = [str(name).lower() for name in item.get("lamps", [])]
    unknown = [name for name in lamps if name not in LAMP_NAMES]
    if unknown:
        raise ValueError(f"unknown lamp names {unknown}")
    return LightView(True, frozenset(lamps))


class Timeline:
    def __init__(self, steps: list[dict], default_event: str | None, course) -> None:
        if not steps:
            raise ValueError("a timeline needs at least one step")
        self.steps = steps
        self.triggers = [Trigger(step, default_event, course) for step in steps]
        self.index = -1
        self.fired_at = 0.0
        self.armed_at = 0.0
        self.fire_times: list[float | None] = [None] * len(steps)

    def update(self, truth: Truth) -> None:
        while self.index + 1 < len(self.steps):
            nxt = self.index + 1
            if not self.triggers[nxt].fired(truth, self.armed_at):
                break
            self.index = nxt
            self.fired_at = truth.t
            self.armed_at = truth.t
            self.fire_times[nxt] = truth.t

    @property
    def current(self) -> dict | None:
        return self.steps[self.index] if self.index >= 0 else None


class LightScript(Timeline):
    def __init__(self, light_id: str, steps: list[dict], default_event: str | None, course) -> None:
        super().__init__(steps, default_event, course)
        self.light_id = light_id
        for step in steps:
            for item in step.get("cycle", []) or [step]:
                _view(item)

    def view(self, t: float) -> LightView:
        step = self.current
        if step is None:
            return LightView(False, frozenset())
        cycle = step.get("cycle")
        if not cycle:
            return _view(step)
        period = sum(float(item["s"]) for item in cycle)
        phase = (t - self.fired_at) % period
        for item in cycle:
            phase -= float(item["s"])
            if phase < -1.0e-9:
                return _view(item)
        return _view(cycle[-1])


@dataclass
class PhaseView:
    published: bool
    aligned: bool = False
    permitted: bool = False
    until_change_s: float = 0.0
    lamps: tuple[str, ...] = ()
    probability: float = 0.0


def _phase_view(step: dict, elapsed_s: float = 0.0) -> PhaseView:
    """elapsed_s: time since the step fired, for `counting` steps."""
    if step.get("unpublished"):
        return PhaseView(False)
    for key in ("aligned", "permitted"):
        if not isinstance(step.get(key), bool):
            raise ValueError(f"a phase step needs {key}: true|false or unpublished: true, got {step}")
    lamps = tuple(str(name) for name in step.get("lamps", ()))
    if any(name not in LAMP_NAMES for name in lamps):
        raise ValueError(f"phase lamps must be among {LAMP_NAMES}: {step}")
    probability = float(step.get("probability", 1.0 if step["permitted"] else 0.0))
    until = float(step.get("until_change_s", 0.0))
    if step.get("counting"):
        until = max(0.0, until - max(0.0, elapsed_s))
    return PhaseView(True, step["aligned"], step["permitted"], until, lamps, probability)


class PhaseScript(Timeline):
    def __init__(self, event, steps: list[dict], course) -> None:
        super().__init__(steps, event.event_id, course)
        self.event = event
        for step in steps:
            _phase_view(step)

    def view(self, t: float | None = None) -> PhaseView:
        step = self.current
        if step is None:
            return PhaseView(False)
        return _phase_view(step, 0.0 if t is None else t - self.fired_at)


PERCEPTION_TOPICS = ("road_segments", "centerline", "objects", "traffic_lights")
DROPOUT_GROUPS = {
    "perception": PERCEPTION_TOPICS,
    "localization": ("odometry", "tf"),
}
# signal_phases: the harness-published phase estimate of `event_phases`.
DROPOUT_TOPICS = (*PERCEPTION_TOPICS, "odometry", "tf", "vehicle_state", "signal_phases")


@dataclass
class Dropout:
    topics: tuple[str, ...]
    start: Trigger
    duration_s: float | None
    started_at: float | None = None
    ended_at: float | None = None

    def update(self, truth: Truth) -> None:
        if self.started_at is None:
            if self.start.fired(truth, 0.0):
                self.started_at = truth.t
            return
        if self.ended_at is None and self.duration_s is not None and truth.t - self.started_at >= self.duration_s - 1.0e-9:
            self.ended_at = truth.t

    def active(self) -> bool:
        return self.started_at is not None and self.ended_at is None


def make_dropout(item: dict, default_event: str | None, course) -> Dropout:
    names = item.get("topics", [])
    if isinstance(names, str):
        names = [names]
    topics: list[str] = []
    for name in names:
        expanded = DROPOUT_GROUPS.get(name, (name,))
        for topic in expanded:
            if topic not in DROPOUT_TOPICS:
                raise ValueError(f"unknown dropout topic [{topic}]; known {DROPOUT_TOPICS} and groups {sorted(DROPOUT_GROUPS)}")
            topics.append(topic)
    start = Trigger(item.get("start", {}) or {}, default_event, course)
    duration = item.get("duration_s")
    return Dropout(tuple(topics), start, None if duration is None else float(duration))


@dataclass
class Drum:
    x: float
    y: float
    diameter_m: float
    height_m: float
    row: int


@dataclass
class HeadingFlip:
    """The localization heading reads 180 deg off from `start` until `end` fires or `duration_s`."""

    start: Trigger
    end: Trigger | None
    duration_s: float | None
    started_at: float | None = None
    ended_at: float | None = None

    def update(self, truth: Truth) -> None:
        if self.started_at is None:
            if self.start.fired(truth, 0.0):
                self.started_at = truth.t
            return
        if self.ended_at is None and (
            (self.duration_s is not None and truth.t - self.started_at >= self.duration_s - 1.0e-9)
            or (self.end is not None and self.end.fired(truth, self.started_at))
        ):
            self.ended_at = truth.t

    def active(self) -> bool:
        return self.started_at is not None and self.ended_at is None


@dataclass
class World:
    lights: dict[str, LightScript] = field(default_factory=dict)
    dropouts: list[Dropout] = field(default_factory=list)
    heading_flips: list[HeadingFlip] = field(default_factory=list)
    drums: list[Drum] = field(default_factory=list)
    rows: list[dict] = field(default_factory=list)
    phases: PhaseScript | None = None

    def update(self, truth: Truth) -> None:
        for script in self.lights.values():
            script.update(truth)
        if self.phases is not None:
            self.phases.update(truth)
        for dropout in self.dropouts:
            dropout.update(truth)
        for flip in self.heading_flips:
            flip.update(truth)

    def dropped(self, topic: str) -> bool:
        return any(topic in dropout.topics and dropout.active() for dropout in self.dropouts)

    def heading_flipped(self) -> bool:
        return any(flip.active() for flip in self.heading_flips)


def make_world(spec: dict, default_event: str | None, course) -> World:
    """`other_lights` scripts the lights of every course event except the scenario event,
    `event_lights` the lights of the scenario event, `lights` single lights by map id (wins),
    `event_phases` the phase estimate of the scenario event's lights, `heading_flips` windows
    `{start: <trigger>, end: <trigger> | duration_s: T}` in which the localization heading reads
    180 deg off (odometry and TF; the odometry forward speed turns negative with it)."""
    world = World()
    scripts: dict[str, list] = {}
    own = set(course.event(default_event).light_ids) if default_event else set()
    if spec.get("event_lights") is not None:
        if not own:
            raise ValueError("event_lights needs a scenario event with traffic lights")
        scripts.update({light_id: spec["event_lights"] for light_id in own})
    if spec.get("other_lights") is not None:
        for event in course.events:
            for light_id in event.light_ids:
                if light_id not in own:
                    scripts.setdefault(light_id, spec["other_lights"])
    scripts.update(spec.get("lights") or {})
    for light_id, steps in scripts.items():
        if light_id not in course.lights:
            raise ValueError(f"unknown light [{light_id}]")
        world.lights[light_id] = LightScript(light_id, [dict(step) for step in steps], default_event, course)
    if spec.get("event_phases") is not None:
        if not own:
            raise ValueError("event_phases needs a scenario event with traffic lights")
        world.phases = PhaseScript(course.event(default_event), [dict(step) for step in spec["event_phases"]], course)
    for item in spec.get("dropouts") or []:
        world.dropouts.append(make_dropout(item, default_event, course))
    for item in spec.get("heading_flips") or []:
        duration = item.get("duration_s")
        world.heading_flips.append(HeadingFlip(
            Trigger(item.get("start", {}) or {}, default_event, course),
            Trigger(item["end"], default_event, course) if item.get("end") else None,
            None if duration is None else float(duration),
        ))
    for index, row in enumerate(spec.get("obstacle_rows") or []):
        count = int(row.get("count", 3))
        diameter = float(row.get("diameter_m", 0.5))
        height = float(row.get("height_m", 0.8))
        center = float(row["center_lateral_m"])
        route_s = float(row["route_s"])
        for k in range(count):
            lateral = center + (k - (count - 1) / 2.0) * diameter
            x, y, _ = course.route.pose_at(route_s, lateral)
            world.drums.append(Drum(x, y, diameter, height, index))
        world.rows.append({"index": index, "route_s": route_s, "center_lateral_m": center,
                           "half_width_m": count * diameter / 2.0})
    return world
