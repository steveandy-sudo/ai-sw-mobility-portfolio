"""Map-wide traffic-signal phase tracking on the map signal programs.

Every alignment group (one controller by default) has one unknown cycle alignment theta in
[0, period): a light of program k shows the program lamps at (t + offset_k + theta) mod period,
as Gazebo's traffic_light_system plays them. The tracker keeps a discrete log-likelihood over
theta, updates it with the round lamps (red, yellow, green) the frames of every light in the
group show and predicts each light's lamps and remaining phase time by running the cycle
forward, also for lights out of view. Arrow readings are no evidence for or against an
alignment: pv26 reads a red-only head as red and arrow, and that pulled the estimate onto the
arrow phase (owner, 2026-10-16). The arrow lamps and the arrow permission therefore come from
the program at the round-lamp alignment. Clean observed phase edges are compared with the
program as duration evidence; the program itself is never rewritten.

An optional wall-clock prior (TrackerConfig.prior_std_sec) puts a wrapped normal over every
group's theta. K-City starts every controller's cycle on the UTC 100 s grid: theta stayed within
-0.7..+0.85 s at six intersections over 42 days, varying about +-0.5 s between runs without a
drift (2026-10-17 wall-clock report). The prior is kept apart from the evidence, so forgetting
and the log-odds floor never erase it, and it does not widen with time. With it a group is
aligned before its first observation.

No rclpy here, so the core is unit-testable; signal_phase_tracker_node.py owns ROS I/O.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Iterable, Mapping

import numpy as np
import yaml

from kaiev26_decision.zone_policy import ZonePolicy


# SceneSummary.TRAFFIC_* codes, the gateway's interpretation of one observation. Mirrored to
# keep rclpy out of the core; test_signal_phase_tracker checks them against the message.
TRAFFIC_UNKNOWN = 0
TRAFFIC_RED = 1
TRAFFIC_YELLOW = 2
TRAFFIC_GREEN = 3
TRAFFIC_ARROW = 4

# TrafficSignalPhase.LAMP_* bits.
LAMP_RED = 1
LAMP_YELLOW = 2
LAMP_GREEN = 4
LAMP_ARROW = 8
LAMP_BITS = {"red": LAMP_RED, "yellow": LAMP_YELLOW, "green": LAMP_GREEN, "arrow": LAMP_ARROW}

COUPLING_MODES = ("program", "controller", "all")
_CIRCLE_BY_STATE = {TRAFFIC_RED: "RED", TRAFFIC_YELLOW: "YELLOW", TRAFFIC_GREEN: "GREEN"}


def lamps_from_names(names: Iterable[str]) -> int:
    lamps = 0
    for name in names:
        bit = LAMP_BITS.get(str(name).strip().lower())
        if bit is None:
            raise ValueError(f"unknown traffic-light lamp [{name}]")
        lamps |= bit
    return lamps


def lamps_name(lamps: int) -> str:
    names = [name.upper() for name, bit in LAMP_BITS.items() if int(lamps) & bit]
    return "+".join(names) if names else "OFF"


def lamps_circle(lamps: int) -> str | None:
    """Circle colour a viewer reports, with the gateway's precedence (yellow, then red)."""
    if lamps & LAMP_YELLOW:
        return "YELLOW"
    if lamps & LAMP_RED:
        return "RED"
    if lamps & LAMP_GREEN:
        return "GREEN"
    return None


def lit_circles(lamps: int) -> frozenset[str]:
    """Every circle colour that is lit. A viewer may report any of them: K-City 375 shows red and
    yellow together while its left arrow clears, and PV26 then reads red."""
    names = set()
    if lamps & LAMP_RED:
        names.add("RED")
    if lamps & LAMP_YELLOW:
        names.add("YELLOW")
    if lamps & LAMP_GREEN:
        names.add("GREEN")
    return frozenset(names)


def lamps_scene_state(lamps: int) -> tuple[int, bool]:
    """SceneSummary state and arrow flag the gateway reports when exactly these lamps are lit.

    Same precedence as PerceptionGatewayNode.map_light_observation and map_light_arrow_active.
    """
    red = bool(lamps & LAMP_RED)
    yellow = bool(lamps & LAMP_YELLOW)
    green = bool(lamps & LAMP_GREEN)
    arrow = bool(lamps & LAMP_ARROW)
    if yellow:
        state = TRAFFIC_YELLOW
    elif red and green:
        state = TRAFFIC_RED
    elif red and arrow:
        state = TRAFFIC_ARROW
    elif red:
        state = TRAFFIC_RED
    elif green:
        state = TRAFFIC_GREEN
    elif arrow:
        state = TRAFFIC_ARROW
    else:
        state = TRAFFIC_UNKNOWN
    return state, arrow


def lamps_permit(lamps: int, maneuver: str) -> bool:
    """event_permission.light_allows() for these lamps on an event with this arrow maneuver."""
    state, arrow_active = lamps_scene_state(lamps)
    if str(maneuver).upper() in {"LEFT", "RIGHT"}:
        return (arrow_active or state == TRAFFIC_ARROW) and state != TRAFFIC_YELLOW
    return state == TRAFFIC_GREEN


def observed_circle(state: int, lamps: int = 0) -> str | None:
    """Circle colour one frame shows, the only alignment evidence. ARROW (red and arrow, or an
    arrow alone) names no circle; the frame's lamp bits tell whether red was lit with it."""
    circle = _CIRCLE_BY_STATE.get(int(state))
    if circle is None and int(state) == TRAFFIC_ARROW and int(lamps) & LAMP_RED:
        return "RED"
    return circle


def observed_lamps(state: int, arrow_active: bool) -> int:
    """Lamps one gateway-interpreted frame shows. ARROW carries no circle colour."""
    lamps = {
        TRAFFIC_RED: LAMP_RED,
        TRAFFIC_YELLOW: LAMP_YELLOW,
        TRAFFIC_GREEN: LAMP_GREEN,
        TRAFFIC_ARROW: LAMP_ARROW,
    }.get(int(state), 0)
    if lamps and arrow_active:
        lamps |= LAMP_ARROW
    return lamps


@dataclass(frozen=True)
class ProgramPhase:
    duration_s: float
    signals: Mapping[str, int]


@dataclass(frozen=True)
class SignalProgram:
    program_id: str
    period_s: float
    offset_s: float
    phases: tuple[ProgramPhase, ...]

    def light_ids(self) -> tuple[str, ...]:
        ids: dict[str, None] = {}
        for phase in self.phases:
            ids.update(dict.fromkeys(phase.signals))
        return tuple(ids)


@dataclass(frozen=True)
class MapTrafficLight:
    light_id: str
    x: float
    y: float
    z: float
    yaw: float
    stop_line_id: str = ""
    controller_id: str = ""


@dataclass(frozen=True)
class RouteSignal:
    light_id: str
    event_id: str
    stage_id: str
    stop_line_id: str
    maneuver: str


@dataclass(frozen=True)
class MapSignals:
    lights: tuple[MapTrafficLight, ...]
    programs: tuple[SignalProgram, ...]


def load_map_signals(path: Path | str) -> MapSignals:
    """Traffic lights and signal programs of a kaiev26_maps course semantics YAML."""
    document = yaml.safe_load(Path(path).expanduser().read_text(encoding="utf-8")) or {}
    lights = []
    for index, item in enumerate(document.get("traffic_lights") or []):
        pose = item.get("pose") or {}
        bbox = item.get("bbox") or {}
        light_id = str(item.get("id", "")).strip()
        if not light_id:
            raise ValueError(f"traffic_lights[{index}] requires an id")
        lights.append(
            MapTrafficLight(
                light_id=light_id,
                x=float(pose.get("x", 0.0)),
                y=float(pose.get("y", 0.0)),
                # Lamp centre height; PV26 reports the same height for a mapped light.
                z=float(bbox.get("center_z", pose.get("z", 0.0))),
                yaw=float(pose.get("yaw", 0.0)),
                stop_line_id=str((item.get("controls") or {}).get("stop_line_id", "") or ""),
                controller_id=str(item.get("controller_id", "") or ""),
            )
        )
    programs = []
    for index, item in enumerate(document.get("traffic_signal_programs") or []):
        context = f"traffic_signal_programs[{index}]"
        program_id = str(item.get("id", "")).strip()
        period_s = float(item.get("period_s", 0.0))
        if not program_id or not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError(f"{context} requires an id and a positive period_s")
        offset_s = float(item.get("offset_s", 0.0))
        phases = []
        for phase in item.get("phases") or []:
            duration_s = float(phase.get("duration_s", 0.0))
            if not math.isfinite(duration_s) or duration_s <= 0.0:
                raise ValueError(f"{context} phase duration_s must be positive")
            signals = {
                str(light_id): lamps_from_names(names)
                for light_id, names in (phase.get("signals") or {}).items()
            }
            phases.append(ProgramPhase(duration_s=duration_s, signals=signals))
        if not phases:
            raise ValueError(f"{context} has no phases")
        programs.append(
            SignalProgram(
                program_id=program_id,
                period_s=period_s,
                offset_s=offset_s if math.isfinite(offset_s) else 0.0,
                phases=tuple(phases),
            )
        )
    return MapSignals(lights=tuple(lights), programs=tuple(programs))


def route_signals_from_policy(policy: ZonePolicy | None) -> tuple[RouteSignal, ...]:
    """Lights the course's SIGNAL_OBEY events obey, with their event and maneuver."""
    if policy is None:
        return ()
    signals = []
    for stage in policy.stages:
        event = stage.event
        if event is None or event.event_type != "SIGNAL_OBEY":
            continue
        for light_id in event.traffic_light_ids:
            signals.append(
                RouteSignal(
                    light_id=light_id,
                    event_id=event.event_id,
                    stage_id=stage.stage_id,
                    stop_line_id=event.stop_line_id,
                    maneuver=event.arrow_maneuver,
                )
            )
    return tuple(signals)


@dataclass(frozen=True)
class TrackerConfig:
    alignment_bin_sec: float = 0.1
    # Probability mass of the published credible intervals.
    credible_mass: float = 0.9
    # Aligned when the credible set of the cycle alignment is at most this wide.
    aligned_interval_sec: float = 2.0
    # Frames of one light closer than this count as one observation in total.
    observation_correlation_sec: float = 0.5
    # A lit circle read as another colour. Arrow readings are no evidence (module docstring).
    circle_error_probability: float = 0.04
    # Old evidence fades toward flat over observed time (time the group is in view).
    forgetting_time_constant_sec: float = 180.0
    # Floor on log-odds against the best alignment, so a wrong lock can recover.
    max_log_odds: float = 40.0
    coupling: str = "controller"
    # Duration evidence: a stable run lasts at least edge_stable_sec over 3 frames and
    # the two runs around an edge are at most edge_max_gap_sec apart.
    edge_stable_sec: float = 1.0
    edge_max_gap_sec: float = 0.5
    # A measured run may have no informative frame for longer than this.
    duration_max_gap_sec: float = 2.0
    duration_warn_sec: float = 2.0
    # Wall-clock prior on theta (module docstring): wrapped normal with this mean and standard
    # deviation. Zero standard deviation disables it.
    prior_mean_sec: float = 0.0
    prior_std_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.coupling not in COUPLING_MODES:
            raise ValueError(f"coupling must be one of {COUPLING_MODES}")
        for name in (
            "alignment_bin_sec",
            "observation_correlation_sec",
            "max_log_odds",
            "edge_stable_sec",
        ):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")
        if not 0.0 < self.credible_mass < 1.0:
            raise ValueError("credible_mass must be within (0, 1)")
        if not 0.0 < self.circle_error_probability < 1.0:
            raise ValueError("circle_error_probability must be within (0, 1)")
        if not (math.isfinite(self.prior_mean_sec) and math.isfinite(self.prior_std_sec)):
            raise ValueError("prior_mean_sec and prior_std_sec must be finite")
        if self.prior_std_sec < 0.0:
            raise ValueError("prior_std_sec must not be negative")


@dataclass
class LightCycle:
    """One light's lamps over its program cycle, as piecewise-constant segments."""

    light_id: str
    program_id: str
    period_s: float
    offset_s: float
    starts: np.ndarray
    lamps: np.ndarray
    # Cycle time, unrolled past the period, at which the run holding each segment's lamps
    # ends; inf when the lamps never change.
    lamp_end: np.ndarray
    next_lamps: np.ndarray
    circle_runs: tuple[tuple[str, float, str, str], ...]
    maneuver: str = ""
    permit: np.ndarray | None = None
    permit_end: np.ndarray | None = None

    def segment_index(self, cycle_time: np.ndarray) -> np.ndarray:
        return np.searchsorted(self.starts, cycle_time, side="right") - 1

    def lamps_at(self, cycle_time: float) -> int:
        return int(self.lamps[self.segment_index(np.asarray([cycle_time]))[0]])


def _run_ends(starts: list[float], labels: list, period: float) -> tuple[np.ndarray, np.ndarray]:
    """End time (unrolled) of the equal-label run holding each segment, and the next label."""
    count = len(starts)
    ends = np.full(count, math.inf)
    following = list(labels)
    for index in range(count):
        for step in range(1, count + 1):
            other = (index + step) % count
            if labels[other] != labels[index]:
                wraps = (index + step) // count
                ends[index] = starts[other] + wraps * period
                following[index] = labels[other]
                break
    return ends, np.asarray(following)


def build_light_cycle(program: SignalProgram, light_id: str, maneuver: str = "") -> LightCycle:
    period = program.period_s
    starts: list[float] = []
    lamps: list[int] = []
    elapsed = 0.0
    for phase in program.phases:
        if elapsed >= period - 1.0e-9:
            break
        # Gazebo shows red on a light that a phase does not mention.
        lit = int(phase.signals.get(light_id, LAMP_RED))
        if not lamps or lamps[-1] != lit:
            starts.append(elapsed)
            lamps.append(lit)
        elapsed += phase.duration_s
    if elapsed < period - 1.0e-9 and lamps[-1] != LAMP_RED:
        # Gazebo shows red after the last phase when the phases are shorter than the period.
        starts.append(elapsed)
        lamps.append(LAMP_RED)
    lamp_end, next_lamps = _run_ends(starts, lamps, period)

    circles = [lamps_circle(value) or "OFF" for value in lamps]
    circle_starts: list[float] = []
    circle_labels: list[str] = []
    for start, label in zip(starts, circles):
        if not circle_labels or circle_labels[-1] != label:
            circle_starts.append(start)
            circle_labels.append(label)
    if len(circle_labels) > 1 and circle_labels[0] == circle_labels[-1]:
        # Join the run that wraps through the cycle start.
        circle_starts[0] = circle_starts.pop() - period
        circle_labels.pop()
    circle_runs = []
    for index, label in enumerate(circle_labels):
        end = circle_starts[index + 1] if index + 1 < len(circle_starts) else circle_starts[0] + period
        circle_runs.append(
            (
                label,
                end - circle_starts[index],
                circle_labels[index - 1],
                circle_labels[(index + 1) % len(circle_labels)],
            )
        )

    cycle = LightCycle(
        light_id=light_id,
        program_id=program.program_id,
        period_s=period,
        offset_s=program.offset_s,
        starts=np.asarray(starts, dtype=float),
        lamps=np.asarray(lamps, dtype=int),
        lamp_end=lamp_end,
        next_lamps=next_lamps.astype(int),
        circle_runs=tuple(circle_runs) if len(circle_labels) > 1 else (),
        maneuver=maneuver,
    )
    if maneuver:
        permit = [lamps_permit(value, maneuver) for value in lamps]
        permit_end, _ = _run_ends(starts, permit, period)
        cycle.permit = np.asarray(permit, dtype=bool)
        cycle.permit_end = permit_end
    return cycle


@dataclass
class AlignmentGroup:
    group_id: str
    period_s: float
    theta: np.ndarray
    light_ids: list[str] = field(default_factory=list)
    # Evidence only; forgetting and the log-odds floor act on it, never on the prior.
    log_likelihood: np.ndarray | None = None
    last_update_sec: float | None = None
    # Wall-clock prior over theta, None without one.
    log_prior: np.ndarray | None = None

    def reset(self) -> None:
        self.log_likelihood = np.zeros_like(self.theta)
        self.last_update_sec = None

    def set_prior(self, mean_sec: float, std_sec: float) -> None:
        """Wrapped normal prior over theta; a non-positive std_sec removes it."""
        if not std_sec > 0.0:
            self.log_prior = None
            return
        distance = np.mod(self.theta - mean_sec + 0.5 * self.period_s, self.period_s) - 0.5 * self.period_s
        self.log_prior = -0.5 * np.square(distance / std_sec)

    @property
    def observed(self) -> bool:
        return self.last_update_sec is not None

    @property
    def informed(self) -> bool:
        """Observed or given a prior: its posterior is not flat by construction."""
        return self.observed or self.log_prior is not None

    @property
    def bin_sec(self) -> float:
        return self.period_s / len(self.theta)

    def log_posterior(self) -> np.ndarray:
        if self.log_prior is None:
            return self.log_likelihood
        return self.log_likelihood + self.log_prior

    def posterior(self) -> np.ndarray:
        log_posterior = self.log_posterior()
        weights = np.exp(log_posterior - log_posterior.max())
        return weights / weights.sum()


@dataclass
class EdgeTracker:
    """Clean circle-colour edges of one light and the run durations between them."""

    stable_label: str | None = None
    stable_first: float = 0.0
    stable_last: float = 0.0
    stable_max_gap: float = 0.0
    stable_from_edge: bool = False
    candidate_label: str | None = None
    candidate_first: float = 0.0
    candidate_last: float = 0.0
    candidate_count: int = 0
    edges: list[dict] = field(default_factory=list)
    durations: list[dict] = field(default_factory=list)
    periods: list[dict] = field(default_factory=list)
    last_edge_by_type: dict[str, float] = field(default_factory=dict)


@dataclass
class LightHistory:
    last_lamps: int = 0
    last_informative_sec: float | None = None
    last_update_sec: float | None = None
    edges: EdgeTracker = field(default_factory=EdgeTracker)


@dataclass(frozen=True)
class PhaseEstimate:
    light_id: str
    program_id: str
    controller_id: str
    x: float
    y: float
    z: float
    yaw: float
    on_route: bool
    route_event_id: str
    stop_line_id: str
    route_maneuver: str
    predicted_lamps: int
    confidence: float
    remaining_sec: float
    remaining_min_sec: float
    remaining_max_sec: float
    next_lamps: int
    aligned: bool
    alignment_width_sec: float
    cycle_time_sec: float
    period_sec: float
    last_observed_lamps: int
    observation_age_sec: float
    permitted_now: bool
    permission_probability: float
    permission_change_sec: float


EVIDENCE_HISTORY = 20


def _weighted_quantiles(values: np.ndarray, weights: np.ndarray, quantiles: Iterable[float]) -> list[float]:
    order = np.argsort(values, kind="stable")
    cumulative = np.cumsum(weights[order])
    cumulative /= cumulative[-1]
    indices = np.searchsorted(cumulative, np.asarray(list(quantiles)), side="left")
    return [float(values[order][min(int(index), len(values) - 1)]) for index in indices]


def credible_set_width(posterior: np.ndarray, mass: float, bin_sec: float) -> float:
    """Total width of the smallest set of alignment bins that holds this probability mass."""
    ordered = np.sort(posterior)[::-1]
    count = int(np.searchsorted(np.cumsum(ordered), mass * ordered.sum(), side="left")) + 1
    return min(len(posterior), count) * bin_sec


class SignalPhaseTracker:
    def __init__(
        self,
        signals: MapSignals,
        route_signals: Iterable[RouteSignal] = (),
        config: TrackerConfig | None = None,
    ) -> None:
        self.config = config or TrackerConfig()
        self.lights = {light.light_id: light for light in signals.lights}
        self.route = {signal.light_id: signal for signal in route_signals}
        self.cycles: dict[str, LightCycle] = {}
        self.light_group: dict[str, str] = {}
        self.groups: dict[str, AlignmentGroup] = {}
        self.warnings_at_build: list[str] = []
        self._build(signals.programs)
        self.set_prior(self.config.prior_mean_sec, self.config.prior_std_sec)
        self.history: dict[str, LightHistory] = {}
        self.reset()

    def set_prior(self, mean_sec: float, std_sec: float) -> None:
        """Give every group the wall-clock prior, or remove it with a non-positive std_sec.
        The evidence stays as it is."""
        self.prior_mean_sec = float(mean_sec)
        self.prior_std_sec = max(0.0, float(std_sec))
        for group in self.groups.values():
            group.set_prior(self.prior_mean_sec, self.prior_std_sec)

    @property
    def prior_active(self) -> bool:
        return self.prior_std_sec > 0.0

    def _group_key(self, program: SignalProgram) -> str:
        if self.config.coupling == "all":
            return "all"
        if self.config.coupling == "controller":
            controllers = {
                self.lights[light_id].controller_id
                for light_id in program.light_ids()
                if light_id in self.lights and self.lights[light_id].controller_id
            }
            if len(controllers) == 1:
                return controllers.pop()
            if len(controllers) > 1:
                self.warnings_at_build.append(
                    f"program [{program.program_id}] spans controllers {sorted(controllers)}; "
                    "tracking it alone"
                )
        return program.program_id

    def _build(self, programs: Iterable[SignalProgram]) -> None:
        for program in programs:
            key = self._group_key(program)
            group = self.groups.get(key)
            if group is not None and abs(group.period_s - program.period_s) > 1.0e-6:
                self.warnings_at_build.append(
                    f"program [{program.program_id}] period {program.period_s:g} s differs from "
                    f"group [{key}] period {group.period_s:g} s; tracking it alone"
                )
                key = program.program_id
                group = self.groups.get(key)
            if group is None:
                bins = max(1, int(round(program.period_s / self.config.alignment_bin_sec)))
                group = AlignmentGroup(
                    group_id=key,
                    period_s=program.period_s,
                    theta=np.arange(bins, dtype=float) * (program.period_s / bins),
                )
                self.groups[key] = group
            for light_id in program.light_ids():
                if light_id in self.cycles:
                    self.warnings_at_build.append(
                        f"light [{light_id}] is in programs [{self.cycles[light_id].program_id}] "
                        f"and [{program.program_id}]; keeping the first"
                    )
                    continue
                route = self.route.get(light_id)
                self.cycles[light_id] = build_light_cycle(
                    program, light_id, route.maneuver if route is not None else ""
                )
                self.light_group[light_id] = key
                group.light_ids.append(light_id)

    def reset(self) -> None:
        for group in self.groups.values():
            group.reset()
        self.history = {light_id: LightHistory() for light_id in self.lights}

    def group_of(self, light_id: str) -> AlignmentGroup | None:
        key = self.light_group.get(light_id)
        return self.groups.get(key) if key is not None else None

    def _cycle_time(self, cycle: LightCycle, group: AlignmentGroup, stamp_sec: float) -> np.ndarray:
        return np.mod(stamp_sec + cycle.offset_s + group.theta, cycle.period_s)

    def _segment_log_likelihood(self, cycle: LightCycle, circle: str) -> np.ndarray:
        """Log-likelihood of a seen circle colour in each segment. The arrow lamp plays no part."""
        hit = math.log(1.0 - self.config.circle_error_probability)
        miss = math.log(0.5 * self.config.circle_error_probability)
        return np.asarray([hit if circle in lit_circles(int(lamps)) else miss for lamps in cycle.lamps])

    def observe(
        self,
        light_id: str,
        stamp_sec: float,
        state: int,
        arrow_active: bool,
        lamps: int = 0,
    ) -> list[str]:
        """Feed one gateway-interpreted frame of a map light (lamps: the observation's LAMP_* bits);
        returns duration warnings. Only the circle colour moves the alignment."""
        history = self.history.get(light_id)
        if history is None or not math.isfinite(stamp_sec):
            return []
        shown = observed_lamps(state, arrow_active)
        if not shown:
            # UNKNOWN carries no information about the phase.
            return []
        history.last_lamps = shown
        history.last_informative_sec = stamp_sec
        circle = observed_circle(state, lamps)
        if circle is None:
            # An arrow without a circle colour is no alignment evidence.
            return []
        warnings = self._update_edges(light_id, history.edges, stamp_sec, circle)

        cycle = self.cycles.get(light_id)
        group = self.group_of(light_id)
        if cycle is None or group is None:
            return warnings
        config = self.config
        if history.last_update_sec is None:
            weight = 1.0
        else:
            weight = min(1.0, (stamp_sec - history.last_update_sec) / config.observation_correlation_sec)
        if weight <= 0.0:
            return warnings
        if group.last_update_sec is not None and config.forgetting_time_constant_sec > 0.0:
            observed_dt = min(
                max(0.0, stamp_sec - group.last_update_sec),
                config.observation_correlation_sec,
            )
            group.log_likelihood *= math.exp(-observed_dt / config.forgetting_time_constant_sec)
        segments = cycle.segment_index(self._cycle_time(cycle, group, stamp_sec))
        group.log_likelihood += weight * self._segment_log_likelihood(cycle, circle)[segments]
        group.log_likelihood -= group.log_likelihood.max()
        np.maximum(group.log_likelihood, -config.max_log_odds, out=group.log_likelihood)
        history.last_update_sec = stamp_sec
        group.last_update_sec = (
            stamp_sec if group.last_update_sec is None else max(group.last_update_sec, stamp_sec)
        )
        return warnings

    def _update_edges(self, light_id: str, edges: EdgeTracker, stamp_sec: float, label: str | None) -> list[str]:
        if label is None:
            return []
        config = self.config
        if label == edges.stable_label:
            edges.stable_max_gap = max(edges.stable_max_gap, stamp_sec - edges.stable_last)
            edges.stable_last = stamp_sec
            edges.candidate_label = None
            return []
        if label != edges.candidate_label:
            edges.candidate_label = label
            edges.candidate_first = stamp_sec
            edges.candidate_count = 0
        edges.candidate_last = stamp_sec
        edges.candidate_count += 1
        if edges.candidate_count < 3 or edges.candidate_last - edges.candidate_first < config.edge_stable_sec:
            return []

        warnings: list[str] = []
        previous = edges.stable_label
        clean = (
            previous is not None
            and edges.stable_last - edges.stable_first >= config.edge_stable_sec
            and edges.candidate_first - edges.stable_last <= config.edge_max_gap_sec
        )
        if clean:
            edge_sec = 0.5 * (edges.stable_last + edges.candidate_first)
            edge = {"t": round(edge_sec, 3), "from": previous, "to": label}
            if edges.stable_from_edge and edges.stable_max_gap <= config.duration_max_gap_sec:
                warnings.extend(self._record_duration(light_id, edges, previous, label, edge_sec))
            warnings.extend(self._record_period(light_id, edges, f"{previous}>{label}", edge_sec))
            edges.edges.append(edge)
            del edges.edges[:-EVIDENCE_HISTORY]
        edges.stable_label = label
        edges.stable_first = edges.candidate_first
        edges.stable_last = edges.candidate_last
        edges.stable_max_gap = 0.0
        edges.stable_from_edge = clean
        edges.candidate_label = None
        return warnings

    def _record_duration(
        self,
        light_id: str,
        edges: EdgeTracker,
        label: str,
        next_label: str,
        edge_sec: float,
    ) -> list[str]:
        start = edges.edges[-1] if edges.edges else None
        if start is None or start["to"] != label:
            return []
        observed = edge_sec - float(start["t"])
        record = {
            "label": label,
            "from": start["from"],
            "to": next_label,
            "end_t": round(edge_sec, 3),
            "observed_sec": round(observed, 2),
            "program_sec": None,
            "error_sec": None,
        }
        warnings: list[str] = []
        cycle = self.cycles.get(light_id)
        if cycle is not None:
            matches = [
                duration
                for run_label, duration, before, after in cycle.circle_runs
                if run_label == label and before == start["from"] and after == next_label
            ]
            if matches:
                program = min(matches, key=lambda value: abs(value - observed))
                record["program_sec"] = round(program, 2)
                record["error_sec"] = round(observed - program, 2)
                if abs(observed - program) > self.config.duration_warn_sec:
                    warnings.append(
                        f"light [{light_id}] {label} lasted {observed:.1f} s between "
                        f"{start['from']} and {next_label}; program [{cycle.program_id}] says {program:.1f} s"
                    )
            else:
                warnings.append(
                    f"light [{light_id}] showed {start['from']} > {label} > {next_label}, "
                    f"which program [{cycle.program_id}] does not contain"
                )
        edges.durations.append(record)
        del edges.durations[:-EVIDENCE_HISTORY]
        return warnings

    def _record_period(self, light_id: str, edges: EdgeTracker, edge_type: str, edge_sec: float) -> list[str]:
        previous = edges.last_edge_by_type.get(edge_type)
        edges.last_edge_by_type[edge_type] = edge_sec
        cycle = self.cycles.get(light_id)
        if previous is None or cycle is None:
            return []
        elapsed = edge_sec - previous
        cycles = max(1, int(round(elapsed / cycle.period_s)))
        observed = elapsed / cycles
        edges.periods.append(
            {
                "edge": edge_type,
                "end_t": round(edge_sec, 3),
                "elapsed_sec": round(elapsed, 2),
                "cycles": cycles,
                "observed_sec": round(observed, 2),
                "program_sec": cycle.period_s,
                "error_sec": round(observed - cycle.period_s, 2),
            }
        )
        del edges.periods[:-EVIDENCE_HISTORY]
        if abs(observed - cycle.period_s) > self.config.duration_warn_sec:
            return [
                f"light [{light_id}] {edge_type} repeated after {elapsed:.1f} s ({cycles} cycle(s)); "
                f"program [{cycle.program_id}] period is {cycle.period_s:.1f} s"
            ]
        return []

    def evidence(self) -> dict:
        """Clean edges, measured durations and periods per light, and each group's alignment.

        theta_sec is the most probable alignment: cycle position = (t + offset_s + theta) mod
        period with t the node clock. Equal theta across groups tracked apart is evidence that
        their controllers run one cycle. With the wall-clock prior theta_sec includes it, and
        evidence_theta_sec is the observations' own maximum.
        """
        groups = {}
        for key, group in self.groups.items():
            if not group.informed:
                continue
            posterior = group.posterior()
            width = credible_set_width(posterior, self.config.credible_mass, group.bin_sec)
            groups[key] = {
                "theta_sec": round(float(group.theta[int(np.argmax(posterior))]), 2),
                "width_sec": round(width, 2),
                "aligned": width <= self.config.aligned_interval_sec,
                "last_update_t": (
                    round(group.last_update_sec, 3) if group.last_update_sec is not None else None
                ),
            }
            if group.observed and group.log_prior is not None:
                # The observations alone, to see where they would put theta without the prior.
                groups[key]["evidence_theta_sec"] = round(
                    float(group.theta[int(np.argmax(group.log_likelihood))]), 2
                )
        lights = {}
        for light_id, history in self.history.items():
            edges = history.edges
            if not (edges.edges or edges.durations or edges.periods):
                continue
            lights[light_id] = {
                "program_id": self.cycles[light_id].program_id if light_id in self.cycles else "",
                "edges": list(edges.edges),
                "durations": list(edges.durations),
                "periods": list(edges.periods),
            }
        prior = (
            {"mean_sec": self.prior_mean_sec, "std_sec": self.prior_std_sec}
            if self.prior_active
            else None
        )
        return {"prior": prior, "groups": groups, "lights": lights}

    def estimate(self, now_sec: float) -> list[PhaseEstimate]:
        posteriors = {key: group.posterior() for key, group in self.groups.items()}
        widths = {
            key: credible_set_width(posteriors[key], self.config.credible_mass, group.bin_sec)
            for key, group in self.groups.items()
        }
        return [
            self._estimate_light(light, now_sec, posteriors, widths)
            for light in self.lights.values()
        ]

    def _estimate_light(
        self,
        light: MapTrafficLight,
        now_sec: float,
        posteriors: Mapping[str, np.ndarray],
        widths: Mapping[str, float],
    ) -> PhaseEstimate:
        history = self.history[light.light_id]
        route = self.route.get(light.light_id)
        age = (
            max(0.0, now_sec - history.last_informative_sec)
            if history.last_informative_sec is not None
            else math.nan
        )
        values = {
            "light_id": light.light_id,
            "program_id": "",
            "controller_id": light.controller_id,
            "x": light.x,
            "y": light.y,
            "z": light.z,
            "yaw": light.yaw,
            "on_route": route is not None,
            "route_event_id": route.event_id if route is not None else "",
            "stop_line_id": route.stop_line_id if route is not None else light.stop_line_id,
            "route_maneuver": route.maneuver if route is not None else "",
            "predicted_lamps": 0,
            "confidence": 0.0,
            "remaining_sec": math.nan,
            "remaining_min_sec": math.nan,
            "remaining_max_sec": math.nan,
            "next_lamps": 0,
            "aligned": False,
            "alignment_width_sec": math.nan,
            "cycle_time_sec": math.nan,
            "period_sec": math.nan,
            "last_observed_lamps": history.last_lamps,
            "observation_age_sec": age,
            "permitted_now": False,
            "permission_probability": math.nan,
            "permission_change_sec": math.nan,
        }
        cycle = self.cycles.get(light.light_id)
        group = self.group_of(light.light_id)
        if cycle is None or group is None:
            return PhaseEstimate(**values)

        posterior = posteriors[group.group_id]
        cycle_time = self._cycle_time(cycle, group, now_sec)
        segments = cycle.segment_index(cycle_time)
        lamps = cycle.lamps[segments]
        candidates = np.unique(cycle.lamps)
        masses = np.asarray([posterior[lamps == value].sum() for value in candidates])
        predicted = int(candidates[int(np.argmax(masses))])
        selected = lamps == predicted
        weights = posterior[selected]
        remaining = cycle.lamp_end[segments][selected] - cycle_time[selected]
        remaining = np.where(np.isfinite(remaining), remaining, cycle.period_s)
        low, high = _weighted_quantiles(
            remaining,
            weights,
            (0.5 * (1.0 - self.config.credible_mass), 0.5 * (1.0 + self.config.credible_mass)),
        )
        next_values = cycle.next_lamps[segments][selected]
        next_candidates = np.unique(next_values)
        next_lamps = int(
            next_candidates[int(np.argmax([weights[next_values == value].sum() for value in next_candidates]))]
        )
        width = widths[group.group_id]
        values.update(
            program_id=cycle.program_id,
            predicted_lamps=predicted,
            confidence=float(masses.max()),
            remaining_sec=float(np.dot(weights, remaining) / weights.sum()),
            remaining_min_sec=low,
            remaining_max_sec=high,
            next_lamps=next_lamps,
            aligned=group.informed and width <= self.config.aligned_interval_sec,
            alignment_width_sec=width,
            cycle_time_sec=float(cycle_time[int(np.argmax(posterior))]) if group.informed else math.nan,
            period_sec=cycle.period_s,
        )
        if route is not None and cycle.permit is not None:
            permit = cycle.permit[segments]
            probability = float(posterior[permit].sum())
            permitted = probability >= 0.5
            chosen = permit == permitted
            change = cycle.permit_end[segments][chosen] - cycle_time[chosen]
            change = np.where(np.isfinite(change), change, cycle.period_s)
            values.update(
                permitted_now=bool(permitted),
                permission_probability=probability,
                permission_change_sec=float(np.dot(posterior[chosen], change) / posterior[chosen].sum()),
            )
        return PhaseEstimate(**values)
