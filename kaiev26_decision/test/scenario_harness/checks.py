"""The fixed vocabulary of scenario expectations.

Every check reads only the run record: plant truth (pose, speed, brake applied, drive active),
the actuator command the plant received and the scripted world. The Decision fields read are the
FINISH_HELD constraint of BehaviorDecision for the finish report (D11) and its active behavior for
behavior_absent (a mission end or stop that must not pass through RECOVERY). Scenario files pick
checks by name; adding a scenario never needs a new check, and a new check is a change of this file.

Instants used by windows: "start", "end", {dropout: i, at: start|end},
{light: <id>|event, step: k} (when step k of that light's script fired; `event` is the scenario
event's first light), {phase: event, step: k} (when step k of the event_phases script fired),
{stop: k, event: <id>, at: start|end} (k-th standstill at the event, 1-based),
each with an optional plus_s.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .world import MOVING_MPS, STOPPED_MPS

# D11: Decision reports "finished" with the FINISH_HELD constraint once it stands still after the finish.
FINISH_REPORT_CONSTRAINT = "FINISH_HELD"
# A standstill shorter than this is a speed dip, not a stop.
MIN_STANDSTILL_S = 0.2
# Standstills and brake applications belong to an event inside this front-bumper window.
EVENT_WINDOW_BEFORE_M = 45.0
EVENT_WINDOW_AFTER_M = 15.0
# Brake while moving faster than this is "braking while moving".
MOVING_BRAKE_MPS = 0.5


@dataclass
class CheckResult:
    check: str
    passed: bool
    detail: str


class CheckError(Exception):
    pass


class Record:
    """Run record with derived standstills, prepared once for all checks."""

    def __init__(self, result) -> None:
        self.result = result
        self.s = result.samples
        self.course = result.course
        self.world = result.world
        self.vehicle = result.vehicle
        self.t = self.s["t"]
        self.first = next(i for i, t in enumerate(self.t) if t >= -1.0e-9)
        self.idx = range(self.first, len(self.t))
        self.standstills = self._standstills()

    def _standstills(self) -> list[dict]:
        found = []
        start = None
        v = self.s["v"]
        for i in self.idx:
            if v[i] <= STOPPED_MPS:
                if start is None:
                    start = i
            elif start is not None:
                found.append((start, i))
                start = None
        if start is not None:
            found.append((start, len(self.t) - 1))
        stops = []
        for a, b in found:
            if self.t[b] - self.t[a] < MIN_STANDSTILL_S and b != len(self.t) - 1:
                continue
            stops.append({"i0": a, "i1": b, "t0": self.t[a], "t1": self.t[b], "front_s": self.s["front_s"][a],
                          "s": self.s["s"][a], "until_end": b == len(self.t) - 1})
        return stops

    def event_line(self, event_id: str) -> float:
        return self.course.event(event_id).line_s

    def near_event(self, front_s: float, event_id: str) -> bool:
        line = self.event_line(event_id)
        return line - EVENT_WINDOW_BEFORE_M <= front_s <= line + EVENT_WINDOW_AFTER_M

    def stops_at(self, event_id: str) -> list[dict]:
        return [stop for stop in self.standstills if self.near_event(stop["front_s"], event_id)]

    def instant(self, spec) -> float:
        if spec is None:
            raise CheckError("missing instant")
        if isinstance(spec, (int, float)):
            return float(spec)
        if spec == "start":
            return 0.0
        if spec == "end":
            return self.t[-1]
        if not isinstance(spec, dict):
            raise CheckError(f"bad instant {spec}")
        plus = float(spec.get("plus_s", 0.0))
        if "dropout" in spec:
            dropout = self.world.dropouts[int(spec["dropout"])]
            value = dropout.started_at if spec.get("at", "start") == "start" else dropout.ended_at
        elif "light" in spec:
            light_id = spec["light"]
            if light_id == "event":
                light_id = self.course.event(self.result.scenario["event"]).light_ids[0]
            value = self.world.lights[light_id].fire_times[int(spec["step"])]
        elif "phase" in spec:
            if spec["phase"] != "event" or self.world.phases is None:
                raise CheckError(f"bad instant {spec}: phase needs `event` and world.event_phases")
            value = self.world.phases.fire_times[int(spec["step"])]
        elif "stop" in spec:
            stops = self.stops_at(spec["event"]) if "event" in spec else self.standstills
            k = int(spec["stop"])
            value = None
            if 1 <= k <= len(stops):
                value = stops[k - 1]["t0" if spec.get("at", "start") == "start" else "t1"]
        else:
            raise CheckError(f"bad instant {spec}")
        if value is None:
            raise CheckError(f"instant {spec} did not happen")
        return float(value) + plus

    def window(self, params: dict) -> range:
        """[from, to]; a `to` instant that never happened (the light never changed) means the end."""
        t0 = self.instant(params.get("from", "start"))
        try:
            t1 = self.instant(params.get("to", "end"))
        except CheckError:
            t1 = self.t[-1]
        return range(
            next((i for i in self.idx if self.t[i] >= t0 - 1.0e-9), len(self.t)),
            next((i for i in self.idx if self.t[i] > t1 + 1.0e-9), len(self.t)),
        )

    def brake_edges(self, indices) -> list[int]:
        brake = self.s["cmd_brake"]
        edges = []
        previous = None
        for i in indices:
            if brake[i] and previous is False:
                edges.append(i)
            previous = brake[i]
        return edges


# ---------------------------------------------------------------- helpers
def _range(values, name: str) -> tuple[float, float]:
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise CheckError(f"{name} must be [low, high]")
    low = -math.inf if values[0] is None else float(values[0])
    high = math.inf if values[1] is None else float(values[1])
    return low, high


def _count_ok(count: int, params: dict) -> tuple[bool, str]:
    if "count" in params:
        return count == int(params["count"]), f"== {int(params['count'])}"
    low = int(params.get("min", 0))
    high = int(params["max"]) if "max" in params else None
    ok = count >= low and (high is None or count <= high)
    return ok, f"in [{low}, {high if high is not None else 'inf'}]"


def _permits(lamps: frozenset, arrow: str) -> bool:
    if "yellow" in lamps:
        return False
    if arrow in ("LEFT", "RIGHT"):
        return "arrow" in lamps
    return "green" in lamps


def _own_red(record: Record, i: int, event) -> bool:
    lights = record.s["lights"][i]
    shown = [lights[light_id] for light_id in event.light_ids if light_id in lights]
    return any("red" in view.lamps and not _permits(view.lamps, event.arrow) for view in shown)


# ---------------------------------------------------------------- checks
def check_stop_position(record: Record, p: dict) -> tuple[bool, str]:
    low, high = _range(p["front_gap_m"], "front_gap_m")
    nth = int(p.get("nth", 1))
    stops = record.stops_at(p["event"])
    if len(stops) < nth:
        return False, f"no standstill #{nth} at {p['event']} (standstills: {len(record.standstills)})"
    gap = record.event_line(p["event"]) - stops[nth - 1]["front_s"]
    return low <= gap <= high, f"front bumper {gap:.2f} m before the line (want [{low}, {high}])"


def check_stop_count(record: Record, p: dict) -> tuple[bool, str]:
    stops = record.stops_at(p["event"])
    if "after" in p:
        after = record.instant(p["after"])
        stops = [stop for stop in stops if stop["t0"] > after + 1.0e-9]
    ok, want = _count_ok(len(stops), p)
    gaps = ", ".join(f"{record.event_line(p['event']) - stop['front_s']:.1f}m@{stop['t0']:.1f}s" for stop in stops)
    return ok, f"{len(stops)} standstill(s) {want} [{gaps}]"


def check_brake_count(record: Record, p: dict) -> tuple[bool, str]:
    indices = record.window(p) if ("from" in p or "to" in p) else record.idx
    if "event" in p:
        indices = [i for i in indices if record.near_event(record.s["front_s"][i], p["event"])]
    edges = record.brake_edges(indices)
    ok, want = _count_ok(len(edges), p)
    times = ", ".join(f"{record.t[i]:.2f}s/v{record.s['v'][i]:.1f}" for i in edges)
    return ok, f"{len(edges)} brake application(s) {want} [{times}]"


def check_brake_held(record: Record, p: dict) -> tuple[bool, str]:
    indices = record.window(p)
    if not indices:
        return False, "empty window"
    released = [i for i in indices if not record.s["cmd_brake"][i]]
    if released:
        i = released[0]
        return False, f"brake released at t={record.t[i]:.2f} s (v={record.s['v'][i]:.2f}) in [{record.t[indices[0]]:.2f}, {record.t[indices[-1]]:.2f}]"
    return True, f"brake held over [{record.t[indices[0]]:.2f}, {record.t[indices[-1]]:.2f}] s"


def check_no_motion(record: Record, p: dict) -> tuple[bool, str]:
    indices = record.window(p)
    if not indices:
        return False, "empty window"
    limit = float(p.get("max_travel_m", 0.1))
    s0 = record.s["s"][indices[0]]
    travel = max(abs(record.s["s"][i] - s0) for i in indices)
    return travel <= limit, f"travel {travel:.2f} m over [{record.t[indices[0]]:.2f}, {record.t[indices[-1]]:.2f}] s (max {limit})"


def check_launch_after(record: Record, p: dict) -> tuple[bool, str]:
    after = record.instant(p["after"])
    low = float(p.get("min_delay_s", 0.0))
    high = float(p["max_delay_s"])
    for i in record.idx:
        if record.t[i] >= after - 1.0e-9 and record.s["v"][i] > MOVING_MPS:
            delay = record.t[i] - after
            return low <= delay <= high, f"moving {delay:.2f} s after t={after:.2f} (want [{low}, {high}])"
    return False, f"never moved after t={after:.2f}"


def check_passes_line(record: Record, p: dict) -> tuple[bool, str]:
    line = record.event_line(p["event"])
    for i in record.idx:
        if record.s["front_s"][i] > line:
            limit = p.get("by_t")
            ok = limit is None or record.t[i] <= float(limit)
            return ok, f"front bumper crossed the line at t={record.t[i]:.2f} s"
    return False, "never crossed the line"


def check_no_red_crossing(record: Record, p: dict) -> tuple[bool, str]:
    """Never cross the event's line, nor move on after an overshoot, while its own light is red."""
    event = record.course.event(p["event"])
    line = event.line_s
    front = record.s["front_s"]
    rear = record.s["s"]
    previous = None
    moved_on_red = 0.0
    for i in record.idx:
        red = _own_red(record, i, event)
        if previous is not None and front[previous] <= line < front[i] and red:
            return False, f"crossed the line on red at t={record.t[i]:.2f} s, v={record.s['v'][i]:.2f}"
        if previous is not None and front[i] > line and rear[i] < line + 10.0 and red:
            moved_on_red += max(0.0, rear[i] - rear[previous])
        previous = i
    limit = float(p.get("max_travel_m", 0.5))
    return moved_on_red <= limit, f"travel past the line on red {moved_on_red:.2f} m (max {limit})"


def check_no_brake_while_moving(record: Record, p: dict) -> tuple[bool, str]:
    """Stale hard brake detector: brake commanded above walking speed outside allowed places."""
    events = list(p.get("allow_events", []))
    if p.get("allow_stop_events"):
        events = [e.event_id for e in record.course.events if e.kind in ("SIGNAL_OBEY", "TIMED_STOP", "FINISH")]
    allowed_windows = [record.window(window) for window in p.get("allow_windows", [])]
    speed = float(p.get("min_speed_mps", MOVING_BRAKE_MPS))
    for i in record.idx:
        if not record.s["cmd_brake"][i] or record.s["v"][i] <= speed:
            continue
        if any(record.near_event(record.s["front_s"][i], event) for event in events):
            continue
        if any(i in window for window in allowed_windows):
            continue
        return False, f"brake at t={record.t[i]:.2f} s, v={record.s['v'][i]:.2f}, s={record.s['s'][i]:.1f}"
    return True, "no brake while moving outside allowed events"


def check_drive_cut(record: Record, p: dict) -> tuple[bool, str]:
    after = record.instant(p["after"])
    within = float(p["within_s"])
    until = record.instant(p["until"]) if "until" in p else None
    for i in record.idx:
        t = record.t[i]
        if t < after + within - 1.0e-9:
            continue
        if until is not None and t > until + 1.0e-9:
            break
        if record.s["v"][i] <= STOPPED_MPS:
            break
        if record.s["drive_active"][i]:
            return False, f"drive still on at t={t:.2f} s ({t - after:.2f} s after t={after:.2f}), cmd {record.s['cmd_speed'][i]:.2f} m/s at v={record.s['v'][i]:.2f}"
    return True, f"no drive from {within} s after t={after:.2f} until standstill"


def check_brake_not_before(record: Record, p: dict) -> tuple[bool, str]:
    after = record.instant(p["after"])
    delay = float(p["delay_s"])
    for i in record.idx:
        t = record.t[i]
        if after - 1.0e-9 <= t < after + delay - 0.02 and record.s["cmd_brake"][i]:
            return False, f"brake at t={t:.2f} s, {t - after:.2f} s after t={after:.2f} (not before {delay} s)"
    return True, f"no brake within {delay} s after t={after:.2f}"


def check_brake_until_stop(record: Record, p: dict) -> tuple[bool, str]:
    """Brake requested by after+within_s and kept until the vehicle stood still."""
    after = record.instant(p["after"])
    within = float(p["within_s"])
    start = next((i for i in record.idx if record.t[i] >= after - 1.0e-9 and record.s["cmd_brake"][i]), None)
    if start is None or record.t[start] > after + within + 1.0e-9:
        when = "never" if start is None else f"at t={record.t[start]:.2f}"
        return False, f"brake {when}, wanted by t={after + within:.2f} ({within} s after t={after:.2f})"
    for i in range(start, len(record.t)):
        if record.s["v"][i] <= STOPPED_MPS:
            return True, f"brake from t={record.t[start]:.2f} s ({record.t[start] - after:.2f} s after) held until standstill at t={record.t[i]:.2f}"
        if not record.s["cmd_brake"][i]:
            return False, f"brake from t={record.t[start]:.2f} s released at t={record.t[i]:.2f} s while moving at {record.s['v'][i]:.2f} m/s"
    return False, f"brake from t={record.t[start]:.2f} s but never stood still"


def check_finish_hold(record: Record, p: dict) -> tuple[bool, str]:
    low, high = _range(p.get("front_past_m", [0.0, None]), "front_past_m")
    line = record.course.route.length
    final = record.standstills[-1] if record.standstills else None
    if final is None or not final["until_end"]:
        return False, f"not standing still at the end (v={record.s['v'][-1]:.2f}, front {record.s['front_s'][-1] - line:.2f} m past the finish)"
    past = final["front_s"] - line
    hold = record.t[-1] - final["t0"]
    released = [i for i in range(final["i0"], len(record.t)) if record.t[i] >= final["t0"] + 0.5 and not record.s["cmd_brake"][i]]
    ok = low <= past <= high and not released
    detail = f"stood still {past:.2f} m past the finish (want [{low}, {high}]), held {hold:.1f} s"
    if released:
        detail += f", brake released at t={record.t[released[0]]:.2f}"
    return ok, detail


def check_finish_reported_once(record: Record, p: dict) -> tuple[bool, str]:
    """D11: the finish report rises once, only when the car stands still past the finish line, and stays."""
    held = record.s["finish_held"]
    line = record.course.route.length
    rises = 0
    previous = False
    first = None
    for i in record.idx:
        now = bool(held[i])
        if now and not previous:
            rises += 1
            first = i if first is None else first
        previous = now
    if first is None:
        return False, "finish never reported"
    past = record.s["front_s"][first] - line
    speed = record.s["v"][first]
    ok = rises == 1 and previous and past >= 0.0 and speed <= STOPPED_MPS
    return ok, (f"finish reported {rises} time(s), still at the end: {previous}; first at t={record.t[first]:.2f} "
                f"with the front {past:.2f} m past the finish at {speed:.2f} m/s")


def check_behavior_absent(record: Record, p: dict) -> tuple[bool, str]:
    """BehaviorDecision never reports `behavior` (e.g. RECOVERY) in the window."""
    name = str(p["behavior"])
    indices = record.window(p)
    found = [i for i in indices if record.s["behavior"][i] == name]
    if found:
        i = found[0]
        span = record.t[found[-1]] - record.t[i]
        return False, (f"{name} from t={record.t[i]:.2f} s ({record.s['fsm'][i]}), {len(found)} sample(s) "
                       f"over {span:.2f} s, front {record.s['front_s'][i] - record.course.route.length:.2f} m past the route end")
    return True, f"no {name} over [{record.t[indices[0]]:.2f}, {record.t[indices[-1]]:.2f}] s" if indices else "empty window"


def check_max_speed(record: Record, p: dict) -> tuple[bool, str]:
    low_s, high_s = _range(p["front_s_range"], "front_s_range")
    limit = float(p["max_mps"])
    worst, where = 0.0, None
    for i in record.idx:
        if low_s <= record.s["front_s"][i] <= high_s and record.s["v"][i] > worst:
            worst, where = record.s["v"][i], i
    if where is None:
        return False, "never inside the range"
    return worst <= limit, f"max {worst:.2f} m/s at front s={record.s['front_s'][where]:.1f} (max {limit})"


def check_speed_reached(record: Record, p: dict) -> tuple[bool, str]:
    low_s, high_s = _range(p["front_s_range"], "front_s_range")
    want = float(p["min_mps"])
    best = max((record.s["v"][i] for i in record.idx if low_s <= record.s["front_s"][i] <= high_s), default=None)
    if best is None:
        return False, "never inside the range"
    return best >= want, f"highest speed {best:.2f} m/s (want at least {want})"


def _body_drum_clearance(record: Record, i: int, drum) -> float:
    x, y, yaw = record.s["x"][i], record.s["y"][i], record.s["yaw"][i]
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = drum.x - x, drum.y - y
    fx, ly = c * dx + s * dy, -s * dx + c * dy
    v = record.vehicle
    ex = max(-v.rear_overhang_m - fx, 0.0, fx - v.front_overhang_m)
    ey = max(-v.width_m / 2.0 - ly, 0.0, ly - v.width_m / 2.0)
    return math.hypot(ex, ey) - drum.diameter_m / 2.0


def check_min_obstacle_clearance(record: Record, p: dict) -> tuple[bool, str]:
    limit = float(p.get("min_m", 0.0))
    worst, where, which = math.inf, None, None
    for i in record.idx:
        for drum in record.world.drums:
            if abs(drum.x - record.s["x"][i]) > 6.0 or abs(drum.y - record.s["y"][i]) > 6.0:
                continue
            clearance = _body_drum_clearance(record, i, drum)
            if clearance < worst:
                worst, where, which = clearance, i, drum
    if where is None:
        return False, "never near a drum"
    return worst > limit, f"min body-drum clearance {worst:.2f} m at t={record.t[where]:.2f} (row {which.row + 1}, must be > {limit})"


def check_obstacle_pass_side(record: Record, p: dict) -> tuple[bool, str]:
    details = []
    ok = True
    for row in record.world.rows:
        index = next((i for i in record.idx if record.s["s"][i] >= row["route_s"]), None)
        if index is None:
            return False, f"never reached row {row['index'] + 1}"
        lateral = record.s["lateral"][index]
        want_left = row["center_lateral_m"] < 0.0
        passed_left = lateral > row["center_lateral_m"]
        side_ok = passed_left == want_left
        ok = ok and side_ok
        details.append(f"row {row['index'] + 1} (centre {row['center_lateral_m']:+.2f}) passed {'left' if passed_left else 'right'} at lateral {lateral:+.2f}")
    return ok, "; ".join(details)


def _boundaries(record: Record, s_value: float, cache: dict) -> list[tuple[float, float]]:
    key = round(s_value * 2.0) / 2.0
    if key in cache:
        return cache[key]
    x, y, heading = record.course.route.pose_at(key)
    nx, ny = -math.sin(heading), math.cos(heading)
    found = []
    for marking in record.course.markings:
        line = marking.line
        for k in range(len(line.xs) - 1):
            ax, ay, bx, by = line.xs[k], line.ys[k], line.xs[k + 1], line.ys[k + 1]
            dx, dy = bx - ax, by - ay
            den = nx * dy - ny * dx
            if abs(den) < 1.0e-9:
                continue
            t = ((ax - x) * dy - (ay - y) * dx) / den
            u = ((ax - x) * ny - (ay - y) * nx) / den
            if 0.0 <= u <= 1.0 and abs(t) < 8.0:
                found.append((t, marking.width_m))
    cache[key] = sorted(found)
    return cache[key]


def _lateral_of(record: Record, i: int, px: float, py: float) -> float:
    return record.course.route.project(px, py, record.s["s"][i], window_m=8.0).lateral


def _corners(record: Record, i: int):
    v = record.vehicle
    x, y, yaw = record.s["x"][i], record.s["y"][i], record.s["yaw"][i]
    c, s = math.cos(yaw), math.sin(yaw)
    points = []
    for fwd in (v.front_overhang_m, -v.rear_overhang_m):
        for left in (v.width_m / 2.0, -v.width_m / 2.0):
            points.append((x + c * fwd - s * left, y + s * fwd + c * left))
    return points


def check_lane_departure(record: Record, p: dict) -> tuple[bool, str]:
    """Body beyond the ego lane's map boundaries: left (centre-line side) and right (curb side)."""
    low_s, high_s = _range(p["s_range"], "s_range")
    left_max = float(p.get("left_max_m", 0.0))
    right_max = float(p.get("right_max_m", 0.0))
    cache: dict = {}
    worst_left = worst_right = 0.0
    at_left = at_right = None
    for i in record.idx[::5]:
        if not low_s <= record.s["s"][i] <= high_s:
            continue
        bounds = _boundaries(record, record.s["s"][i], cache)
        left = [t for t, _ in bounds if t > 0.3]
        right = [t for t, _ in bounds if t < -0.3]
        if not left or not right:
            continue
        lats = [_lateral_of(record, i, *corner) for corner in _corners(record, i)]
        dl = max(lats) - min(left)
        dr = max(right) - min(lats)
        if dl > worst_left:
            worst_left, at_left = dl, i
        if dr > worst_right:
            worst_right, at_right = dr, i
    ok = worst_left <= left_max and worst_right <= right_max
    detail = f"left {worst_left:.2f} m (max {left_max})" + (f" at s={record.s['s'][at_left]:.1f}" if at_left else "")
    detail += f", right {worst_right:.2f} m (max {right_max})" + (f" at s={record.s['s'][at_right]:.1f}" if at_right else "")
    return ok, detail


def check_route_offset(record: Record, p: dict) -> tuple[bool, str]:
    low_s, high_s = _range(p["s_range"], "s_range")
    limit = float(p["max_m"])
    worst, where = 0.0, None
    for i in record.idx:
        if low_s <= record.s["s"][i] <= high_s and abs(record.s["lateral"][i]) > worst:
            worst, where = abs(record.s["lateral"][i]), i
    detail = f"max |route offset| {worst:.2f} m (max {limit})"
    if where is not None:
        detail += f" at s={record.s['s'][where]:.1f}"
    return worst <= limit, detail


def check_reaches(record: Record, p: dict) -> tuple[bool, str]:
    target = float(p["route_s"])
    for i in record.idx:
        if record.s["s"][i] >= target:
            limit = p.get("by_t")
            ok = limit is None or record.t[i] <= float(limit)
            return ok, f"reached s={target:.1f} at t={record.t[i]:.2f}"
    return False, f"never reached s={target:.1f} (last s={record.s['s'][-1]:.1f})"


CHECKS = {
    "stop_position": check_stop_position,
    "stop_count": check_stop_count,
    "brake_count": check_brake_count,
    "brake_held": check_brake_held,
    "no_motion": check_no_motion,
    "launch_after": check_launch_after,
    "passes_line": check_passes_line,
    "no_red_crossing": check_no_red_crossing,
    "no_brake_while_moving": check_no_brake_while_moving,
    "drive_cut": check_drive_cut,
    "brake_not_before": check_brake_not_before,
    "brake_until_stop": check_brake_until_stop,
    "finish_hold": check_finish_hold,
    "finish_reported_once": check_finish_reported_once,
    "behavior_absent": check_behavior_absent,
    "max_speed": check_max_speed,
    "speed_reached": check_speed_reached,
    "min_obstacle_clearance": check_min_obstacle_clearance,
    "obstacle_pass_side": check_obstacle_pass_side,
    "lane_departure": check_lane_departure,
    "route_offset": check_route_offset,
    "reaches": check_reaches,
}


def evaluate(result) -> list[CheckResult]:
    record = Record(result)
    outcomes = []
    for item in result.scenario.get("expect") or []:
        item = dict(item)
        name = item.pop("check", None)
        label = item.pop("label", None) or name
        function = CHECKS.get(name)
        if function is None:
            raise ValueError(f"unknown check [{name}]; the vocabulary is {sorted(CHECKS)}")
        try:
            passed, detail = function(record, item)
        except CheckError as error:
            passed, detail = False, str(error)
        outcomes.append(CheckResult(label, bool(passed), detail))
    return outcomes
