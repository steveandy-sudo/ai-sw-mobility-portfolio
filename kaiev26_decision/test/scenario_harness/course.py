"""Ground truth of one course: route polyline, event lines, lights, lane markings.

Read only from the data files the launch hands to the nodes (route yaml, policy, landmarks and
the map semantics they reference). Nothing here imports Decision code, so the measurements stay
independent of the stack under test.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import bisect
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def resolve_package_path(value: str) -> Path:
    if value.startswith("package://"):
        package, _, relative = value[len("package://"):].partition("/")
        return Path(get_package_share_directory(package)) / relative
    return Path(value).expanduser()


_LOADER = getattr(yaml, "CSafeLoader", yaml.SafeLoader)


def _load(path: str | Path) -> dict:
    return yaml.load(Path(path).read_text(encoding="utf-8"), Loader=_LOADER) or {}


@dataclass(frozen=True)
class Projection:
    s: float
    lateral: float  # left of the route positive
    distance: float
    heading: float


class Polyline:
    def __init__(self, points: list[tuple[float, float]]) -> None:
        if len(points) < 2:
            raise ValueError("a polyline needs two points")
        self.xs = [float(p[0]) for p in points]
        self.ys = [float(p[1]) for p in points]
        self.cumulative = [0.0]
        for index in range(1, len(points)):
            self.cumulative.append(
                self.cumulative[-1]
                + math.hypot(self.xs[index] - self.xs[index - 1], self.ys[index] - self.ys[index - 1])
            )

    @property
    def length(self) -> float:
        return self.cumulative[-1]

    def segment_heading(self, index: int) -> float:
        return math.atan2(self.ys[index + 1] - self.ys[index], self.xs[index + 1] - self.xs[index])

    def pose_at(self, s: float, lateral: float = 0.0) -> tuple[float, float, float]:
        # A negative s lies behind the first waypoint on the first segment's line.
        s = min(float(s), self.length)
        index = min(max(0, bisect.bisect_right(self.cumulative, s) - 1), len(self.xs) - 2)
        seg = self.cumulative[index + 1] - self.cumulative[index]
        ratio = 0.0 if seg <= 1.0e-9 else (s - self.cumulative[index]) / seg
        x = self.xs[index] + ratio * (self.xs[index + 1] - self.xs[index])
        y = self.ys[index] + ratio * (self.ys[index + 1] - self.ys[index])
        heading = self.segment_heading(index)
        return x - math.sin(heading) * lateral, y + math.cos(heading) * lateral, heading

    def project(self, x: float, y: float, hint_s: float | None = None, window_m: float = 30.0) -> Projection:
        if hint_s is None:
            lo, hi = 0, len(self.xs) - 2
        else:
            lo = max(0, bisect.bisect_right(self.cumulative, hint_s - window_m) - 1)
            hi = min(len(self.xs) - 2, bisect.bisect_right(self.cumulative, hint_s + window_m))
        best = None
        for index in range(lo, hi + 1):
            ax, ay = self.xs[index], self.ys[index]
            dx, dy = self.xs[index + 1] - ax, self.ys[index + 1] - ay
            length2 = dx * dx + dy * dy
            if length2 < 1.0e-12:
                continue
            t = ((x - ax) * dx + (y - ay) * dy) / length2
            first, last = index == 0, index == len(self.xs) - 2
            t = min(max(t, -math.inf if first else 0.0), math.inf if last else 1.0)
            px, py = ax + t * dx, ay + t * dy
            distance = math.hypot(x - px, y - py)
            if best is None or distance < best[0]:
                length = math.sqrt(length2)
                lateral = (dx * (y - ay) - dy * (x - ax)) / length
                best = (distance, self.cumulative[index] + t * length, lateral, math.atan2(dy, dx))
        assert best is not None
        return Projection(s=best[1], lateral=best[2], distance=best[0], heading=best[3])

    def crossings(self, a: tuple[float, float], b: tuple[float, float], extension_m: float = 1.0) -> list[float]:
        (ax, ay), (bx, by) = a, b
        length = math.hypot(bx - ax, by - ay)
        if length < 1.0e-6:
            return []
        ex, ey = (bx - ax) / length * extension_m, (by - ay) / length * extension_m
        ax, ay, bx, by = ax - ex, ay - ey, bx + ex, by + ey
        found = []
        for index in range(len(self.xs) - 1):
            sx, sy = self.xs[index], self.ys[index]
            rx, ry = self.xs[index + 1] - sx, self.ys[index + 1] - sy
            lx, ly = bx - ax, by - ay
            denominator = rx * ly - ry * lx
            if abs(denominator) < 1.0e-12:
                continue
            t = ((ax - sx) * ly - (ay - sy) * lx) / denominator
            u = ((ax - sx) * ry - (ay - sy) * rx) / denominator
            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                found.append(self.cumulative[index] + t * math.hypot(rx, ry))
        return sorted(found)


@dataclass(frozen=True)
class Event:
    event_id: str
    stage_id: str
    kind: str  # SIGNAL_OBEY, TIMED_STOP, FINISH
    line_s: float  # route s of the painted line (the route end for FINISH)
    stop_line_id: str = ""
    light_ids: tuple[str, ...] = ()
    arrow: str = "NONE"
    hold_sec: float = 0.0
    brake_margin_m: float = 0.0


@dataclass(frozen=True)
class Light:
    light_id: str
    x: float
    y: float
    z: float
    numeric_id: int


@dataclass
class LaneMarking:
    marking_id: str
    kind: str
    width_m: float
    line: Polyline


@dataclass
class Course:
    name: str
    route: Polyline
    events: list[Event]
    lights: dict[str, Light]
    stop_lines: dict[str, tuple[tuple[float, float], ...]]
    markings: list[LaneMarking] = field(default_factory=list)
    signal_programs: list[dict] = field(default_factory=list)

    def event(self, event_id: str) -> Event:
        for event in self.events:
            if event.event_id == event_id:
                return event
        raise KeyError(f"course {self.name} has no event {event_id}; known {[e.event_id for e in self.events]}")


def _light_numeric_id(light_id: str) -> int:
    digits = ""
    for char in reversed(light_id):
        if not char.isdigit():
            break
        digits = char + digits
    return int(digits) if digits else 0


def load_course(name: str, route_config: str, policy_config: str, landmark_config: str) -> Course:
    route_data = _load(route_config)
    route = Polyline([(float(p["x"]), float(p["y"])) for p in route_data["waypoints"]])
    landmarks = _load(landmark_config)
    semantics = _load(resolve_package_path(str(landmarks["map_semantics"])))
    stop_lines = {
        str(item["id"]): tuple((float(p["x"]), float(p["y"])) for p in item["points"])
        for item in semantics.get("stop_lines", [])
    }
    lights = {}
    for item in semantics.get("traffic_lights", []):
        light_id = str(item["id"])
        lights[light_id] = Light(
            light_id,
            float(item["pose"]["x"]),
            float(item["pose"]["y"]),
            float((item.get("bbox") or {}).get("center_z", item["pose"].get("z", 0.0))),
            _light_numeric_id(light_id),
        )
    markings = []
    for item in semantics.get("lane_markings", []):
        points = [(float(p["x"]), float(p["y"])) for p in item.get("points", [])]
        if len(points) >= 2:
            markings.append(LaneMarking(
                str(item["id"]), str(item.get("kind", "")), float(item.get("width_m", 0.15)), Polyline(points)
            ))
    policy = _load(policy_config)
    events = []
    previous_s = 0.0
    for stage in policy.get("stages", []):
        event = stage.get("event") or {}
        kind = str(event.get("type", "")).upper()
        if kind == "FINISH":
            line_s = route.length
        else:
            line_id = str(event.get("stop_line_id", ""))
            crossings = [s for s in route.crossings(*_ends(stop_lines[line_id])) if s >= previous_s]
            if not crossings:
                raise ValueError(f"route does not cross stop line {line_id}")
            line_s = crossings[0]
            previous_s = line_s
        events.append(Event(
            event_id=str(event.get("id", stage.get("id"))),
            stage_id=str(stage.get("id", "")),
            kind=kind,
            line_s=line_s,
            stop_line_id=str(event.get("stop_line_id", "")),
            light_ids=tuple(str(v) for v in event.get("traffic_light_ids", []) or []),
            arrow=str(event.get("arrow_maneuver", "NONE")).upper(),
            hold_sec=float(event.get("hold_sec", 0.0) or 0.0),
            brake_margin_m=float(event.get("brake_margin_m", 0.0) or 0.0),
        ))
    return Course(name, route, events, lights, stop_lines, markings, list(semantics.get("traffic_signal_programs", [])))


def _ends(points: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], tuple[float, float]]:
    return points[0], points[-1]
