"""Scripted perception: the four pv26 topics of one camera epoch, built from map truth.

- traffic_lights: every scripted light that is not `unseen` and is ahead within the camera range and
  horizontal field of view (no vertical limit), by its map id (track_id = trailing
  digits, as pv26 assigns), state from the lit lamps with pv26's precedence, map head position in
  base_footprint;
- objects: drums within LiDAR range and field of view, touching drums merged into one cluster
  box like a Euclidean clustering would, CLASS_OBSTACLE, track_id 0, twist 0;
- road_segments: map lane markings (and stop lines) inside the camera window, 0.5 m spacing;
- centerline: the ego lane centre (the route) from 2 to 15 m ahead.
All four share one header (stamp = epoch time, frame base_footprint) like pv26.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import (
    Centerline,
    PerceptionObject,
    PerceptionObjectArray,
    RoadSegment,
    RoadSegmentArray,
    TrafficLightObservation,
    TrafficLightObservationArray,
)
from std_msgs.msg import Header

from .course import Polyline
from .world import LightView


LAMP_BITS = {
    "red": TrafficLightObservation.LAMP_RED,
    "yellow": TrafficLightObservation.LAMP_YELLOW,
    "green": TrafficLightObservation.LAMP_GREEN,
    "arrow": TrafficLightObservation.LAMP_ARROW,
}


def lamps_state(lamps: frozenset) -> int:
    """pv26 precedence: yellow > red+green=red > red+arrow=ARROW > red > green > arrow."""
    if "yellow" in lamps:
        return TrafficLightObservation.STATE_YELLOW
    if "red" in lamps and "green" in lamps:
        return TrafficLightObservation.STATE_RED
    if "red" in lamps and "arrow" in lamps:
        return TrafficLightObservation.STATE_ARROW
    if "red" in lamps:
        return TrafficLightObservation.STATE_RED
    if "green" in lamps:
        return TrafficLightObservation.STATE_GREEN
    if "arrow" in lamps:
        return TrafficLightObservation.STATE_ARROW
    return TrafficLightObservation.STATE_UNKNOWN


@dataclass
class SensorParams:
    rate_hz: float = 30.0
    light_range_m: float = 80.0
    light_min_x_m: float = 1.0
    light_half_fov_deg: float = 60.0
    lidar_range_m: float = 40.0
    lidar_min_x_m: float = 0.5
    lidar_half_fov_deg: float = 90.0
    cluster_gap_m: float = 0.3
    lane_min_x_m: float = 4.0
    lane_max_x_m: float = 25.0
    lane_max_lateral_m: float = 6.0
    centerline_min_x_m: float = 2.0
    centerline_max_x_m: float = 15.0
    confidence: float = 0.9

    @classmethod
    def from_dict(cls, values: dict | None) -> "SensorParams":
        params = cls()
        for key, value in (values or {}).items():
            if not hasattr(params, key):
                raise ValueError(f"unknown perception parameter [{key}]")
            setattr(params, key, float(value))
        return params


def to_base(pose: tuple[float, float, float], x: float, y: float) -> tuple[float, float]:
    px, py, yaw = pose
    dx, dy = x - px, y - py
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def point(x: float, y: float, z: float = 0.0) -> Point:
    value = Point()
    value.x, value.y, value.z = float(x), float(y), float(z)
    return value


class PerceptionDriver:
    def __init__(self, course, world, params: SensorParams) -> None:
        self.course = course
        self.world = world
        self.p = params
        self.marking_samples = []
        for marking in course.markings:
            kind = marking.kind.lower()
            seg_type = RoadSegment.TYPE_YESOL if "yellow" in kind else RoadSegment.TYPE_WHSOL
            self.marking_samples.append(self._prepared(seg_type, self._resample(marking.line)))
        for line in course.stop_lines.values():
            self.marking_samples.append(self._prepared(RoadSegment.TYPE_STOP_LINE, self._resample(Polyline(list(line)))))

    @staticmethod
    def _prepared(seg_type: int, samples: list[tuple[float, float]]):
        xs = [x for x, _ in samples]
        ys = [y for _, y in samples]
        return seg_type, samples, (min(xs), max(xs), min(ys), max(ys))

    @staticmethod
    def _resample(line, spacing: float = 0.5) -> list[tuple[float, float]]:
        count = max(1, int(line.length / spacing))
        return [line.pose_at(i * line.length / count)[:2] for i in range(count + 1)]

    def header(self, stamp) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = "base_footprint"
        return header

    def traffic_lights(self, stamp, pose, t: float) -> TrafficLightObservationArray:
        msg = TrafficLightObservationArray()
        msg.header = self.header(stamp)
        for index, (light_id, script) in enumerate(self.world.lights.items()):
            view: LightView = script.view(t)
            if not view.visible:
                continue
            light = self.course.lights[light_id]
            bx, by = to_base(pose, light.x, light.y)
            if (
                bx < self.p.light_min_x_m
                or math.hypot(bx, by) > self.p.light_range_m
                or abs(math.degrees(math.atan2(by, bx))) > self.p.light_half_fov_deg
            ):
                continue
            obs = TrafficLightObservation()
            obs.detection_id = index
            obs.track_id = light.numeric_id
            obs.state = lamps_state(view.lamps)
            obs.lamps = sum(LAMP_BITS[name] for name in view.lamps)
            obs.position = point(bx, by, light.z)
            obs.red_score = 0.9 if "red" in view.lamps else 0.05
            obs.yellow_score = 0.9 if "yellow" in view.lamps else 0.05
            obs.green_score = 0.9 if "green" in view.lamps else 0.05
            obs.arrow_score = 0.9 if "arrow" in view.lamps else 0.05
            obs.confidence = float(self.p.confidence)
            obs.source = "pv26/left"
            msg.lights.append(obs)
        return msg

    def objects(self, stamp, pose) -> PerceptionObjectArray:
        msg = PerceptionObjectArray()
        msg.header = self.header(stamp)
        visible = []
        for drum in self.world.drums:
            bx, by = to_base(pose, drum.x, drum.y)
            distance = math.hypot(bx, by)
            bearing = math.degrees(math.atan2(by, bx))
            if bx < self.p.lidar_min_x_m or distance > self.p.lidar_range_m or abs(bearing) > self.p.lidar_half_fov_deg:
                continue
            visible.append((bx, by, drum))
        clusters: list[list[tuple[float, float, object]]] = []
        for item in visible:
            joined = None
            for cluster in clusters:
                if any(
                    math.hypot(item[0] - other[0], item[1] - other[1])
                    <= (item[2].diameter_m + other[2].diameter_m) / 2.0 + self.p.cluster_gap_m
                    for other in cluster
                ):
                    joined = cluster
                    break
            if joined is None:
                clusters.append([item])
            else:
                joined.append(item)
        for index, cluster in enumerate(clusters):
            xmin = min(bx - d.diameter_m / 2.0 for bx, _, d in cluster)
            xmax = max(bx + d.diameter_m / 2.0 for bx, _, d in cluster)
            ymin = min(by - d.diameter_m / 2.0 for _, by, d in cluster)
            ymax = max(by + d.diameter_m / 2.0 for _, by, d in cluster)
            height = max(d.height_m for _, _, d in cluster)
            obj = PerceptionObject()
            obj.detection_id = index
            obj.track_id = 0
            obj.class_id = PerceptionObject.CLASS_OBSTACLE
            obj.pose.position = point((xmin + xmax) / 2.0, (ymin + ymax) / 2.0, height / 2.0)
            obj.pose.orientation.w = 1.0
            obj.size.x, obj.size.y, obj.size.z = xmax - xmin, ymax - ymin, height
            obj.confidence = 1.0
            obj.source = "pv26/lidar"
            msg.objects.append(obj)
        return msg

    def road_segments(self, stamp, pose) -> RoadSegmentArray:
        msg = RoadSegmentArray()
        msg.header = self.header(stamp)
        px, py, yaw = pose
        c, sn = math.cos(yaw), math.sin(yaw)
        reach = self.p.lane_max_x_m + self.p.lane_max_lateral_m
        x_lo, x_hi, y_max = self.p.lane_min_x_m, self.p.lane_max_x_m, self.p.lane_max_lateral_m
        detection = 0
        for seg_type, samples, (bx0, bx1, by0, by1) in self.marking_samples:
            if bx0 > px + reach or bx1 < px - reach or by0 > py + reach or by1 < py - reach:
                continue
            runs = []
            run: list[tuple[float, float]] = []
            for x, y in samples:
                dx, dy = x - px, y - py
                fx, ly = c * dx + sn * dy, -sn * dx + c * dy
                if x_lo <= fx <= x_hi and -y_max <= ly <= y_max:
                    run.append((fx, ly))
                elif run:
                    runs.append(run)
                    run = []
            if run:
                runs.append(run)
            for points in runs:
                if len(points) < 2:
                    continue
                segment = RoadSegment()
                segment.detection_id = detection
                detection += 1
                segment.track_id = 0
                segment.type = seg_type
                segment.points = [point(fx, ly) for fx, ly in points]
                segment.confidence = float(self.p.confidence)
                segment.source = "pv26/left"
                msg.segments.append(segment)
        return msg

    def centerline(self, stamp, pose, route_s: float) -> Centerline:
        msg = Centerline()
        msg.header = self.header(stamp)
        points = []
        s = route_s + self.p.centerline_min_x_m
        while s <= route_s + self.p.centerline_max_x_m + 1.0e-6 and s <= self.course.route.length:
            x, y, _ = self.course.route.pose_at(s)
            bx, by = to_base(pose, x, y)
            if bx >= self.p.centerline_min_x_m - 0.5:
                points.append(point(bx, by))
            s += 0.5
        msg.points = points
        msg.confidence = float(self.p.confidence) if len(points) >= 2 else 0.0
        msg.source = "pv26/centerline"
        return msg
