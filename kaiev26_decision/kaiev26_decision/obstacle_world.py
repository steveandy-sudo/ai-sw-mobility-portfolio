"""Map-frame obstacle world of the planning engine.

Perception publishes every object; Decision decides what matters. The obstacle zone is a polygon
of the course policy; objects outside it are ignored, and inside it the lane limits come from the
surveyed lane lines of the map. An object counts once it is confirmed (seen in confirm_frames
frames); an unconfirmed one not seen again for unconfirmed_memory_sec is dropped, so a cluster that
flickers for a frame or two never becomes a row (2026-10-23 Midan: 2-frame ghosts 35 m ahead were
kept to the end and the car swerved around nothing). A box whose bottom is above
max_bottom_height_m (a sign or branch over the road; 10/23 Midan: bottom 3.5 m), or whose top is below
min_top_height_m (road surface returns; 10/23 Midan: top -0.02 to -0.08 m), is not an obstacle.
A confirmed object is kept in the map frame
after it leaves the sensor view and is forgotten only once the car has passed it. pv26 gives a
static object the same track_id across frames; an object with track_id 0 is matched to the nearest
kept object.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math

from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray
import yaml


OBSTACLE_CLASSES = {PerceptionObject.CLASS_OBSTACLE, PerceptionObject.CLASS_TRAFFIC_CONE}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_map(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def from_map(self, x: float, y: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.x, y - self.y
        return c * dx + s * dy, -s * dx + c * dy


def point_in_polygon(polygon, x: float, y: float) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, (*polygon[1:], polygon[0])):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def quaternion_yaw(q) -> float:
    norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
    if norm < 1.0e-12:
        return 0.0
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), norm - 2.0 * (q.y * q.y + q.z * q.z))


def route_pose(route_geometry, route_msg) -> Pose2D | None:
    """Rear-axle map pose the RouteContext was computed from (its progress, cte and heading error)."""
    values = (
        float(route_msg.progress_s),
        float(route_msg.cross_track_error),
        float(route_msg.heading_error),
    )
    if not route_msg.route_projection_valid or not all(math.isfinite(v) for v in values):
        return None
    s, cte, heading_error = values
    rx, ry, _, _ = route_geometry.sample_at(s)
    route_yaw = route_geometry.heading_at(s)
    return Pose2D(
        rx - math.sin(route_yaw) * cte,
        ry + math.cos(route_yaw) * cte,
        route_yaw - heading_error,
    )


def project_onto_route(route_geometry, x: float, y: float) -> tuple[float, float]:
    projection = route_geometry.project(x, y)
    return projection.progress_s, projection.cross_track_error


class ObstacleZone:
    def __init__(self, policy, route_geometry, *, map_semantics_path=None, sample_m: float = 0.5) -> None:
        self.polygon = tuple(policy.polygon)
        if policy.lane_offsets_m is not None:
            # A test route without a map (Midan): the lane is fixed at constant route offsets
            # from start_route_s to the route end, and every object counts.
            length = float(route_geometry.total_length)
            self.polygon_entry_s = self.entry_s = float(policy.start_route_s)
            self.exit_s = length
            if not self.entry_s < self.exit_s:
                raise ValueError(
                    f"obstacle zone start {self.entry_s:.1f} m is not before the route end {length:.1f} m")
            self.right, self.left = (((0.0, d), (length, d)) for d in policy.lane_offsets_m)
            return
        runs: list[list[float]] = []
        previous = False
        for s in self._stations(route_geometry.total_length, sample_m):
            inside = point_in_polygon(self.polygon, *route_geometry.sample_at(s)[:2])
            if inside and not previous:
                runs.append([s, s])
            if inside:
                runs[-1][1] = s
            previous = inside
        if not runs:
            raise ValueError("obstacle zone polygon does not contain any part of the route")
        # The polygon covers both lanes, so a route that also drives the opposite lane (the K-City
        # final start) passes through it twice. The zone is the last pass, the one to the finish.
        entry, exit_ = runs[-1]
        # A run end sample lies up to one step inside the polygon edge; the edge station is found
        # between it and the outside sample next to it.
        self.polygon_entry_s = self._edge(route_geometry, entry, max(0.0, entry - sample_m))
        self.exit_s = self._edge(route_geometry, exit_, min(route_geometry.total_length, exit_ + sample_m))
        # The speed cap starts at the policy's start_route_s when it gives one (a waypoint).
        self.entry_s = self.polygon_entry_s if policy.start_route_s is None else policy.start_route_s
        if not self.entry_s < self.exit_s:
            raise ValueError(
                f"obstacle zone start {self.entry_s:.1f} m is not before its end {self.exit_s:.1f} m")
        path = map_semantics_path
        if path is None:
            from kaiev26_decision.route_zone_manager_node import resolve_package_path

            path = resolve_package_path(policy.map_semantics)
        with open(path, "r", encoding="utf-8") as stream:
            semantics = yaml.safe_load(stream) or {}
        markings = {str(item["id"]): item for item in semantics.get("lane_markings", [])}
        self.right = self._boundary(
            markings, policy.right_boundary_ids, route_geometry, policy.boundary_max_offset_m)
        self.left = self._boundary(
            markings, policy.left_boundary_ids, route_geometry, policy.boundary_max_offset_m)

    @staticmethod
    def _stations(length: float, step: float):
        count = max(1, int(math.ceil(length / step)))
        return [min(length, index * step) for index in range(count + 1)]

    def _edge(self, route_geometry, inside_s: float, outside_s: float, tolerance_m: float = 1.0e-4) -> float:
        """Last route s inside the polygon between an inside and an outside station."""
        if point_in_polygon(self.polygon, *route_geometry.sample_at(outside_s)[:2]):
            return inside_s
        while abs(outside_s - inside_s) > tolerance_m:
            middle = 0.5 * (inside_s + outside_s)
            if point_in_polygon(self.polygon, *route_geometry.sample_at(middle)[:2]):
                inside_s = middle
            else:
                outside_s = middle
        return inside_s

    def _boundary(self, markings, ids, route_geometry, max_offset_m):
        samples = []
        for marking_id in ids:
            if marking_id not in markings:
                raise ValueError(f"obstacle zone lane line [{marking_id}] is not in the map")
            points = [(float(p["x"]), float(p["y"])) for p in markings[marking_id]["points"]]
            dense = []
            for (ax, ay), (bx, by) in zip(points, points[1:]):
                count = max(1, int(math.hypot(bx - ax, by - ay) / 1.0))
                dense.extend((ax + (bx - ax) * k / count, ay + (by - ay) * k / count) for k in range(count))
            dense.append(points[-1])
            for x, y in dense:
                s, d = project_onto_route(route_geometry, x, y)
                if abs(d) <= max_offset_m:
                    samples.append((s, d))
        samples.sort()
        if len(samples) < 2:
            raise ValueError("obstacle zone lane line has fewer than two points near the route")
        return tuple(samples)

    @property
    def fixed_lane(self) -> bool:
        """A test-route zone with lane_offsets_m: no polygon, the zone runs to the route end."""
        return not self.polygon

    def contains(self, x: float, y: float) -> bool:
        return self.fixed_lane or point_in_polygon(self.polygon, x, y)

    def lane_lines(self, progress_s: float, ahead_m: float, behind_m: float = 5.0):
        """Right and left limits as (local s, d) samples, local s = route s - progress_s.

        The painted lines have gaps (about 622-646 m on the K-City final). The nearest sample
        outside the window on each side is kept, so a gap at the window edge is bridged.
        """
        lines = []
        start, end = progress_s - behind_m, progress_s + ahead_m
        for samples in (self.right, self.left):
            first = next((i for i, (s, _) in enumerate(samples) if s >= start), len(samples))
            last = next((i for i in range(len(samples) - 1, -1, -1) if samples[i][0] <= end), -1)
            chosen = samples[max(0, first - 1):min(len(samples), last + 2)]
            local = [(s - progress_s, d) for s, d in chosen]
            if len(local) >= 2:
                lines.append(local)
        return lines


@dataclass
class KeptObject:
    key: int
    obj: PerceptionObject
    x: float
    y: float
    yaw: float
    route_s: float
    last_seen_sec: float
    frames: int = 1


class ObstacleWorld:
    def __init__(
        self,
        route_geometry,
        zone: ObstacleZone | None = None,
        *,
        association_m: float = 1.0,
        passed_margin_m: float = 1.5,
        rear_overhang_m: float = 0.62,
        max_objects: int = 64,
        confirm_frames: int = 1,
        unconfirmed_memory_sec: float = 0.8,
        max_bottom_height_m: float = math.inf,
        min_top_height_m: float = -math.inf,
    ) -> None:
        self.route_geometry = route_geometry
        self.zone = zone
        self.association_m = max(0.1, association_m)
        self.passed_margin_m = max(0.0, passed_margin_m)
        self.rear_overhang_m = max(0.0, rear_overhang_m)
        self.max_objects = max(1, max_objects)
        self.confirm_frames = max(1, int(confirm_frames))
        self.unconfirmed_memory_sec = max(0.0, unconfirmed_memory_sec)
        self.max_bottom_height_m = max_bottom_height_m
        self.min_top_height_m = min_top_height_m
        self.reset()

    def reset(self) -> None:
        self.kept: dict[int, KeptObject] = {}
        self.next_untracked_key = -1
        self.last_stamp_sec: float | None = None

    def observe(self, objects: PerceptionObjectArray, sensor_pose: Pose2D | None, stamp_sec: float) -> int:
        """Add one perception frame; sensor_pose maps the message frame into the map frame."""
        if sensor_pose is None or (self.last_stamp_sec is not None and stamp_sec <= self.last_stamp_sec):
            return 0
        self.last_stamp_sec = stamp_sec
        added = 0
        for obj in objects.objects:
            if obj.class_id not in OBSTACLE_CLASSES:
                continue
            values = (obj.pose.position.x, obj.pose.position.y, obj.size.x, obj.size.y)
            if not all(math.isfinite(float(v)) for v in values):
                continue
            bottom = float(obj.pose.position.z) - 0.5 * float(obj.size.z)
            top = float(obj.pose.position.z) + 0.5 * float(obj.size.z)
            if (math.isfinite(bottom) and bottom > self.max_bottom_height_m) or (
                    math.isfinite(top) and top < self.min_top_height_m):
                continue
            x, y = sensor_pose.to_map(float(obj.pose.position.x), float(obj.pose.position.y))
            if self.zone is not None and not self.zone.contains(x, y):
                continue
            yaw = sensor_pose.yaw + quaternion_yaw(obj.pose.orientation)
            key = self._key_for(int(obj.track_id), x, y)
            s, _ = project_onto_route(self.route_geometry, x, y)
            previous = self.kept.get(key)
            frames = previous.frames + 1 if previous is not None else 1
            kept = KeptObject(key, deepcopy(obj), x, y, yaw, s, stamp_sec, frames)
            self._drop_absorbed(kept, stamp_sec)
            self.kept[key] = kept
            added += 1
        for key in [key for key, item in self.kept.items()
                    if not self.confirmed(item) and stamp_sec - item.last_seen_sec > self.unconfirmed_memory_sec]:
            del self.kept[key]
        if len(self.kept) > self.max_objects:
            for key in sorted(self.kept, key=lambda k: self.kept[k].last_seen_sec)[:len(self.kept) - self.max_objects]:
                del self.kept[key]
        return added

    def confirmed(self, item: KeptObject) -> bool:
        return item.frames >= self.confirm_frames

    def _drop_absorbed(self, kept: KeptObject, stamp_sec: float) -> None:
        """A far row can first arrive as fragments with short-lived ids that later merge into one
        cluster. A kept object not seen in this frame whose centre lies inside the new box is such
        a fragment and goes away."""
        half_length = 0.5 * float(kept.obj.size.x)
        half_width = 0.5 * float(kept.obj.size.y)
        c, s = math.cos(kept.yaw), math.sin(kept.yaw)
        for key in list(self.kept):
            other = self.kept[key]
            if key == kept.key or other.last_seen_sec >= stamp_sec:
                continue
            dx, dy = other.x - kept.x, other.y - kept.y
            if abs(c * dx + s * dy) <= half_length and abs(-s * dx + c * dy) <= half_width:
                del self.kept[key]

    def _key_for(self, track_id: int, x: float, y: float) -> int:
        if track_id > 0:
            return track_id
        nearest = min(
            (item for item in self.kept.values() if item.key < 0),
            key=lambda item: math.hypot(item.x - x, item.y - y),
            default=None,
        )
        if nearest is not None and math.hypot(nearest.x - x, nearest.y - y) <= self.association_m:
            return nearest.key
        key = self.next_untracked_key
        self.next_untracked_key -= 1
        return key

    def forget_passed(self, rear_axle_s: float) -> None:
        limit = rear_axle_s - self.rear_overhang_m - self.passed_margin_m
        for key in [key for key, item in self.kept.items() if item.route_s < limit]:
            del self.kept[key]

    def local_objects(self, pose: Pose2D, header) -> PerceptionObjectArray:
        """Confirmed kept objects in the frame of a RouteContext whose rear-axle map pose is pose."""
        message = PerceptionObjectArray()
        message.header = deepcopy(header)
        for item in sorted(self.kept.values(), key=lambda kept: kept.route_s):
            if not self.confirmed(item):
                continue
            obj = deepcopy(item.obj)
            obj.pose.position.x, obj.pose.position.y = pose.from_map(item.x, item.y)
            half = 0.5 * (item.yaw - pose.yaw)
            obj.pose.orientation.x = 0.0
            obj.pose.orientation.y = 0.0
            obj.pose.orientation.z = math.sin(half)
            obj.pose.orientation.w = math.cos(half)
            message.objects.append(obj)
        return message
