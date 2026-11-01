"""Generate a lane-bound path through observed obstacle rows.

One tick is observe (rows tracked from the objects, the lane limits) and, only when the obstacle
behavior asks for a new path, search. The search ranks every candidate by one rule (ranked):
inside the lane limits first, then the largest body-obstacle clearance up to obstacle_clearance_m,
then the right-passage margin where it costs no clearance, then the smoothest path. A path outside
the limits is taken only when no candidate stays inside; the fixed lane-edge S is the last resort.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, replace
from itertools import product
import math
from statistics import median

import numpy as np

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import PerceptionObject, PerceptionObjectArray
from kaiev26_decision.obstacle_geometry import box_corners, rectangle_corners


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    s: float
    d: float
    radius: float
    corners: tuple[tuple[float, float], ...] = ()
    footprint_sd: tuple[tuple[float, float], ...] = ()


@dataclass
class Row:
    s: float
    low: float
    high: float
    front: float
    rear: float
    members: int
    right_limit: float | None = None
    left_limit: float | None = None


@dataclass
class TrackedRow:
    samples: deque[Row]
    last_seen_sec: float


@dataclass(frozen=True)
class PathKnot:
    s: float
    d: float
    slope: float
    second_derivative: float = 0.0


@dataclass(frozen=True)
class Scene:
    """One tick's planning input; stations are local (0 at the vehicle's route projection)."""
    points: list[Point]
    stations: list[float]
    lines: list
    boundary_source: str
    obstacles: list[Obstacle]
    rows: tuple[Row, ...]
    # Right and left limit per row (left widened by centerline_departure); None when a row has none.
    limits_by_row: tuple[tuple[float, float], ...] | None
    current_d: float
    initial_slope: float
    route_progress_s: float


@dataclass(frozen=True)
class Placement:
    """Where the passages sit along the rows: station shift of the first and the later rows from
    the row centre, the entry transition length, the return length scale and the early approach
    (reach the first offset before the row instead of turning in beside it)."""
    first_shift: float
    later_shift: float
    transition: float
    return_scale: float = 1.0
    early: bool = False


@dataclass(frozen=True)
class Measured:
    """One candidate path, sampled: its geometry, clearance and how far it leaves the lane."""
    xy: np.ndarray
    curvature: float
    steering_gradient: float
    # Smallest body-obstacle gap, capped at the target clearance; negative is the penetration
    # depth of an overlap [m].
    gap: float
    # How far the body reaches past the lane limits (0 inside) [m].
    corridor: float
    # Smallest gap to the obstacles the path passes on their right, capped at the target
    # clearance plus right_passage_margin [m].
    right_gap: float = math.inf


@dataclass(frozen=True)
class SequencePath:
    points: list[Point]
    curvature: float
    rows: int
    steering_rate_speed_mps: float
    row_stations: tuple[float, ...]
    row_occupancy: tuple[tuple[float, float], ...]
    passage_offsets: tuple[float, ...]
    rejoin_station: float
    # The clearance the path must keep on later ticks to stay committed (its gap less the
    # recheck tolerance) [m]; -inf for the fixed lane-edge S, which is never re-checked.
    clearance_m: float
    # Outside the lane limits, touching an obstacle, or the fixed lane-edge S.
    forced: bool = False
    boundary_source: str = "none"
    # The measured gap (Measured.gap); NaN when the objects were not checked.
    body_clearance_m: float = math.nan
    knots: tuple[PathKnot, ...] = ()
    corridor_excess_m: float = 0.0


class ObstacleSequencePlanner:
    def __init__(
        self,
        *,
        vehicle_width_m: float,
        front_overhang_m: float,
        clearance_m: float,
        wheelbase_m: float = 1.2991017929,
        max_steering_rad: float = 0.4363,
        max_steering_rate_deg_s: float = 60.0,
        steering_gradient_window_m: float = 1.0,
        transition_length_m: float = 10.0,
        return_length_m: float = 10.0,
        row_cluster_tolerance_m: float = 0.8,
        row_pair_min_gap_m: float = 3.5,
        row_pair_max_gap_m: float = 8.5,
        row_pair_nominal_gap_m: float = 6.0,
        row_memory_sec: float = 0.8,
        row_track_match_m: float = 1.25,
        row_history_size: int = 7,
        route_half_width_m: float = 0.0,
        centerline_departure_m: float = 0.0,
        right_passage_margin_m: float = 0.0,
        gap_tolerance_m: float = 0.03,
        recheck_tolerance_m: float = 0.1,
        side_lock_distance_m: float | None = None,
        continuity_max_offset_m: float = 0.5,
    ) -> None:
        self.half_width = vehicle_width_m * 0.5
        self.front = front_overhang_m
        self.rear = max(0.0, 2.60 - front_overhang_m)
        # Target clearance: candidates are ranked by their gap up to this value only.
        self.clearance = max(0.0, clearance_m)
        self.max_curvature = math.tan(max_steering_rad) / wheelbase_m
        self.wheelbase = wheelbase_m
        # <= 0: no steering-rate speed limit (10/23 Midan: it held the bypass at 1-2 m/s).
        self.max_steering_rate = (math.radians(max_steering_rate_deg_s) if max_steering_rate_deg_s > 0.0
                                  else math.inf)
        # The steering gradient is the steering change over this distance, not between neighbouring
        # samples: the 3-point curvature of a 0.25 m path is noisy and its sample-to-sample change
        # read 2-3 times the smooth gradient (10/23 Midan: 0.4-1.0 rad/m against 0.15-0.18).
        self.steering_gradient_window_m = max(0.0, steering_gradient_window_m)
        self.transition_length = max(2.5, transition_length_m)
        self.return_length = max(2.5, return_length_m)
        self.row_cluster_tolerance = max(0.2, row_cluster_tolerance_m)
        self.row_pair_min_gap = max(1.0, row_pair_min_gap_m)
        self.row_pair_max_gap = max(self.row_pair_min_gap, row_pair_max_gap_m)
        self.row_pair_nominal_gap = min(
            self.row_pair_max_gap,
            max(self.row_pair_min_gap, row_pair_nominal_gap_m),
        )
        self.row_memory_sec = max(0.1, row_memory_sec)
        self.row_track_match_m = max(0.2, row_track_match_m)
        self.row_history_size = max(1, row_history_size)
        self.route_half_width = max(0.0, route_half_width_m)
        # The centre line is on the left of the route (right-hand traffic). Every row may be
        # passed with the body up to this far past the left lane limit; the curb side gets none.
        self.centerline_departure = max(0.0, centerline_departure_m)
        # Right steering authority is weaker: a right-side passage wants this much more than the
        # target clearance, where that costs no clearance elsewhere.
        self.right_passage_margin = max(0.0, right_passage_margin_m)
        # Candidates whose gap is within this of the best are equal (also for the right gap).
        self.gap_tolerance = max(0.0, gap_tolerance_m)
        # A committed path stays while its gap is at least the committed gap less this.
        self.recheck_tolerance = max(0.0, recheck_tolerance_m)
        # Passage side per row (route station, +1 left / -1 right), fixed once the row's front is
        # within side_lock_distance (default: the transition length). Every later search passes
        # it on that side, so a row centre read near the route centre cannot flip it
        # (2026-10-17 171607: the side of the first row changed at 360.7 s).
        self.side_lock_distance = (
            self.transition_length if side_lock_distance_m is None else max(0.0, side_lock_distance_m)
        )
        self.row_sides: list[tuple[float, float]] = []
        self.progress_s = 0.0
        # A new path starts with the offset, slope and curvature of the committed path at the car,
        # so a replan does not move the near path (171607 at 369.1 s: 0.72 m to the left, steering
        # +9.2 deg in 0.3 s). Its offset is kept only while the car is within this of it.
        self.continuity_max_offset = max(0.0, continuity_max_offset_m)
        self._followed_knots = None
        self.first_row_station: float | None = None
        self.second_row_station: float | None = None
        self.row_tracks: list[TrackedRow] = []
        self.selected_rows: tuple[Row, ...] = ()
        self.logical_time_sec = 0.0
        self.last_observation_stamp_sec: float | None = None
        # Candidates measured by the last search (for tests and the planning-time breakdown).
        self.last_search_candidates = 0
        self._geometry_by_count = None

    def reset(self) -> None:
        self.row_tracks.clear()
        self.selected_rows = ()
        self.logical_time_sec = 0.0
        self.last_observation_stamp_sec = None
        self.first_row_station = None
        self.second_row_station = None
        self.last_search_candidates = 0
        self._geometry_by_count = None
        self.row_sides = []
        self.progress_s = 0.0
        self._followed_knots = None

    @staticmethod
    def route_samples(points: list[Point]) -> list[float]:
        stations = [0.0]
        for a, b in zip(points, points[1:]):
            stations.append(stations[-1] + math.hypot(b.x - a.x, b.y - a.y))
        return stations

    @staticmethod
    def route_pose(points: list[Point], stations: list[float], s: float) -> tuple[float, float, float]:
        if len(points) < 2:
            raise ValueError("route requires two points")
        i = min(len(points)-2, max(0, bisect_left(stations, s)-1))
        a, b = points[i], points[i + 1]
        span = max(1e-6, stations[i + 1] - stations[i])
        ratio = min(1.0, max(0.0, (s - stations[i]) / span))
        return (a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y), math.atan2(b.y - a.y, b.x - a.x))

    @staticmethod
    def projection_segments(points: list[Point], stations: list[float]):
        segments = []
        for i, (a, b) in enumerate(zip(points, points[1:])):
            dx, dy = b.x-a.x, b.y-a.y
            length2 = dx*dx + dy*dy
            if length2 >= 1e-8:
                if segments:
                    px, py, pdx, pdy, plen2, _, ps = segments[-1]
                    same_line = abs(pdx*dy-pdy*dx) <= 1.0e-10*math.sqrt(plen2*length2)
                    if same_line and pdx*dx+pdy*dy > 0.0:
                        mdx, mdy = b.x-px, b.y-py
                        merged2 = mdx*mdx+mdy*mdy
                        segments[-1] = (px, py, mdx, mdy, merged2, math.sqrt(merged2), ps)
                        continue
                segments.append((a.x, a.y, dx, dy, length2, math.sqrt(length2), stations[i]))
        return segments

    @classmethod
    def project(cls, points: list[Point], stations: list[float], x: float, y: float, *, segments=None) -> tuple[float, float]:
        if segments is None:
            segments = cls.projection_segments(points, stations)
        best = (math.inf, 0.0, 0.0)
        for ax, ay, dx, dy, length2, length, start_s in segments:
            t = min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length2))
            ex, ey = x - ax - t * dx, y - ay - t * dy
            distance2 = ex * ex + ey * ey
            if distance2 < best[0]:
                best = (distance2, start_s + t * length, (-dy * ex + dx * ey) / length)
        return best[1], best[2]

    def visible_obstacles(
        self, points: list[Point], stations: list[float], objects: PerceptionObjectArray, lines,
        horizon_s: float | None = None,
    ) -> list[Obstacle]:
        """horizon_s: objects whose far end lies beyond this local station are left out."""
        selected = []
        segments = self.projection_segments(points, stations)
        for obj in objects.objects:
            if obj.class_id not in {PerceptionObject.CLASS_OBSTACLE, PerceptionObject.CLASS_TRAFFIC_CONE}:
                continue
            x, y = float(obj.pose.position.x), float(obj.pose.position.y)
            corners = box_corners(obj)
            if not corners:
                continue
            radius = 0.5 * math.hypot(float(obj.size.x), float(obj.size.y))
            if not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0.0:
                continue
            s, d = self.project(points, stations, x, y, segments=segments)
            projected_x, projected_y, yaw = self.route_pose(points, stations, s)
            c, sn = math.cos(yaw), math.sin(yaw)
            if s <= 1.0e-6 or s >= stations[-1]-1.0e-6:
                s += c*(x-projected_x)+sn*(y-projected_y)
            footprint_sd = tuple((s+c*(cx-x)+sn*(cy-y), d-sn*(cx-x)+c*(cy-y)) for cx, cy in corners)
            low_s = min(p[0] for p in footprint_sd)
            high_s = max(p[0] for p in footprint_sd)
            if high_s < -self.rear or low_s > stations[-1] + self.front:
                continue
            if horizon_s is not None and high_s > horizon_s:
                continue
            limits = self.lane_limits(lines, s)
            if limits is None and self.route_half_width > 0.0:
                # No lane limits here: the configured corridor, so roadside posts in the next
                # lane are not taken for a drum row (2026-10-17 final).
                limits = -self.route_half_width, self.route_half_width
            if limits is None or (
                max(p[1] for p in footprint_sd) >= limits[0]
                and min(p[1] for p in footprint_sd) <= limits[1]
            ):
                selected.append(Obstacle(x, y, s, d, radius, corners, footprint_sd))
        return sorted(selected, key=lambda obj: obj.s)

    def rows(self, obstacles: list[Obstacle]) -> list[Row]:
        clusters: list[list[Obstacle]] = []
        for obstacle in sorted(obstacles, key=lambda item: item.s):
            nearest = None
            nearest_distance = math.inf
            for cluster in clusters:
                center = sum(item.s for item in cluster) / len(cluster)
                distance = abs(obstacle.s - center)
                if distance <= self.row_cluster_tolerance and distance < nearest_distance:
                    nearest = cluster
                    nearest_distance = distance
            if nearest is None:
                clusters.append([obstacle])
            else:
                nearest.append(obstacle)

        rows = []
        for cluster in clusters:
            rows.append(Row(
                s=sum(item.s for item in cluster) / len(cluster),
                low=min(min(p[1] for p in item.footprint_sd) if item.footprint_sd else item.d-item.radius for item in cluster),
                high=max(max(p[1] for p in item.footprint_sd) if item.footprint_sd else item.d+item.radius for item in cluster),
                front=min(min(p[0] for p in item.footprint_sd) if item.footprint_sd else item.s-item.radius for item in cluster),
                rear=max(max(p[0] for p in item.footprint_sd) if item.footprint_sd else item.s+item.radius for item in cluster),
                members=len(cluster),
            ))
        return sorted(rows, key=lambda row: row.s)

    def row_obstacles(self, obstacles: list[Obstacle], lines) -> list[Obstacle]:
        selected = []
        for obj in obstacles:
            if not obj.footprint_sd:
                continue
            ss, ds = zip(*obj.footprint_sd)
            limits = self.lane_limits(lines, obj.s)
            if limits is not None and (max(ds) < limits[0] or min(ds) > limits[1]):
                continue
            # A longitudinal roadside cluster is still a collision object, not a row.
            if max(ss)-min(ss) > max(max(ds)-min(ds), 2.0*self.row_cluster_tolerance):
                continue
            selected.append(obj)
        return selected

    def observation_time(
        self,
        objects: PerceptionObjectArray | None,
        fallback_sec: float | None,
    ) -> float:
        if fallback_sec is not None and math.isfinite(fallback_sec) and fallback_sec > 0.0:
            self.logical_time_sec = max(self.logical_time_sec, fallback_sec)
            return self.logical_time_sec
        if objects is not None:
            stamp = float(objects.header.stamp.sec) + 1.0e-9 * float(objects.header.stamp.nanosec)
            if math.isfinite(stamp) and stamp > 0.0:
                self.logical_time_sec = max(self.logical_time_sec, stamp)
                return self.logical_time_sec
        self.logical_time_sec += 0.05
        return self.logical_time_sec

    @staticmethod
    def global_row(row: Row, route_progress_s: float) -> Row:
        return Row(
            s=route_progress_s + row.s,
            low=row.low,
            high=row.high,
            front=route_progress_s + row.front,
            rear=route_progress_s + row.rear,
            members=row.members,
            right_limit=row.right_limit,
            left_limit=row.left_limit,
        )

    @staticmethod
    def aggregate_track(track: TrackedRow) -> Row:
        samples = list(track.samples)
        right_limits = [row.right_limit for row in samples if row.right_limit is not None]
        left_limits = [row.left_limit for row in samples if row.left_limit is not None]
        return Row(
            s=median(row.s for row in samples),
            low=median(row.low for row in samples),
            high=median(row.high for row in samples),
            front=median(row.front for row in samples),
            rear=median(row.rear for row in samples),
            members=max(row.members for row in samples),
            right_limit=median(right_limits) if right_limits else None,
            left_limit=median(left_limits) if left_limits else None,
        )

    def tracked_rows(
        self,
        observed_rows: list[Row],
        route_progress_s: float,
        now_sec: float,
        route_length_m: float,
        observation_stamp_sec: float | None = None,
    ) -> list[Row]:
        retained = []
        for track in self.row_tracks:
            row = self.aggregate_track(track)
            fresh = now_sec - track.last_seen_sec <= self.row_memory_sec
            not_passed = row.rear >= route_progress_s - self.rear - 0.5
            if fresh and not_passed:
                retained.append(track)
        self.row_tracks = retained

        measured_time = now_sec
        if observation_stamp_sec is not None and math.isfinite(observation_stamp_sec) and observation_stamp_sec > 0.0:
            measured_time = observation_stamp_sec
            if self.last_observation_stamp_sec is not None and measured_time <= self.last_observation_stamp_sec:
                observed_rows = []
            else:
                self.last_observation_stamp_sec = measured_time

        used_tracks: set[int] = set()
        for observed in observed_rows:
            candidate = self.global_row(observed, route_progress_s)
            match_index = None
            match_distance = math.inf
            for index, track in enumerate(self.row_tracks):
                if index in used_tracks:
                    continue
                previous = self.aggregate_track(track)
                lateral_gap = max(previous.low-candidate.high, candidate.low-previous.high, 0.0)
                if lateral_gap > self.row_cluster_tolerance:
                    continue
                if (previous.low+previous.high)*(candidate.low+candidate.high) < 0.0 and lateral_gap > 0.0:
                    continue
                distance = abs(previous.s - candidate.s)
                if distance <= self.row_track_match_m and distance < match_distance:
                    match_index = index
                    match_distance = distance
            if match_index is None:
                self.row_tracks.append(TrackedRow(
                    samples=deque([candidate], maxlen=self.row_history_size),
                    last_seen_sec=measured_time,
                ))
                used_tracks.add(len(self.row_tracks) - 1)
            else:
                track = self.row_tracks[match_index]
                if candidate.right_limit is None or candidate.left_limit is None:
                    # The camera loses the lane next to a near row; keep the limits it measured.
                    known = self.aggregate_track(track)
                    if known.right_limit is not None and known.left_limit is not None:
                        candidate = replace(candidate, right_limit=known.right_limit,
                                            left_limit=known.left_limit)
                track.samples.append(candidate)
                track.last_seen_sec = measured_time
                used_tracks.add(match_index)

        local_rows = []
        for track in self.row_tracks:
            row = self.aggregate_track(track)
            local = Row(
                s=row.s - route_progress_s,
                low=row.low,
                high=row.high,
                front=row.front - route_progress_s,
                rear=row.rear - route_progress_s,
                members=row.members,
                right_limit=row.right_limit,
                left_limit=row.left_limit,
            )
            if local.rear >= -self.rear - 0.5 and local.front <= route_length_m + self.front:
                local_rows.append(local)
        return sorted(local_rows, key=lambda row: row.s)

    def select_rows(self, rows: list[Row], *, route_progress_s: float | None = None) -> list[Row]:
        rows = [row for row in rows if row.rear >= 0.0]
        if len(rows) <= 1:
            return rows

        if route_progress_s is not None and self.second_row_station is not None:
            matched = []
            for station in (self.first_row_station, self.second_row_station):
                previous = next((row for row in self.selected_rows
                                 if station is not None and abs(row.s-station) < 1.0e-6), None)
                choices = [row for row in rows if station is not None and
                           abs(route_progress_s+row.s-station) <= self.row_track_match_m and
                           (previous is None or min(previous.high, row.high) >= max(previous.low, row.low))]
                if choices:
                    matched.append(min(choices, key=lambda row: abs(route_progress_s+row.s-station)))
            if len(matched) == 2 and matched[0] is not matched[1]:
                if (matched[0].low+matched[0].high)*(matched[1].low+matched[1].high) < 0.0:
                    return sorted(matched, key=lambda row: row.s)
            if len(matched) == 1 and abs(route_progress_s+matched[0].s-self.second_row_station) <= self.row_track_match_m:
                return matched

        candidates = []
        for first_index, first in enumerate(rows[:-1]):
            for second in rows[first_index + 1:]:
                gap = second.s - first.s
                opposite_sides = (first.low + first.high) * (second.low + second.high) < 0.0
                if opposite_sides and self.row_pair_min_gap <= gap <= self.row_pair_max_gap:
                    score = (
                        abs(gap - self.row_pair_nominal_gap)
                        + 0.05 * max(0.0, first.s)
                        - 0.15 * (first.members + second.members)
                    )
                    candidates.append((score, first, second))
        if not candidates:
            return rows[:1]
        _, first, second = min(candidates, key=lambda item: item[0])
        return [first, second]

    @staticmethod
    def lane_limits(lines, station: float) -> tuple[float, float] | None:
        values = []
        for samples in lines:
            if not samples[0][0] - 0.5 <= station <= samples[-1][0] + 0.5:
                continue
            for a, b in zip(samples, samples[1:]):
                if a[0] <= station <= b[0]:
                    t = (station - a[0]) / max(1e-6, b[0] - a[0])
                    values.append(a[1] + t * (b[1] - a[1]))
                    break
        left = [value for value in values if value > 0.3]
        right = [value for value in values if value < -0.3]
        if not left or not right:
            return None
        return max(right), min(left)

    @staticmethod
    def lane_limits_many(lines, stations) -> tuple[np.ndarray, np.ndarray]:
        """lane_limits at many stations at once; NaN on both sides where it returns None."""
        stations = np.asarray(stations, dtype=float)
        right = np.full(stations.shape, -np.inf)
        left = np.full(stations.shape, np.inf)
        for samples in lines:
            if len(samples) < 2:
                continue
            line = np.asarray(samples, dtype=float)
            s, d = line[:, 0], line[:, 1]
            # First segment whose end reaches the station, as the scalar loop finds it.
            index = np.minimum(np.searchsorted(s[1:], stations, side="left"), len(s)-2)
            start, end = s[index], s[index+1]
            covered = (start <= stations) & (stations <= end)
            t = (stations-start)/np.maximum(1e-6, end-start)
            values = d[index]+t*(d[index+1]-d[index])
            right = np.where(covered & (values < -0.3), np.maximum(right, values), right)
            left = np.where(covered & (values > 0.3), np.minimum(left, values), left)
        missing = np.isinf(right) | np.isinf(left)
        return np.where(missing, np.nan, right), np.where(missing, np.nan, left)

    # Path geometry -------------------------------------------------------------------------

    @staticmethod
    def quintic_segment(station: float, start: PathKnot, end: PathKnot) -> float:
        length = max(1.0e-6, end.s - start.s)
        t = min(1.0, max(0.0, (station - start.s) / length))
        t2, t3, t4, t5 = t*t, t*t*t, t*t*t*t, t*t*t*t*t
        h00 = 1.0 - 10.0*t3 + 15.0*t4 - 6.0*t5
        h10 = t - 6.0*t3 + 8.0*t4 - 3.0*t5
        h20 = 0.5 * (t2 - 3.0*t3 + 3.0*t4 - t5)
        h01 = 10.0*t3 - 15.0*t4 + 6.0*t5
        h11 = -4.0*t3 + 7.0*t4 - 3.0*t5
        h21 = 0.5 * (t3 - 2.0*t4 + t5)
        return (
            h00 * start.d
            + h10 * length * start.slope
            + h20 * length * length * start.second_derivative
            + h01 * end.d
            + h11 * length * end.slope
            + h21 * length * length * end.second_derivative
        )

    @classmethod
    def offset(cls, station: float, knots: list[PathKnot]) -> float:
        if station <= knots[0].s:
            return knots[0].d
        for start, end in zip(knots, knots[1:]):
            if station <= end.s:
                return cls.quintic_segment(station, start, end)
        return knots[-1].d

    @staticmethod
    def make_knots(
        stations_and_offsets: list[tuple[float, float]], initial_slope: float = 0.0,
    ) -> list[PathKnot]:
        deduplicated = []
        for station, offset in stations_and_offsets:
            if deduplicated and station <= deduplicated[-1][0] + 1.0e-3:
                deduplicated[-1] = (station, offset)
            else:
                deduplicated.append((station, offset))

        if len(deduplicated) == 1:
            station, offset = deduplicated[0]
            return [PathKnot(station, offset, 0.0)]

        intervals = [
            max(1.0e-6, following[0] - current[0])
            for current, following in zip(deduplicated, deduplicated[1:])
        ]
        lower = [0.0] * len(deduplicated)
        diagonal = [0.0] * len(deduplicated)
        upper = [0.0] * len(deduplicated)
        right_hand = [0.0] * len(deduplicated)

        first_interval = intervals[0]
        first_secant = (deduplicated[1][1] - deduplicated[0][1]) / first_interval
        diagonal[0] = 2.0 * first_interval
        upper[0] = first_interval
        right_hand[0] = 6.0 * (first_secant-initial_slope)

        for index in range(1, len(deduplicated) - 1):
            previous_interval = intervals[index - 1]
            following_interval = intervals[index]
            previous_secant = (
                deduplicated[index][1] - deduplicated[index - 1][1]
            ) / previous_interval
            following_secant = (
                deduplicated[index + 1][1] - deduplicated[index][1]
            ) / following_interval
            lower[index] = previous_interval
            diagonal[index] = 2.0 * (previous_interval + following_interval)
            upper[index] = following_interval
            right_hand[index] = 6.0 * (following_secant - previous_secant)

        last_interval = intervals[-1]
        last_secant = (deduplicated[-1][1] - deduplicated[-2][1]) / last_interval
        lower[-1] = last_interval
        diagonal[-1] = 2.0 * last_interval
        right_hand[-1] = -6.0 * last_secant

        for index in range(1, len(deduplicated)):
            factor = lower[index] / diagonal[index - 1]
            diagonal[index] -= factor * upper[index - 1]
            right_hand[index] -= factor * right_hand[index - 1]
        second_derivatives = [0.0] * len(deduplicated)
        second_derivatives[-1] = right_hand[-1] / diagonal[-1]
        for index in range(len(deduplicated) - 2, -1, -1):
            second_derivatives[index] = (
                right_hand[index] - upper[index] * second_derivatives[index + 1]
            ) / diagonal[index]

        slopes = [initial_slope]
        for index in range(1, len(deduplicated) - 1):
            interval = intervals[index - 1]
            secant = (
                deduplicated[index][1] - deduplicated[index - 1][1]
            ) / interval
            slopes.append(
                secant
                + interval * (
                    second_derivatives[index - 1]
                    + 2.0 * second_derivatives[index]
                ) / 6.0
            )
        slopes.append(0.0)

        return [
            PathKnot(station, offset, slope, curvature)
            for (station, offset), slope, curvature in zip(
                deduplicated, slopes, second_derivatives
            )
        ]

    def route_frame(self, points: list[Point], stations: list[float], spacing: float):
        """Route samples (s, x, y, cos, sin) about spacing apart and the merged route segments,
        cached for this route (the scene's points) until the next observe."""
        count = max(2, math.ceil(stations[-1] / spacing))
        cache = self._geometry_by_count
        if cache is None or cache.get("points") is not points:
            cache = self._geometry_by_count = {"points": points}
        if "segments" not in cache:
            cache["segments"] = np.asarray(self.projection_segments(points, stations))
        if count not in cache:
            samples = []
            for i in range(count + 1):
                s = min(stations[-1], i * stations[-1] / count)
                x, y, yaw = self.route_pose(points, stations, s)
                samples.append((s, x, y, math.cos(yaw), math.sin(yaw)))
            cache[count] = np.asarray(samples)
        return cache[count], cache["segments"]

    def trace_many(self, points: list[Point], stations: list[float], knots_list, spacing: float = 0.25):
        """Every candidate (knot list) sampled along the route at once: offsets (C×N), xy
        (C×N×2), largest |curvature| (C), steering gradient [rad/m] (C), heading cos and sin
        (C×N) and the body rectangle at every sample (C×N×4×2)."""
        samples, _ = self.route_frame(points, stations, spacing)
        s = samples[:, 0]
        count = len(knots_list)
        size = max(len(knots) for knots in knots_list)
        ks = np.full((count, size), np.inf)
        kd = np.zeros((count, size))
        kp = np.zeros((count, size))
        kq = np.zeros((count, size))
        last = np.zeros(count, dtype=int)
        for i, knots in enumerate(knots_list):
            n = len(knots)
            ks[i, :n] = [k.s for k in knots]
            kd[i, :n] = [k.d for k in knots]
            kp[i, :n] = [k.slope for k in knots]
            kq[i, :n] = [k.second_derivative for k in knots]
            last[i] = n-1
        rows = np.arange(count)[:, None]
        # Segment i holds the stations in (s_i, s_i+1], as offset() reads the knots.
        index = (s[None, None, :] > ks[:, :, None]).sum(axis=1)-1
        last_d = kd[np.arange(count), last][:, None]
        if size >= 2:
            start = np.clip(index, 0, size-2)
            s0, s1 = ks[rows, start], ks[rows, start+1]
            length = np.where(np.isfinite(s1), s1-s0, 1.0)
            t = np.clip((s[None, :]-s0)/np.maximum(length, 1.0e-9), 0.0, 1.0)
            t2, t3 = t*t, t*t*t
            t4, t5 = t3*t, t3*t2
            quintic = (
                (1-10*t3+15*t4-6*t5)*kd[rows, start]
                + (t-6*t3+8*t4-3*t5)*length*kp[rows, start]
                + 0.5*(t2-3*t3+3*t4-t5)*length**2*kq[rows, start]
                + (10*t3-15*t4+6*t5)*kd[rows, start+1]
                + (-4*t3+7*t4-3*t5)*length*kp[rows, start+1]
                + 0.5*(t3-2*t4+t5)*length**2*kq[rows, start+1]
            )
            offsets = np.where(index < 0, kd[:, :1], np.where(index >= last[:, None], last_d, quintic))
        else:
            offsets = np.repeat(kd[:, :1], len(s), axis=1)
        normal = np.column_stack((-samples[:, 4], samples[:, 3]))
        xy = samples[None, :, 1:3]+normal[None]*offsets[..., None]
        ab, bc = xy[:, 1:-1]-xy[:, :-2], xy[:, 2:]-xy[:, 1:-1]
        denominator = (np.linalg.norm(ab, axis=2)*np.linalg.norm(bc, axis=2)
                       * np.linalg.norm(ab+bc, axis=2))
        curvature = np.divide(2*(ab[..., 0]*bc[..., 1]-ab[..., 1]*bc[..., 0]), denominator,
                              out=np.zeros(denominator.shape), where=denominator > 1e-8)
        max_curvature = np.max(np.abs(curvature), axis=1, initial=0.0)
        steering = np.arctan(self.wheelbase*curvature)
        step = s[-1]/(len(s)-1) if len(s) > 1 else 1.0
        window = int(min(max(1, round(self.steering_gradient_window_m/step)), max(1, steering.shape[1]-1)))
        gradient = (np.max(np.abs(steering[:, window:]-steering[:, :-window]), axis=1, initial=0.0)/(window*step)
                    if steering.shape[1] > window else np.zeros(len(steering)))
        n = xy.shape[1]
        tangents = xy[:, np.minimum(np.arange(n)+1, n-1)]-xy[:, np.maximum(np.arange(n)-1, 0)]
        yaw = np.arctan2(tangents[..., 1], tangents[..., 0])
        c, sn = np.cos(yaw), np.sin(yaw)
        corners = np.array([[-self.rear, -self.half_width], [self.front, -self.half_width],
                            [self.front, self.half_width], [-self.rear, self.half_width]])
        body = xy[:, :, None, :]+np.stack(
            (c[..., None]*corners[:, 0]-sn[..., None]*corners[:, 1],
             sn[..., None]*corners[:, 0]+c[..., None]*corners[:, 1]), axis=3)
        return offsets, xy, max_curvature, gradient, (c, sn), body

    @staticmethod
    def corner_stations(body, samples):
        """(s, d) of every body corner on the route, from a route frame at its own sample: the
        heading over the polyline 1 m either side and the circle of the local curvature. Exact
        on a straight route, within a few cm of the polyline projection on a 10 m radius."""
        n = len(samples)
        step = max(1, int(round(1.0/max(1.0e-6, samples[-1, 0]/max(1, n-1)))))
        ahead, behind = np.minimum(np.arange(n)+step, n-1), np.maximum(np.arange(n)-step, 0)
        heading = np.unwrap(np.arctan2(samples[ahead, 2]-samples[behind, 2],
                                       samples[ahead, 1]-samples[behind, 1]))
        span = samples[ahead, 0]-samples[behind, 0]
        curvature = np.divide(heading[ahead]-heading[behind], span, out=np.zeros(n), where=span > 1.0e-6)
        cos, sin = np.cos(heading)[None, :, None], np.sin(heading)[None, :, None]
        delta = body-samples[None, :, None, 1:3]
        along = delta[..., 0]*cos+delta[..., 1]*sin
        across = -delta[..., 0]*sin+delta[..., 1]*cos
        # On the circle of the sample's curvature (a straight line when it is zero).
        k = curvature[None, :, None]
        lateral = (2.0*across-k*(across*across+along*along))/(
            1.0+np.sqrt((1.0-k*across)**2+(k*along)**2))
        safe = np.where(np.abs(k) > 1.0e-9, k, 1.0)
        station = np.where(np.abs(k) > 1.0e-9, np.arctan2(k*along, 1.0-k*across)/safe, along)
        return samples[None, :, None, 0]+station, lateral

    def corridor_excess_many(self, corner_s, corner_d, lines, rows, boundary_source: str,
                             margin: float = 0.05) -> np.ndarray:
        """How far [m] each candidate's farthest corner (first axis) reaches past the corridor
        less margin (0 inside, inf unknown). The left (centre-line) side is widened by
        centerline_departure."""
        flat_s, flat_d = corner_s.reshape(-1), corner_d.reshape(-1)
        if boundary_source == "configured_route_corridor":
            right = np.full(len(flat_s), -self.route_half_width)
            left = np.full(len(flat_s), self.route_half_width)
        else:
            right, left = self.lane_limits_many(lines, flat_s)
            missing = np.isnan(right)
            if missing.any():
                fallback = self.fallback_lane_limits(rows)
                if fallback is None and self.route_half_width > 0.0:
                    fallback = -self.route_half_width, self.route_half_width
                if fallback is None:
                    return np.full(corner_s.shape[0], math.inf)
                right = np.where(missing, fallback[0], right)
                left = np.where(missing, fallback[1], left)
        left = left+self.centerline_departure
        excess = np.maximum(right+margin-flat_d, flat_d-(left-margin)).reshape(corner_s.shape[0], -1)
        return np.maximum(0.0, np.max(excess, axis=1, initial=0.0))

    @staticmethod
    def project_many(xy, segments):
        ax, ay, dx, dy, length2, length, start_s = segments.T
        ex, ey = xy[:, 0, None]-ax, xy[:, 1, None]-ay
        t = np.clip((ex*dx+ey*dy)/length2, 0.0, 1.0)
        ex, ey = ex-t*dx, ey-t*dy
        closest = np.argmin(ex*ex+ey*ey, axis=1)
        indexes = np.arange(len(xy)), closest
        return ((start_s+t*length)[indexes], ((-dy*ex+dx*ey)/length)[indexes])

    @staticmethod
    def polygon_gaps(bodies, obstacle) -> np.ndarray:
        """Signed gap of each body rectangle (B×4×2) to one convex obstacle polygon (P×2): the
        exact distance when apart, minus the penetration depth (smallest SAT overlap) when they
        overlap."""
        bodies = np.asarray(bodies, dtype=float)
        obstacle = np.asarray(obstacle, dtype=float)
        other = np.broadcast_to(obstacle, (len(bodies),)+obstacle.shape)
        edges = np.concatenate((np.roll(bodies, -1, axis=1)-bodies,
                                np.roll(other, -1, axis=1)-other), axis=1)
        axes = np.stack((-edges[:, :, 1], edges[:, :, 0]), axis=2)
        axes = axes/np.maximum(np.linalg.norm(axes, axis=2, keepdims=True), 1.0e-12)
        a = np.einsum('npd,nad->nap', bodies, axes)
        b = np.einsum('npd,nad->nap', other, axes)
        separation = np.maximum(a.min(axis=2)-b.max(axis=2), b.min(axis=2)-a.max(axis=2)).max(axis=1)
        distance2 = np.full(len(bodies), np.inf)
        for vertices, starts in ((bodies, other), (other, bodies)):
            edge = np.roll(starts, -1, axis=1)-starts
            delta = vertices[:, :, None, :]-starts[:, None, :, :]
            t = np.clip(np.sum(delta*edge[:, None, :, :], axis=3)
                        / np.maximum(np.sum(edge*edge, axis=2)[:, None, :], 1e-12), 0.0, 1.0)
            residual = delta-t[:, :, :, None]*edge[:, None, :, :]
            distance2 = np.minimum(distance2, np.sum(residual*residual, axis=3).min(axis=(1, 2)))
        return np.where(separation <= 0.0, separation, np.sqrt(distance2))

    @staticmethod
    def fallback_lane_limits(rows: list[Row]) -> tuple[float, float] | None:
        right = [row.right_limit for row in rows if row.right_limit is not None]
        left = [row.left_limit for row in rows if row.left_limit is not None]
        if not right or not left:
            return None
        return median(right), median(left)

    def missing_row_obstacles(self, points, stations, rows, observed_rows):
        remembered = []
        for row in rows:
            if any(abs(row.s-observed.s) <= self.row_track_match_m and
                   min(row.high, observed.high) >= max(row.low, observed.low)
                   for observed in observed_rows):
                continue
            s, d = 0.5*(row.front+row.rear), 0.5*(row.low+row.high)
            x, y, yaw = self.route_pose(points, stations, max(0.0, s))
            x += math.cos(yaw)*min(0.0, s)-math.sin(yaw)*d
            y += math.sin(yaw)*min(0.0, s)+math.cos(yaw)*d
            length, width = max(0.01, row.rear-row.front), max(0.01, row.high-row.low)
            corners = rectangle_corners(x, y, yaw, -length/2, length/2, width/2)
            sd = ((row.front, row.low), (row.rear, row.low),
                  (row.rear, row.high), (row.front, row.high))
            remembered.append(Obstacle(x, y, s, d, 0.5*math.hypot(length, width), corners, sd))
        return remembered

    # Candidates ------------------------------------------------------------------------------

    def locked_side(self, row: Row) -> float | None:
        """The fixed passage side of this row (+1 left, -1 right), None while still free."""
        station = self.progress_s+row.s
        for locked_s, side in self.row_sides:
            if abs(locked_s-station) <= self.row_track_match_m:
                return side
        return None

    def lock_row_sides(self, rows, limits_by_row) -> None:
        """Fix the passage side of each row once its front is within side_lock_distance."""
        for row, limits in zip(rows, limits_by_row):
            if row.front > self.side_lock_distance or self.locked_side(row) is not None:
                continue
            self.row_sides.append((self.progress_s+row.s, self.passage_side(row, limits)))

    def passage_side(self, row: Row, limits: tuple[float, float]) -> float:
        """+1 to pass the row on its left, -1 on its right: the locked side, else the open side
        (D13)."""
        locked = self.locked_side(row)
        if locked is not None:
            return locked
        center = 0.5*(row.low+row.high)
        if center < 0.0:
            return 1.0
        if center > 0.0:
            return -1.0
        return 1.0 if limits[1]-row.high >= row.low-limits[0] else -1.0

    def row_offsets(self, row: Row, limits: tuple[float, float]) -> list[float]:
        """Lateral offsets to pass the row at, all on its open side, so no candidate runs through
        the opposite lane and alternating rows always give an S.

        The lane edge keeps the most clearance, the offset that just keeps the target clearance
        the least lateral travel; the ones between trade one for the other where no S keeps the
        target. Where the route centre already keeps the target, it is the near end instead. Where
        the row reaches the lane edge only the edge is left (the path then touches the row).
        """
        side = self.passage_side(row, limits)
        right_edge, left_edge = limits
        if side > 0.0:
            edge, touch, margin = left_edge-self.half_width-0.05, row.high+self.half_width, 0.0
        else:
            edge = right_edge+self.half_width+0.05
            touch, margin = row.low-self.half_width, self.right_passage_margin
        room = side*(edge-touch)
        if room <= 0.0:
            return [edge]
        nominal = touch+side*min(room, self.clearance+margin)
        if side*nominal <= 0.0 <= side*edge:
            offsets = [0.0, 0.5*edge, edge]
        else:
            # Above the target too: the body turning between rows comes closer than the offset,
            # and a wide lane needs one between those and the edge (a third object further on).
            wanted = {k*self.clearance for k in (1.6, 1.3, 1.0, 0.7, 0.4)} | {self.clearance+margin}
            gaps = [room]+[gap for gap in wanted if gap < room]
            if room > max(wanted)+0.3:
                gaps.append(0.5*(room+max(wanted)))
            offsets = [touch+side*gap for gap in gaps]
        return sorted(set(round(offset, 6) for offset in offsets))

    def placements(self, rows) -> list[Placement]:
        """Passage placements tied to the vehicle footprint, not to barrel size or a fixed gap.
        The first is the default the offsets are searched with."""
        f, t = self.front, self.transition_length
        options = [
            Placement(0.0, 0.0, t), Placement(-0.5*f, -0.5*f, t), Placement(-f, -0.5*f, t),
            Placement(0.0, self.rear, t), Placement(-0.5*f, 0.0, t, 1.5), Placement(0.0, 0.0, t, 1.5),
            Placement(0.0, 0.0, 0.5*t), Placement(-0.5*f, -0.5*f, 0.5*t),
        ]
        if rows[0].front > t:
            options += [replace(option, early=True) for option in options[:6]]
        return options

    def knots_for(self, scene: Scene, offsets, placement: Placement,
                  steering_rad: float | None) -> list[PathKnot] | None:
        rows = scene.rows
        passage = [row.s+(placement.first_shift if i == 0 else placement.later_shift)
                   for i, row in enumerate(rows)]
        rejoin = min(scene.stations[-1], rows[-1].rear+self.return_length*placement.return_scale)
        if rejoin <= rows[-1].rear+1.0:
            return None
        controls = [(0.0, scene.current_d)]
        entry = max(0.0, passage[0]-placement.transition)
        if entry > self.wheelbase:
            controls.append((entry, scene.current_d))
        for s, d in zip(passage, offsets):
            if s > controls[-1][0]+0.25:
                controls.append((s, d))
        controls.append((rejoin, 0.0))
        if not placement.early:
            return self.continue_start(scene, self.make_knots(controls, initial_slope=scene.initial_slope))
        # Reach the open side before the body enters the row, without delaying the reverse
        # turn until the first row has been completely passed.
        alignment = passage[0]-2.0*self.front
        if alignment <= 2.0*self.wheelbase:
            return None
        tail_controls = [(s, d) for s, d in [(alignment, offsets[0])]+controls[1:] if s >= alignment]
        if len(tail_controls) < 2:
            return None
        tail = self.make_knots(tail_controls, initial_slope=0.0)
        tail[0] = replace(tail[0], slope=0.0, second_derivative=0.0)
        initial_curvature = math.tan(steering_rad)/self.wheelbase if steering_rad is not None else 0.0
        slope = scene.initial_slope
        return [PathKnot(0.0, scene.current_d, slope, initial_curvature*(1+slope**2)**1.5)]+tail

    def continue_start(self, scene: Scene, knots: list[PathKnot]) -> list[PathKnot]:
        """Start a new path with the offset, slope and curvature of the committed path here.

        Only while a committed path covers the car's station. Its offset is kept while the car is
        within continuity_max_offset of it; otherwise the path starts at the car with the
        committed slope and curvature.
        """
        followed = self._followed_knots
        station = scene.route_progress_s
        if not followed or not knots or abs(knots[0].s) > 1.0e-6 or not followed[0].s <= station < followed[-1].s:
            return knots
        h = 0.1
        reference = list(followed)
        d0 = self.offset(station, reference)
        ahead, behind = self.offset(station+h, reference), self.offset(station-h, reference)
        offset = d0 if abs(d0-scene.current_d) <= self.continuity_max_offset else knots[0].d
        return [
            replace(knots[0], d=offset, slope=(ahead-behind)/(2.0*h),
                    second_derivative=(ahead-2.0*d0+behind)/(h*h)),
            *knots[1:],
        ]

    def measure_many(self, scene: Scene, knots_list, spacing: float = 0.25,
                     curvature_limit: float | None = None) -> list[Measured | None]:
        """Every candidate sampled and measured at once; None past the curvature limit or where
        there is no lane limit at all."""
        results: list[Measured | None] = [None]*len(knots_list)
        if not knots_list:
            return results
        offsets, xy, max_curvature, gradient, heading, body = self.trace_many(
            scene.points, scene.stations, knots_list, spacing)
        limit = self.max_curvature if curvature_limit is None else curvature_limit
        index = np.flatnonzero(np.isfinite(xy).all(axis=(1, 2)) & (max_curvature <= limit))
        if not len(index):
            return results
        samples, _ = self.route_frame(scene.points, scene.stations, spacing)
        corner_s, corner_d = self.corner_stations(body[index], samples)
        corridor = self.corridor_excess_many(corner_s, corner_d, scene.lines, list(scene.rows),
                                             scene.boundary_source)
        gaps, right_gaps = self.body_gaps(samples[:, 0], offsets[index], xy[index], heading[0][index],
                                          heading[1][index], body[index], scene.obstacles)
        for k, i in enumerate(index):
            if math.isfinite(corridor[k]):
                results[i] = Measured(xy[i], float(max_curvature[i]), float(gradient[i]),
                                      float(gaps[k]), float(corridor[k]), float(right_gaps[k]))
        return results

    def measure(self, scene: Scene, knots, spacing: float = 0.25,
                curvature_limit: float | None = None) -> Measured | None:
        return self.measure_many(scene, [knots], spacing, curvature_limit)[0]

    def body_gaps(self, s, offsets, xy, c, sn, body, obstacles) -> tuple[np.ndarray, np.ndarray]:
        """Per candidate, the smallest gap between the body along the path and any obstacle,
        capped at the target clearance, and the smallest gap to the obstacles it passes on their
        right (they lie left of the path), capped at the target plus right_passage_margin."""
        cap = self.clearance
        right_cap = self.clearance+self.right_passage_margin
        best = np.full(len(xy), cap)
        best_right = np.full(len(xy), right_cap)
        for obj in obstacles:
            column = min(len(s)-1, int(np.searchsorted(s, obj.s)))
            right = obj.d > offsets[:, column]
            dx, dy = obj.x-xy[..., 0], obj.y-xy[..., 1]
            ox, oy = c*dx+sn*dy, -sn*dx+c*dy
            # Distance to the obstacle centre less its radius is never more than the true gap,
            # so only samples that may beat the current best are measured exactly.
            distances = np.hypot(ox-np.clip(ox, -self.rear, self.front),
                                 oy-np.clip(oy, -self.half_width, self.half_width))
            bound = np.where(right, np.maximum(best, best_right), best)
            candidate, sample = np.nonzero(distances-obj.radius < bound[:, None])
            if not len(candidate):
                continue
            if obj.corners:
                gaps = self.polygon_gaps(body[candidate, sample], obj.corners)
            else:
                gaps = distances[candidate, sample]-obj.radius
            np.minimum.at(best, candidate, gaps)
            on_right = right[candidate]
            np.minimum.at(best_right, candidate[on_right], gaps[on_right])
        return np.minimum(best, cap), best_right

    def smoothness(self, scene: Scene, knots, steering_rad, previous_knots, curvature: float) -> float:
        """Lateral travel, peak sampled curvature (the tracking error grows with it), distance
        from the committed path and the jump from the current steering angle; it only orders
        candidates of equal clearance."""
        travel = sum(abs(b.d-a.d) for a, b in zip(knots, knots[1:]))
        # 0.1 1/m counts as 1 m of travel: in Gazebo the sharper of two equal-clearance S paths
        # passed the second row up to 5 cm closer.
        bend = 10.0*curvature
        continuity = 0.0
        if previous_knots:
            samples = [min(knots[-1].s, distance) for distance in (2.0, 4.0, 6.0, 8.0)]
            continuity = sum(abs(self.offset(s, knots)-self.offset(scene.route_progress_s+s, previous_knots))
                             for s in samples)/len(samples)
        steering = 0.0
        if steering_rad is not None:
            kappa = knots[0].second_derivative/(1+knots[0].slope**2)**1.5
            steering = abs(math.atan(self.wheelbase*kappa)-steering_rad)
        return travel+bend+2.0*continuity+steering

    def ranked(self, candidates: list) -> list:
        """The one ranking rule: (cost, knots, measured) tuples, best first.

        Inside the lane limits first. Among those, the largest gap up to the target clearance;
        candidates within gap_tolerance of the best are equal. Among the equal ones, the largest
        right-passage gap (up to the target plus right_passage_margin), again within
        gap_tolerance, then the smoothest. Outside the limits only when nothing is inside, then
        the least excess against the largest gap.
        """
        inside = [c for c in candidates if c[2].corridor <= 0.0]
        outside = sorted((c for c in candidates if c[2].corridor > 0.0),
                         key=lambda c: 50.0*c[2].corridor-20.0*c[2].gap+0.05*c[0])
        if not inside:
            return outside
        best = max(c[2].gap for c in inside)-self.gap_tolerance
        equal = [c for c in inside if c[2].gap >= best]
        best_right = max(c[2].right_gap for c in equal)-self.gap_tolerance
        first = sorted((c for c in equal if c[2].right_gap >= best_right), key=lambda c: c[0])
        second = sorted((c for c in equal if c[2].right_gap < best_right),
                        key=lambda c: (-c[2].right_gap, c[0]))
        rest = sorted((c for c in inside if c[2].gap < best), key=lambda c: -c[2].gap)
        return first+second+rest+outside

    def search(self, scene: Scene, *, steering_rad: float | None = None,
               previous_knots=None) -> SequencePath | None:
        """The best path past scene.rows by ranked(); previous_knots (route stations), the
        committed path, is one more candidate and the reference for continuity.

        Coarse to fine: every offset combination with the default placement, every placement
        for the three best combinations, every combination again with the best placement; the
        winner is checked again at the fine spacing.
        """
        rows = scene.rows
        if not rows or scene.limits_by_row is None:
            return None
        if steering_rad is not None and not math.isfinite(steering_rad):
            steering_rad = None
        self._followed_knots = tuple(previous_knots) if previous_knots else None
        offsets_by_row = [self.row_offsets(row, limits) for row, limits in zip(rows, scene.limits_by_row)]
        placements = self.placements(rows)
        measured: dict = {}
        coarse_limit = 0.95*self.max_curvature  # so the fine check and later ticks still pass

        def consider(keys):
            built = []
            for key in dict.fromkeys(keys):
                if key in measured:
                    continue
                measured[key] = None
                knots = self.knots_for(scene, key[0], key[1], steering_rad)
                if knots is not None:
                    built.append((key, knots))
            results = self.measure_many(scene, [knots for _, knots in built], 0.5, coarse_limit)
            for (key, knots), result in zip(built, results):
                if result is not None:
                    cost = self.smoothness(scene, knots, steering_rad, previous_knots, result.curvature)
                    measured[key] = (cost, knots, result)

        def best_keys(count):
            entries = [(key, value) for key, value in measured.items() if value is not None]
            order = {id(value): i for i, value in enumerate(self.ranked([v for _, v in entries]))}
            return [key for key, value in sorted(entries, key=lambda e: order[id(e[1])])][:count]

        combinations = [tuple(offsets) for offsets in product(*offsets_by_row)]
        consider((offsets, placements[0]) for offsets in combinations)
        consider((offsets, placement) for offsets in dict.fromkeys(key[0] for key in best_keys(3))
                 for placement in placements)
        best = best_keys(1)
        if best and best[0][1] != placements[0]:
            consider((offsets, best[0][1]) for offsets in combinations)
        self.last_search_candidates = len(measured)
        candidates = [value for value in measured.values() if value is not None]
        if previous_knots:
            knots = [replace(k, s=k.s-scene.route_progress_s) for k in previous_knots]
            if knots[-1].s > 0.0:
                result = self.measure(scene, knots, 0.5, coarse_limit)
                if result is not None:
                    candidates.append((self.smoothness(scene, knots, steering_rad, previous_knots, result.curvature),
                                       knots, result))
        for _, knots, coarse in self.ranked(candidates):
            final = self.measure(scene, knots)
            # A corner the coarse samples missed may leave the lane; the next candidate may not.
            if final is not None and (final.corridor <= 0.0 or coarse.corridor > 0.0):
                return self.path(scene, knots, final)
        return self.lane_edge_path(scene, steering_rad)

    def lane_edge_path(self, scene: Scene, steering_rad) -> SequencePath | None:
        """Last resort: each row passed at its lane edge, objects not checked."""
        offsets = tuple(self.forced_passage_offset(row, limits)
                        for row, limits in zip(scene.rows, scene.limits_by_row))
        for placement in self.placements(scene.rows):
            if placement.early:
                continue
            knots = self.knots_for(scene, offsets, placement, steering_rad)
            if knots is None:
                continue
            result = self.measure(scene, knots)
            if result is not None:
                return replace(self.path(scene, knots, replace(result, gap=math.nan)),
                               forced=True, clearance_m=-math.inf, corridor_excess_m=math.inf)
        return None

    def forced_passage_offset(self, row: Row, limits: tuple[float, float]) -> float:
        right_edge, left_edge = limits
        right_limit = right_edge + self.half_width + 0.05
        left_limit = left_edge - self.half_width - 0.05
        locked = self.locked_side(row)
        if locked is not None:
            return left_limit if locked > 0.0 else right_limit
        row_center = 0.5 * (row.low + row.high)
        if row_center < 0.0:
            return left_limit
        if row_center > 0.0:
            return right_limit
        left_gap = left_edge - row.high
        right_gap = row.low - right_edge
        return left_limit if left_gap >= right_gap else right_limit

    def path(self, scene: Scene, knots, measured: Measured) -> SequencePath:
        rows = scene.rows
        return SequencePath(
            [Point(x=float(x), y=float(y)) for x, y in measured.xy], measured.curvature, len(rows),
            (self.max_steering_rate/max(measured.steering_gradient, 1e-6)
             if math.isfinite(self.max_steering_rate) else math.inf),
            tuple(row.s for row in rows),
            tuple((row.low, row.high) for row in rows),
            tuple(self.offset(row.s, knots) for row in rows),
            knots[-1].s,
            measured.gap-self.recheck_tolerance,
            measured.corridor > 0.0 or measured.gap <= 0.0,
            scene.boundary_source,
            measured.gap,
            tuple(knots),
            measured.corridor,
        )

    def follow(self, scene: Scene, knots, *, min_gap: float, corridor_allowance: float,
               check_objects: bool = True) -> SequencePath | None:
        """The committed path (knots at route stations) from the current pose, or None when its
        gap fell below min_gap or it reaches further past the lane limits than allowed. Without
        check_objects only the geometry is rebuilt."""
        local = [replace(k, s=k.s-scene.route_progress_s) for k in knots]
        if not local or local[-1].s <= 0.0:
            return None
        # The samples move with the car: allow the curvature a little over the search limit.
        result = self.measure(scene, local, curvature_limit=1.02*self.max_curvature)
        if result is None or result.corridor > corridor_allowance:
            return None
        if not check_objects:
            result = replace(result, gap=math.nan)
        elif result.gap < min_gap:
            return None
        path = self.path(scene, local, result)
        return replace(path, rows=0, row_stations=(), row_occupancy=(), passage_offsets=())

    def local_route(self, route_points: list[Point], route_progress_s: float):
        """Route points from the vehicle's projection on, with their stations."""
        if len(route_points) < 2:
            return None
        stations = self.route_samples(route_points)
        if stations[-1] < 1.0e-3 or not math.isfinite(route_progress_s):
            return None
        ego_s, _ = self.project(route_points, stations, 0.0, 0.0)
        if ego_s > 1.0e-3:
            x, y, _ = self.route_pose(route_points, stations, ego_s)
            route_points = [Point(x=x, y=y)]+[p for p, s in zip(route_points, stations) if s > ego_s+1.0e-3]
            if len(route_points) < 2:
                return None
            stations = self.route_samples(route_points)
        else:
            # The route manager drops the points behind the car (local x < -0.2). With the car
            # yawed against the route the first point then lies up to about 1 m past the car's
            # projection, and station 0 would not be route_progress_s. Start at the projection on
            # the first segment extended back.
            a, b = route_points[0], route_points[1]
            length = max(1.0e-6, math.hypot(b.x-a.x, b.y-a.y))
            ux, uy = (b.x-a.x)/length, (b.y-a.y)/length
            behind = a.x*ux+a.y*uy
            if 1.0e-3 < behind <= 3.0:
                route_points = [Point(x=a.x-behind*ux, y=a.y-behind*uy)]+list(route_points)
                stations = self.route_samples(route_points)
        return route_points, stations

    def return_horizon(self, stations: list[float], complete_return: bool) -> float | None:
        """Farthest object station a bypass can be planned for with its full return on this route.

        The local route ends at the route lookahead. A row nearer that end than the return length
        would get a return cut short at the route end, so it is planned for only once its full
        return fits. Without complete_return (the route itself ends in the window) nothing is left
        out.
        """
        return stations[-1] - self.return_length if complete_return else None

    # One tick ---------------------------------------------------------------------------------

    def observe(
        self,
        route_points: list[Point],
        objects: PerceptionObjectArray | None,
        *,
        route_progress_s: float = 0.0,
        observation_time_sec: float | None = None,
        observation_stamp_sec: float | None = None,
        lane_lines=None,
        complete_return: bool = False,
        track: bool = True,
    ) -> Scene | None:
        """The scene of this tick. With track the objects update the row tracks and the selected
        rows; without (perception lost) the scene has no rows and only serves the committed path.

        lane_lines: map lane limits as (s, d) samples with s from the vehicle's route station;
        without them the configured route corridor stands in. complete_return: the route goes on
        past the local route, so objects are planned for only once their full return fits
        (return_horizon)."""
        local = self.local_route(route_points, route_progress_s)
        if local is None:
            return None
        route_points, stations = local
        self._geometry_by_count = None
        now_sec = self.observation_time(objects, observation_time_sec) if track else math.nan
        lines = [list(line) for line in lane_lines] if lane_lines is not None else []
        boundary_source = "map_lane" if lane_lines is not None else "none"
        paired_boundaries = any(self.lane_limits(lines, s) is not None for s in stations)
        if not paired_boundaries and self.route_half_width > 0.0:
            lines = [[(-self.rear, d), (stations[-1]+self.front, d)] for d in (-self.route_half_width, self.route_half_width)]
            boundary_source = "configured_route_corridor"
        # Objects off the map lane (or the configured corridor) are not obstacles, so roadside
        # posts in the next lane are not taken for a drum row (2026-10-17 final). Perceived lane
        # lines are not used for this; the corridor stands in for them.
        obstacles = (
            self.visible_obstacles(route_points, stations, objects,
                                   lines if boundary_source in {"map_lane", "configured_route_corridor"} else [],
                                   self.return_horizon(stations, complete_return))
            if objects is not None else []
        )
        _, current_d = self.project(route_points, stations, 0.0, 0.0)
        _, _, route_yaw = self.route_pose(route_points, stations, 0.0)
        scene = Scene(route_points, stations, lines, boundary_source, obstacles, (), None,
                      current_d, math.tan(-route_yaw), route_progress_s)
        if not track:
            return scene

        observed_rows = self.rows(self.row_obstacles(obstacles, lines))
        for row in observed_rows:
            limits = self.lane_limits(lines, row.s)
            if limits is not None:
                row.right_limit, row.left_limit = limits
        if observation_stamp_sec is None and objects is not None:
            stamp = objects.header.stamp
            observation_stamp_sec = float(stamp.sec) + 1.0e-9*float(stamp.nanosec)
        rows = self.select_rows(self.tracked_rows(
            observed_rows,
            route_progress_s,
            now_sec,
            stations[-1],
            observation_stamp_sec=observation_stamp_sec,
        ), route_progress_s=route_progress_s)
        if not rows:
            if not self.row_tracks:
                self.row_sides = []
                self.first_row_station = None
                self.second_row_station = None
                self.selected_rows = ()
            return scene

        self.selected_rows = tuple(self.global_row(row, route_progress_s) for row in rows)
        global_first = route_progress_s + rows[0].s
        remaining_second = (
            len(rows) == 1
            and self.second_row_station is not None
            and abs(global_first-self.second_row_station) <= self.row_track_match_m
        )
        if len(rows) == 2:
            self.first_row_station = global_first
            self.second_row_station = route_progress_s + rows[1].s
        elif not remaining_second:
            self.first_row_station = global_first
            self.second_row_station = None

        self.progress_s = route_progress_s
        limits_by_row = []
        for row in rows:
            limits = self.lane_limits(lines, row.s)
            if limits is None and row.right_limit is not None and row.left_limit is not None:
                limits = row.right_limit, row.left_limit
            if limits is None:
                # The far row is often beyond the perceived lane lines (2026-10-17 final: the
                # second row 25-30 m ahead had none). Use the other rows' limits, then the corridor.
                limits = self.fallback_lane_limits(rows)
            if limits is None and self.route_half_width > 0.0:
                limits = -self.route_half_width, self.route_half_width
            if limits is None:
                limits_by_row = None
                break
            limits_by_row.append((limits[0], limits[1]+self.centerline_departure))
        if limits_by_row is not None:
            self.lock_row_sides(rows, limits_by_row)
        obstacles = obstacles+self.missing_row_obstacles(route_points, stations, rows, observed_rows)
        return replace(scene, obstacles=obstacles, rows=tuple(rows),
                       limits_by_row=tuple(limits_by_row) if limits_by_row is not None else None)
