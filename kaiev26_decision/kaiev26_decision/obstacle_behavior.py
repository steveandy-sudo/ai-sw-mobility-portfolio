"""Obstacle behavior: zone speed caps and the committed bypass path. It never requests a stop.

Every tick the planner observes the rows; it searches for a path only when there is none
committed, the committed path lost its clearance (it now keeps less than its committed gap less
the planner's recheck tolerance, or reaches further out of the lane) or a new row appears. A
committed forced (best-effort) path is searched again every forced_replan_sec and replaced by a
valid path, or a forced one with more gap, as main does (10/23 Midan: a forced path committed
for a short-lived fragment was kept past both drum rows while valid paths existed). While
perception is lost the committed path is kept at the zone cap. The path belongs to this behavior
until the committed return point, also while a stop event brakes the car.

The speed follows the clearance the committed path keeps: the zone speed at the target clearance,
the fallback speed at slow_clearance_m and below, in between linearly, until the rear axle is
midway between the two rows (one row: past it); from there only the path curvature limits it.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from statistics import median

from kaiev26_msgs.msg import Centerline

from kaiev26_decision.arbiter import ModuleOutput
from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner, PathKnot, Scene, SequencePath


# States, also the fsm_state shown to people (docs/ARCHITECTURE.md).
IDLE = "OBSTACLE_IDLE"
PRE_SLOW = "OBSTACLE_APPROACH"
IN_ZONE = "OBSTACLE_LANE_KEEP"
BYPASS = "OBSTACLE_BYPASS"
BYPASS_FORCED = "OBSTACLE_BYPASS_FORCED"
RETURN = "OBSTACLE_RETURN"
FALLBACK = "OBSTACLE_NO_PATH"


@dataclass(frozen=True)
class CommittedBypass:
    knots: tuple[PathKnot, ...]
    row_stations: tuple[float, ...]
    return_s: float
    # Gap measured when committed (NaN for the lane-edge S) and the least it may keep later.
    gap_m: float
    min_gap_m: float
    corridor_allowance_m: float
    forced: bool
    rows: int
    # Rear-axle station midway between the two rows where the bypass and zone caps end
    # (2026-10-24: accelerate out from between the drum rows); inf with a single row.
    release_s: float = math.inf


class ObstacleBehavior:
    def __init__(
        self,
        planner: ObstacleSequencePlanner,
        *,
        zone=None,
        zone_speed_mps: float = 3.0,
        bypass_speed_mps: float = math.inf,
        pre_slow_decel_mps2: float = 0.5,
        fallback_speed_mps: float = 1.0,
        forced_speed_mps: float | None = None,
        slow_clearance_m: float = 0.25,
        curve_speed_window_sec: float = 0.0,
        lateral_accel_mps2: float = 1.4,
        front_overhang_m: float = 1.98,
        route_length_m: float = math.inf,
        search_retry_sec: float = 0.5,
        forced_replan_sec: float = 0.5,
        release_past_row_m: float = 3.0,
        release_offset_m: float = 0.5,
    ) -> None:
        self.planner = planner
        self.zone = zone
        self.zone_speed_mps = max(0.0, zone_speed_mps)
        # Cap while a bypass path is followed, until the body is past the last row. On the final it
        # equals the zone cap; a zone with no cap of its own (Midan, 15 m/s) keeps the course speed
        # until the drums are seen and gets it back right after them.
        self.bypass_speed_mps = max(0.0, bypass_speed_mps)
        self.pre_slow_decel_mps2 = max(0.05, pre_slow_decel_mps2)
        self.fallback_speed_mps = max(0.1, fallback_speed_mps)
        # A forced (best-effort) path and a path at slow_clearance_m or less; the fallback speed by
        # default. The speed rises from it to the bypass cap at the target clearance.
        self.forced_speed_mps = self.fallback_speed_mps if forced_speed_mps is None else max(0.1, forced_speed_mps)
        self.slow_clearance_m = slow_clearance_m
        self.curve_speed_window_sec = max(0.0, curve_speed_window_sec)
        self.curve_speeds: deque[tuple[float, float]] = deque()
        self.lateral_accel_mps2 = max(0.1, lateral_accel_mps2)
        self.front_overhang_m = max(0.0, front_overhang_m)
        self.route_length_m = route_length_m
        # A search that found nothing is not repeated for the same rows within this time.
        self.search_retry_sec = max(0.0, search_retry_sec)
        self.forced_replan_sec = max(0.0, forced_replan_sec)
        # The bypass counts as finished for the zone cap once the rear axle is this far past its
        # last row and the committed path is back within this offset of the route, without
        # waiting for the return point (2026-10-17 143332: the car was back on the route at s 672
        # and still held 3 m/s to the last knot at s 688). The path stays until the return point.
        self.release_past_row_m = max(0.0, release_past_row_m)
        self.release_offset_m = max(0.0, release_offset_m)
        self.reset()

    def reset(self) -> None:
        self.planner.reset()
        self.committed: CommittedBypass | None = None
        self.failed_search: tuple[float, tuple[tuple[float, float], ...]] | None = None
        self.forced_searched_sec: float | None = None
        self.state = IDLE
        self.curve_speeds.clear()

    @property
    def bypass_active(self) -> bool:
        return self.committed is not None

    def bypass_finished(self, s_rear: float) -> bool:
        """Past the last row by release_past_row_m and the committed path within
        release_offset_m of the route here."""
        committed = self.committed
        if committed is None or not committed.row_stations or not committed.knots:
            return False
        return (
            s_rear >= max(committed.row_stations) + self.release_past_row_m
            and abs(ObstacleSequencePlanner.offset(s_rear, list(committed.knots))) <= self.release_offset_m
        )

    def zone_cap(self, s_rear: float) -> tuple[float | None, str]:
        if self.zone is None:
            return None, ""
        s_front = s_rear + self.front_overhang_m
        if s_front < self.zone.entry_s:
            # Reach the zone speed at the entry by rolling at the normal deceleration.
            distance = self.zone.entry_s - s_front
            return (
                math.sqrt(self.zone_speed_mps ** 2 + 2.0 * self.pre_slow_decel_mps2 * distance),
                "OBSTACLE_ZONE_PRE_SLOW",
            )
        # The cap ends once the rear axle is midway between the committed rows, or else once the car
        # is past the zone exit and a committed bypass is back on the route (bypass_finished, at
        # the latest its return point); then the course speed applies.
        if self.committed is not None and s_rear >= self.committed.release_s:
            return None, ""
        if s_rear <= self.zone.exit_s or (self.committed is not None and not self.bypass_finished(s_rear)):
            return self.zone_speed_mps, "OBSTACLE_ZONE_CAP"
        return None, ""

    def path_speed(self, path: SequencePath) -> float:
        return min(
            math.sqrt(self.lateral_accel_mps2 / max(path.curvature, 1.0e-6)),
            path.steering_rate_speed_mps,
        )

    def clearance_speed(self, gap_m: float, forced: bool) -> float | None:
        """Speed cap for a path that keeps gap_m; None at the target clearance or more."""
        target = self.planner.clearance
        if forced or not math.isfinite(gap_m) or gap_m <= min(self.slow_clearance_m, target):
            return self.forced_speed_mps
        if gap_m >= target:
            return None
        ratio = (gap_m - self.slow_clearance_m) / (target - self.slow_clearance_m)
        top = min(self.zone_speed_mps, self.bypass_speed_mps)
        return self.forced_speed_mps + ratio * max(0.0, top - self.forced_speed_mps)

    def filtered_path_speed(self, speed: float, now_sec: float | None) -> float:
        """Median over a short window; the bypass path curvature changes from frame to frame."""
        if now_sec is None or self.curve_speed_window_sec <= 0.0:
            return speed
        speeds = self.curve_speeds
        while speeds and not 0.0 <= now_sec - speeds[0][0] <= self.curve_speed_window_sec:
            speeds.popleft()
        speeds.append((now_sec, speed))
        return median(value for _, value in speeds)

    def new_row_seen(self) -> bool:
        if self.committed is None:
            return False
        match = self.planner.row_track_match_m
        return any(
            all(abs(row.s - station) > match for station in self.committed.row_stations)
            for row in self.planner.selected_rows
        )

    def release_station(self, row_stations: tuple[float, ...]) -> float:
        """Midway between the first and last row. A replan once the first row is behind sees only
        the last one; it keeps the midpoint of the committed pair it continues."""
        if len(row_stations) >= 2:
            return 0.5 * (min(row_stations) + max(row_stations))
        previous = self.committed
        if (row_stations and previous is not None and len(previous.row_stations) >= 2
                and abs(row_stations[-1] - max(previous.row_stations)) <= self.planner.row_track_match_m):
            return previous.release_s
        return math.inf

    def commit(self, path: SequencePath, progress_s: float) -> None:
        row_stations = tuple(progress_s + s for s in path.row_stations)
        self.committed = CommittedBypass(
            knots=tuple(PathKnot(k.s + progress_s, k.d, k.slope, k.second_derivative) for k in path.knots),
            row_stations=row_stations,
            return_s=progress_s + path.rejoin_station,
            gap_m=path.body_clearance_m,
            min_gap_m=path.clearance_m,
            corridor_allowance_m=path.corridor_excess_m + 0.05,
            forced=path.forced,
            rows=path.rows,
            release_s=self.release_station(row_stations),
        )

    def follow(self, scene: Scene, *, check_objects: bool) -> SequencePath | None:
        committed = self.committed
        return self.planner.follow(
            scene, committed.knots, min_gap=committed.min_gap_m,
            corridor_allowance=committed.corridor_allowance_m, check_objects=check_objects,
        )

    def search(self, scene: Scene, steering_rad: float | None, now_sec: float | None) -> SequencePath | None:
        """A new path for the rows of the scene. A search that found nothing waits
        search_retry_sec for the same rows: it measures every candidate and would find nothing again."""
        if not scene.rows:
            return None
        match = self.planner.row_track_match_m
        rows = tuple((scene.route_progress_s + row.s, 0.5 * (row.low + row.high)) for row in scene.rows)
        failed = self.failed_search
        if (failed is not None and now_sec is not None
                and 0.0 <= now_sec - failed[0] <= self.search_retry_sec
                and len(failed[1]) == len(rows)
                and all(abs(a[0] - b[0]) <= match and abs(a[1] - b[1]) <= match
                        for a, b in zip(failed[1], rows))):
            return None
        path = self.planner.search(
            scene, steering_rad=steering_rad,
            previous_knots=self.committed.knots if self.committed is not None else None,
        )
        self.failed_search = None if path is not None or now_sec is None else (now_sec, rows)
        return path

    def replan_reason(self, committed_path: SequencePath | None, now_sec: float | None = None) -> str:
        """Why a new path is searched this tick; empty when the committed path stays."""
        if self.committed is None:
            return "FIRST" if self.planner.selected_rows else ""
        if self.new_row_seen():
            return "NEW_ROW"
        if committed_path is None:
            return "CLEARANCE_LOST"
        if (self.committed.forced and now_sec is not None and (
                self.forced_searched_sec is None
                or not 0.0 <= now_sec - self.forced_searched_sec < self.forced_replan_sec)):
            return "FORCED_RETRY"
        return ""

    def improves_on_committed(self, planned: SequencePath) -> bool:
        """A retried forced path is replaced only by a valid path or a forced one with more gap."""
        if not planned.forced:
            return True
        old, new = self.committed.gap_m, planned.body_clearance_m
        return math.isfinite(new) and (not math.isfinite(old) or new > old + self.planner.gap_tolerance)

    def update(
        self,
        route,
        *,
        mode_enabled: bool,
        stage_enabled: bool,
        objects,
        perception_ok: bool,
        lane_lines=None,
        steering_rad: float | None = None,
        now_sec: float | None = None,
    ) -> ModuleOutput:
        output = ModuleOutput("obstacle")
        if not mode_enabled:
            self.state = IDLE
            return output
        s_rear = float(route.progress_s)
        if self.committed is not None and s_rear >= self.committed.return_s:
            self.committed = None
            output.tags.append("OBSTACLE_BYPASS_RETURNED")
        cap, cap_tag = self.zone_cap(s_rear)
        self.state = {"OBSTACLE_ZONE_CAP": IN_ZONE, "OBSTACLE_ZONE_PRE_SLOW": PRE_SLOW}.get(cap_tag, IDLE)

        path = None
        fallback = False
        points = list(route.local_path_points)
        scene = None
        if stage_enabled and len(points) >= 2:
            # The local route stops at the lookahead. While the route goes on past it, a row is
            # planned for only once its full return fits, so no committed path has a cut-off return.
            complete_return = s_rear + self.planner.route_samples(points)[-1] + 0.5 < self.route_length_m
            scene = self.planner.observe(
                points, objects if perception_ok else None, route_progress_s=s_rear,
                observation_time_sec=now_sec, observation_stamp_sec=now_sec, lane_lines=lane_lines,
                complete_return=complete_return, track=perception_ok,
            )
        if scene is not None:
            committed_path = self.follow(scene, check_objects=perception_ok) if self.committed is not None else None
            if perception_ok:
                reason = self.replan_reason(committed_path, now_sec)
                planned = self.search(scene, steering_rad, now_sec) if reason else None
                if reason == "FORCED_RETRY":
                    self.forced_searched_sec = now_sec
                    if planned is not None and not self.improves_on_committed(planned):
                        planned = None
                if planned is not None:
                    self.forced_searched_sec = now_sec
                    self.commit(planned, s_rear)
                    path = planned
                    output.tags.extend([
                        "OBSTACLE_BYPASS_COMMITTED", f"OBSTACLE_REPLAN_{reason}",
                        f"OBSTACLE_BYPASS_GAP:{planned.body_clearance_m:.2f}",
                    ])
                elif committed_path is not None:
                    path = committed_path
                elif self.committed is not None or self.planner.selected_rows:
                    # Rows ahead and no valid path: the only place the fallback speed applies
                    # without a path of its own.
                    fallback = True
                    if self.committed is not None:
                        path = self.planner.follow(scene, self.committed.knots, min_gap=-math.inf,
                                                   corridor_allowance=math.inf, check_objects=False)
            elif committed_path is not None:
                path = committed_path
                output.tags.append("OBSTACLE_PERCEPTION_LOST_KEEP_PATH")
            else:
                output.tags.append("OBSTACLE_PERCEPTION_LOST")

        if path is not None:
            committed = self.committed
            rows = committed.rows if committed is not None else path.rows
            output.path = self.path_message(route, path, rows)
            output.path_request = "BYPASS_PATH"
            speed = self.filtered_path_speed(self.path_speed(path), now_sec)
            forced = committed.forced if committed is not None else path.forced
            # The bypass cap and the committed gap set the speed until the rear axle is midway
            # between the two rows (one row: the body past it); from there only the path curvature
            # limits it (10/23 Midan: 3 m/s held to the return point after the drums).
            past_rows = committed is not None and bool(committed.row_stations) and (
                s_rear >= committed.release_s
                or s_rear > committed.row_stations[-1] + self.planner.rear + 0.5)
            self.state = RETURN if past_rows else BYPASS_FORCED if forced else BYPASS
            if not past_rows:
                speed = min(speed, self.bypass_speed_mps)
                gap = committed.gap_m if committed is not None else path.body_clearance_m
                clearance_cap = self.clearance_speed(gap, forced)
                if clearance_cap is not None:
                    speed = min(speed, clearance_cap)
            cap = speed if cap is None else min(cap, speed)
            output.tags.append("OBSTACLE_FORCED_S_PATH" if forced else "OBSTACLE_SEQUENCE_PATH")
        if fallback:
            self.state = FALLBACK
            cap = self.fallback_speed_mps if cap is None else min(cap, self.fallback_speed_mps)
            output.tags.extend(["OBSTACLE_NO_SAFE_PATH", "LOW_SPEED_CONTINUE"])
        if cap_tag:
            output.tags.append(cap_tag)
        output.cap_mps = cap
        return output

    @staticmethod
    def path_message(route, path: SequencePath, rows: int) -> Centerline:
        message = Centerline()
        message.header = route.header
        message.detection_id = 1
        message.points = path.points
        message.confidence = min(0.85, float(route.projection_confidence))
        prefix = "forced_" if path.forced else ""
        message.source = f"{prefix}sequential_bypass_{rows}_rows_{path.boundary_source}"
        return message
