from __future__ import annotations

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import Centerline, RouteContext, SceneSummary, TargetSpeed

from kaiev26_decision.common import DecisionCommand, INF_DISTANCE, clamp


@dataclass
class RememberedObstacle:
    route_progress_s: float | None
    x: float
    y: float
    length: float
    width: float
    bypass_side: float = 0.0


class PlanningCore:
    def __init__(
        self,
        base_speed_mps: float,
        degraded_speed_mps: float,
        comfortable_decel_mps2: float = 1.4,
        route_rejoin_min_distance_m: float = 4.0,
        route_rejoin_max_distance_m: float = 10.0,
        vehicle_front_overhang_m: float = 1.98,
        vehicle_width_m: float = 1.18,
        obstacle_clearance_m: float = 0.30,
        obstacle_lane_departure_allowed: bool = True,
        obstacle_side_deadband_m: float = 0.30,
        obstacle_preferred_side: str = "left",
        obstacle_transition_length_m: float = 10.0,
        obstacle_return_length_m: float = 10.0,
        stopline_clearance_m: float = 1.0,
    ) -> None:
        self.base_speed_mps = base_speed_mps
        self.degraded_speed_mps = degraded_speed_mps
        self.comfortable_decel_mps2 = comfortable_decel_mps2
        self.route_rejoin_min_distance_m = route_rejoin_min_distance_m
        self.route_rejoin_max_distance_m = route_rejoin_max_distance_m
        self.vehicle_front_overhang_m = max(0.0, vehicle_front_overhang_m)
        self.vehicle_half_width_m = max(0.0, vehicle_width_m * 0.5)
        self.obstacle_clearance_m = max(0.0, obstacle_clearance_m)
        self.obstacle_lane_departure_allowed = obstacle_lane_departure_allowed
        self.obstacle_side_deadband_m = max(0.0, obstacle_side_deadband_m)
        self.obstacle_preferred_side = (
            -1.0 if obstacle_preferred_side.strip().lower() == "right" else 1.0
        )
        self.obstacle_transition_length_m = max(2.5, obstacle_transition_length_m)
        self.obstacle_return_length_m = max(2.5, obstacle_return_length_m)
        self.stopline_clearance_m = max(0.0, stopline_clearance_m)
        self.previous_safe_path: Centerline | None = None
        self.remembered_obstacle: RememberedObstacle | None = None

    def reset_transient_state(self) -> None:
        self.previous_safe_path = None
        self.remembered_obstacle = None

    def stop_distance_before_line(self, stopline_distance: float) -> float:
        return max(
            0.0,
            float(stopline_distance) - self.vehicle_front_overhang_m - self.stopline_clearance_m,
        )

    def make_plan(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        route: RouteContext | None,
        extra_constraints: list[str],
    ) -> tuple[Centerline, TargetSpeed]:
        target_path = self.make_target_path(decision, scene, route)
        speed_limit = self.resolve_speed_limit(decision, scene, route, extra_constraints)
        target_speed = self.make_target_speed(decision, scene, speed_limit, extra_constraints)
        return target_path, target_speed

    def make_target_path(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        route: RouteContext | None,
    ) -> Centerline:
        msg = Centerline()
        msg.header = scene.header
        msg.track_id = 0
        msg.source = "main_planning_engine"

        if decision.path_request != "BYPASS_PATH" and not scene.obstacle_on_path:
            self.remembered_obstacle = None

        if decision.path_request == "STOP_PATH":
            stop_path = self.make_stop_path(decision, scene, route)
            if stop_path is not None:
                self.previous_safe_path = stop_path
                return stop_path

        if decision.path_request == "BYPASS_PATH":
            bypass = self.make_bypass_path(scene, route)
            if bypass is not None:
                self.previous_safe_path = bypass
                return bypass

        if decision.path_request == "ROUTE_REJOIN_PATH":
            rejoin = self.make_route_rejoin_path(scene, route)
            if rejoin is not None:
                self.previous_safe_path = rejoin
                return rejoin

        can_use_scene_path = scene.centerline_valid and len(scene.centerline_points) >= 2
        if decision.path_request == "CENTERLINE" and can_use_scene_path:
            msg.detection_id = 1
            msg.points = list(scene.centerline_points)
            msg.confidence = float(scene.centerline_quality)
            self.previous_safe_path = msg
            return msg

        can_use_route_path = (
            decision.path_request == "ROUTE_LOCAL_PATH"
            and route is not None
            and route.route_projection_valid
            and len(route.local_path_points) >= 2
        )
        if can_use_route_path:
            msg.header = route.header
            msg.detection_id = 1
            msg.points = list(route.local_path_points)
            msg.confidence = float(route.projection_confidence)
            msg.source = "global_route_local_path"
            self.previous_safe_path = msg
            return msg

        if self.previous_safe_path is not None:
            msg.header = scene.header
            msg.detection_id = self.previous_safe_path.detection_id
            msg.track_id = self.previous_safe_path.track_id
            msg.points = list(self.previous_safe_path.points)
            msg.confidence = min(0.5, float(self.previous_safe_path.confidence))
            msg.source = "previous_safe_path"
            return msg

        msg.confidence = 0.0
        return msg

    def make_stop_path(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        route: RouteContext | None,
    ) -> Centerline | None:
        # 종방향 정지는 TargetSpeed가 담당한다. 횡방향 경로를 정지점에서 자르면
        # 저속 Pure Pursuit의 lookahead가 사라져 조향이 포화될 수 있으므로,
        # 정지 중에도 진행 방향을 나타내는 정상 경로를 유지한다.
        base_path, confidence, source = self.bypass_base_path_points(scene, route)
        if len(base_path) < 2:
            return None

        msg = Centerline()
        msg.header = scene.header
        if route is not None and source == "global_route_local_path":
            msg.header = route.header
        msg.detection_id = 1
        msg.track_id = 0
        msg.points = [self.copy_point(point) for point in base_path]
        msg.confidence = max(0.1, min(0.9, confidence))
        msg.source = f"longitudinal_stop_from_{source}"
        return msg

    def resolve_stop_distance(self, decision: DecisionCommand, scene: SceneSummary) -> float:
        stop_distance = float(decision.stop_target_distance)
        if math.isfinite(stop_distance) and 0.0 <= stop_distance < INF_DISTANCE:
            return stop_distance

        stopline_distance = float(scene.stopline_distance)
        if math.isfinite(stopline_distance) and 0.0 <= stopline_distance < INF_DISTANCE:
            return self.stop_distance_before_line(stopline_distance)

        return INF_DISTANCE

    def truncate_path_at_x(self, base_path: list[Point], stop_x: float) -> list[Point]:
        if not math.isfinite(stop_x) or stop_x >= INF_DISTANCE:
            return [self.copy_point(point) for point in base_path]

        stop_x = max(0.0, stop_x)
        points = [self.copy_point(base_path[0])]
        if base_path[0].x >= stop_x:
            points.append(self.point_at_x(base_path[0], base_path[1], stop_x))
            return points

        for previous, current in zip(base_path, base_path[1:]):
            if current.x < stop_x:
                points.append(self.copy_point(current))
                continue

            points.append(self.point_at_x(previous, current, stop_x))
            return self.drop_duplicate_tail(points)

        return [self.copy_point(point) for point in base_path]

    def point_at_x(self, start: Point, end: Point, target_x: float) -> Point:
        point = Point()
        dx = float(end.x - start.x)
        if abs(dx) < 1.0e-6:
            ratio = 0.0
        else:
            ratio = clamp((target_x - float(start.x)) / dx, 0.0, 1.0)

        point.x = float(start.x) + ratio * dx
        point.y = float(start.y) + ratio * float(end.y - start.y)
        point.z = float(start.z) + ratio * float(end.z - start.z)
        return point

    def copy_point(self, point: Point) -> Point:
        copied = Point()
        copied.x = point.x
        copied.y = point.y
        copied.z = point.z
        return copied

    def drop_duplicate_tail(self, points: list[Point]) -> list[Point]:
        if len(points) < 2:
            return points

        tail = points[-1]
        previous = points[-2]
        if math.hypot(tail.x - previous.x, tail.y - previous.y) < 1.0e-4:
            return points[:-1]
        return points

    def make_route_rejoin_path(self, scene: SceneSummary, route: RouteContext | None) -> Centerline | None:
        if route is None or not route.route_projection_valid or len(route.local_path_points) < 2:
            return None

        route_points = [self.copy_point(point) for point in route.local_path_points]
        merge_x = self.resolve_rejoin_merge_x(route, route_points)
        merge_point = self.interpolate_route_point(route_points, merge_x)
        if merge_point is None:
            return None

        point_count = max(5, int(math.ceil(max(merge_x, 1.0))))
        points: list[Point] = []
        for index in range(point_count + 1):
            ratio = index / point_count
            point = Point()
            point.x = ratio * merge_point.x
            point.y = self.smoothstep(ratio) * merge_point.y
            point.z = self.smoothstep(ratio) * merge_point.z
            points.append(point)

        for route_point in route_points:
            if route_point.x > merge_point.x + 0.1:
                points.append(route_point)

        msg = Centerline()
        msg.header = route.header if route.header.frame_id else scene.header
        msg.detection_id = 1
        msg.track_id = 0
        msg.points = self.drop_duplicate_points(points)
        msg.confidence = max(0.1, min(0.85, float(route.projection_confidence)))
        msg.source = "local_rejoin_to_global_route"
        return msg

    def resolve_rejoin_merge_x(self, route: RouteContext, route_points: list[Point]) -> float:
        desired = 3.0 + 2.0 * abs(float(route.cross_track_error)) + 2.0 * abs(float(route.heading_error))
        desired = clamp(desired, self.route_rejoin_min_distance_m, self.route_rejoin_max_distance_m)
        forward_x_values = [point.x for point in route_points if point.x > 0.5]
        if not forward_x_values:
            return max(1.0, route_points[-1].x)
        return min(max(forward_x_values), desired)

    def interpolate_route_point(self, points: list[Point], target_x: float) -> Point | None:
        if not points:
            return None
        if target_x <= points[0].x and len(points) >= 2:
            return self.point_at_x(points[0], points[1], target_x)
        for previous, current in zip(points, points[1:]):
            if current.x >= target_x:
                return self.point_at_x(previous, current, target_x)
        return self.copy_point(points[-1])

    def drop_duplicate_points(self, points: list[Point]) -> list[Point]:
        filtered: list[Point] = []
        for point in points:
            if filtered and math.hypot(point.x - filtered[-1].x, point.y - filtered[-1].y) < 1.0e-4:
                continue
            filtered.append(point)
        return filtered

    def make_bypass_path(self, scene: SceneSummary, route: RouteContext | None) -> Centerline | None:
        base_path, confidence, source = self.bypass_base_path_points(scene, route)
        obstacle = self.resolve_bypass_obstacle(scene, route, base_path)
        if len(base_path) < 2 or obstacle is None:
            return None

        obstacle_x = obstacle.x
        obstacle_y = obstacle.y
        obstacle_length = obstacle.length
        obstacle_width = obstacle.width

        target_lane_points, target_lane_name, bypass_side = self.select_bypass_lane_center(
            scene,
            obstacle,
            base_path,
        )
        target_offset_m, corridor_limited = self.minimum_clearance_target_offset(
            scene,
            obstacle,
            base_path,
            target_lane_points,
            bypass_side,
        )
        lateral_shift_m = target_offset_m

        obstacle_front_x = max(0.0, obstacle_x - obstacle_length * 0.5)
        obstacle_rear_x = obstacle_x + obstacle_length * 0.5
        post_obstacle_margin_m = 1.5
        approach_start, hold_start = self.bypass_transition_window(
            obstacle_front_x,
            lateral_shift_m,
        )
        hold_end = obstacle_rear_x + post_obstacle_margin_m
        return_length_m = max(
            self.obstacle_return_length_m,
            abs(lateral_shift_m) * 4.0,
        )
        return_end = hold_end + return_length_m
        points = self.blend_to_normal_offset_path(
            base_path,
            lateral_shift_m,
            approach_start,
            hold_start,
            hold_end,
            return_end,
        )
        path_kind = (
            f"min_clearance_{target_lane_name}"
            if target_lane_points
            else "min_clearance_offset"
        )
        if corridor_limited:
            path_kind += "_corridor_limited"

        msg = Centerline()
        msg.header = scene.header
        if route is not None and source == "global_route_local_path":
            msg.header = route.header
        msg.detection_id = 1
        msg.track_id = 0
        msg.points = points
        msg.confidence = max(0.1, min(0.85, confidence))
        memory_suffix = "_memory" if not scene.obstacle_on_path else ""
        msg.source = f"local_bypass{memory_suffix}_{path_kind}_from_{source}"
        return msg

    def bypass_transition_window(
        self,
        obstacle_front_x: float,
        lateral_shift_m: float,
    ) -> tuple[float, float]:
        path_spacing_guard_m = 2.0
        clearance_lead_m = self.vehicle_front_overhang_m + path_spacing_guard_m
        if obstacle_front_x <= 6.0:
            hold_start = max(1.0, obstacle_front_x - clearance_lead_m)
            return 0.0, hold_start

        hold_start = max(1.0, obstacle_front_x - clearance_lead_m)
        transition_length_m = max(
            self.obstacle_transition_length_m,
            abs(lateral_shift_m) * 4.0,
        )
        approach_start = max(0.0, hold_start - transition_length_m)
        return approach_start, hold_start

    def select_bypass_lane_center(
        self,
        scene: SceneSummary,
        obstacle: RememberedObstacle,
        base_path: list[Point],
    ) -> tuple[list[Point], str, float]:
        bypass_side = float(obstacle.bypass_side)
        if bypass_side == 0.0:
            lateral_y = float(obstacle.y)
            if lateral_y > self.obstacle_side_deadband_m:
                bypass_side = -1.0
            elif lateral_y < -self.obstacle_side_deadband_m:
                bypass_side = 1.0
            else:
                bypass_side = self.obstacle_preferred_side
            obstacle.bypass_side = bypass_side
            if self.remembered_obstacle is not None:
                self.remembered_obstacle.bypass_side = bypass_side

        if bypass_side > 0.0:
            if scene.left_lane_center_valid and len(scene.left_lane_center_points) >= 2:
                return list(scene.left_lane_center_points), "left", 1.0
            return [], "none", 1.0

        if scene.right_lane_center_valid and len(scene.right_lane_center_points) >= 2:
            return list(scene.right_lane_center_points), "right", -1.0
        return [], "none", -1.0

    def minimum_clearance_target_offset(
        self,
        scene: SceneSummary,
        obstacle: RememberedObstacle,
        base_path: list[Point],
        target_lane_points: list[Point],
        bypass_side: float,
    ) -> tuple[float, bool]:
        required = self.required_obstacle_center_clearance(obstacle.width)
        target_offset_m = float(obstacle.y) + bypass_side * required

        # 이미 기준 경로가 선택한 쪽에서 충분한 간격을 확보하면 더 움직이지 않는다.
        if bypass_side * target_offset_m <= 0.0:
            target_offset_m = 0.0

        corridor_limited = False
        if target_lane_points:
            lane_offset_m = self.lane_offset_at_path_s(
                base_path,
                target_lane_points,
                obstacle.x,
            )
            if lane_offset_m is not None and bypass_side * lane_offset_m > 0.0:
                lane_width = float(scene.lane_width_estimate_m)
                if not math.isfinite(lane_width) or lane_width <= 0.0:
                    lane_width = 3.5
                outer_room = max(
                    0.0,
                    lane_width * 0.5 - self.vehicle_half_width_m - self.obstacle_clearance_m,
                )
                outer_limit_m = lane_offset_m + bypass_side * outer_room
                if bypass_side * (target_offset_m - outer_limit_m) > 0.0:
                    target_offset_m = outer_limit_m
                    corridor_limited = True
        elif not self.obstacle_lane_departure_allowed:
            lane_width = float(scene.lane_width_estimate_m)
            if not math.isfinite(lane_width) or lane_width <= 0.0:
                lane_width = 3.5
            in_lane_room = max(
                0.35,
                lane_width * 0.5 - self.vehicle_half_width_m - self.obstacle_clearance_m,
            )
            in_lane_limit_m = bypass_side * in_lane_room
            if bypass_side * (target_offset_m - in_lane_limit_m) > 0.0:
                target_offset_m = in_lane_limit_m
                corridor_limited = True

        return target_offset_m, corridor_limited

    def blend_to_normal_offset_path(
        self,
        base_path: list[Point],
        lateral_shift_m: float,
        approach_start: float,
        hold_start: float,
        hold_end: float,
        return_end: float,
    ) -> list[Point]:
        points: list[Point] = []
        cumulative_s = self.path_cumulative_s(base_path)
        for index, point in enumerate(base_path):
            tangent_x, tangent_y = self.path_tangent(base_path, index)
            normal_x = -tangent_y
            normal_y = tangent_x
            weight = self.bypass_weight(
                cumulative_s[index],
                approach_start,
                hold_start,
                hold_end,
                return_end,
            )
            offset_m = lateral_shift_m * weight
            shifted = Point()
            shifted.x = float(point.x) + normal_x * offset_m
            shifted.y = float(point.y) + normal_y * offset_m
            shifted.z = point.z
            points.append(shifted)
        return points

    def path_cumulative_s(self, points: list[Point]) -> list[float]:
        cumulative_s = [0.0]
        for start, end in zip(points, points[1:]):
            cumulative_s.append(
                cumulative_s[-1] + math.hypot(float(end.x - start.x), float(end.y - start.y))
            )
        return cumulative_s

    def path_tangent(self, points: list[Point], index: int) -> tuple[float, float]:
        if index <= 0:
            start, end = points[0], points[1]
        elif index >= len(points) - 1:
            start, end = points[-2], points[-1]
        else:
            start, end = points[index - 1], points[index + 1]
        dx = float(end.x - start.x)
        dy = float(end.y - start.y)
        length = math.hypot(dx, dy)
        if length <= 1.0e-6:
            return 1.0, 0.0
        return dx / length, dy / length

    def project_point_to_path(
        self,
        points: list[Point],
        x: float,
        y: float,
    ) -> tuple[float, float]:
        cumulative_s = self.path_cumulative_s(points)
        best_distance = math.inf
        best_s = 0.0
        best_lateral = 0.0
        for index, (start, end) in enumerate(zip(points, points[1:])):
            dx = float(end.x - start.x)
            dy = float(end.y - start.y)
            length_squared = dx * dx + dy * dy
            if length_squared <= 1.0e-9:
                continue
            ratio = ((x - float(start.x)) * dx + (y - float(start.y)) * dy) / length_squared
            ratio = clamp(ratio, 0.0, 1.0)
            projected_x = float(start.x) + ratio * dx
            projected_y = float(start.y) + ratio * dy
            error_x = x - projected_x
            error_y = y - projected_y
            distance = math.hypot(error_x, error_y)
            if distance >= best_distance:
                continue
            segment_length = math.sqrt(length_squared)
            best_distance = distance
            best_s = cumulative_s[index] + ratio * segment_length
            best_lateral = (-dy * error_x + dx * error_y) / segment_length
        return best_s, best_lateral

    def sample_path_at_s(
        self,
        points: list[Point],
        target_s: float,
    ) -> tuple[float, float, float, float]:
        cumulative_s = self.path_cumulative_s(points)
        clamped_s = clamp(target_s, 0.0, cumulative_s[-1])
        for index, (start_s, end_s) in enumerate(zip(cumulative_s, cumulative_s[1:])):
            if clamped_s > end_s and index + 1 < len(points) - 1:
                continue
            segment_length = max(1.0e-9, end_s - start_s)
            ratio = (clamped_s - start_s) / segment_length
            start = points[index]
            end = points[index + 1]
            tangent_x = float(end.x - start.x) / segment_length
            tangent_y = float(end.y - start.y) / segment_length
            return (
                float(start.x) + ratio * float(end.x - start.x),
                float(start.y) + ratio * float(end.y - start.y),
                tangent_x,
                tangent_y,
            )
        tangent_x, tangent_y = self.path_tangent(points, len(points) - 1)
        return float(points[-1].x), float(points[-1].y), tangent_x, tangent_y

    def lane_offset_at_path_s(
        self,
        base_path: list[Point],
        lane_points: list[Point],
        path_s: float,
    ) -> float | None:
        if len(base_path) < 2 or not lane_points:
            return None
        base_x, base_y, tangent_x, tangent_y = self.sample_path_at_s(base_path, path_s)
        lane_point = min(
            lane_points,
            key=lambda point: math.hypot(float(point.x) - base_x, float(point.y) - base_y),
        )
        error_x = float(lane_point.x) - base_x
        error_y = float(lane_point.y) - base_y
        return -tangent_y * error_x + tangent_x * error_y

    def required_obstacle_center_clearance(self, obstacle_width: float) -> float:
        return (
            self.vehicle_half_width_m
            + max(0.0, obstacle_width) * 0.5
            + self.obstacle_clearance_m
        )

    def resolve_bypass_obstacle(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        base_path: list[Point],
    ) -> RememberedObstacle | None:
        if scene.obstacle_on_path:
            locked_side = (
                float(self.remembered_obstacle.bypass_side)
                if self.remembered_obstacle is not None
                else 0.0
            )
            observation_x = (
                float(scene.front_obstacle_x)
                if math.isfinite(float(scene.front_obstacle_x)) and scene.front_obstacle_x < INF_DISTANCE
                else float(scene.front_obstacle_distance)
            )
            observation_x = max(0.0, observation_x)
            obstacle_s, obstacle_lateral_m = self.project_point_to_path(
                base_path,
                observation_x,
                float(scene.front_obstacle_y),
            )
            obstacle = RememberedObstacle(
                route_progress_s=self.obstacle_route_progress(route, obstacle_s),
                x=obstacle_s,
                y=obstacle_lateral_m,
                length=max(0.5, float(scene.front_obstacle_length)),
                width=max(0.5, float(scene.front_obstacle_width)),
                bypass_side=locked_side,
            )
            self.remembered_obstacle = obstacle
            return obstacle

        if self.remembered_obstacle is None:
            return None

        remembered = self.remembered_obstacle
        if route is None or not route.route_projection_valid or remembered.route_progress_s is None:
            return remembered

        estimated_s = float(remembered.route_progress_s) - float(route.progress_s)
        if estimated_s < -(remembered.length + 5.0):
            self.remembered_obstacle = None
            return None

        return RememberedObstacle(
            route_progress_s=remembered.route_progress_s,
            x=estimated_s,
            y=remembered.y,
            length=remembered.length,
            width=remembered.width,
            bypass_side=remembered.bypass_side,
        )

    def obstacle_route_progress(self, route: RouteContext | None, obstacle_x: float) -> float | None:
        if route is None or not route.route_projection_valid:
            return None
        return float(route.progress_s) + max(0.0, obstacle_x)

    def bypass_base_path_points(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
    ) -> tuple[list[Point], float, str]:
        if route is not None and route.route_projection_valid and len(route.local_path_points) >= 2:
            return list(route.local_path_points), float(route.projection_confidence), "global_route_local_path"
        if scene.centerline_valid and len(scene.centerline_points) >= 2:
            return list(scene.centerline_points), float(scene.centerline_quality), "centerline"
        if self.previous_safe_path is not None and len(self.previous_safe_path.points) >= 2:
            return list(self.previous_safe_path.points), float(self.previous_safe_path.confidence), "previous_safe_path"
        return [], 0.0, "none"

    def nominal_path_points(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
    ) -> tuple[list[Point], float, str]:
        if scene.centerline_valid and len(scene.centerline_points) >= 2:
            return list(scene.centerline_points), float(scene.centerline_quality), "centerline"
        if route is not None and route.route_projection_valid and len(route.local_path_points) >= 2:
            return list(route.local_path_points), float(route.projection_confidence), "global_route_local_path"
        if self.previous_safe_path is not None and len(self.previous_safe_path.points) >= 2:
            return list(self.previous_safe_path.points), float(self.previous_safe_path.confidence), "previous_safe_path"
        return [], 0.0, "none"

    def bypass_weight(
        self,
        x: float,
        approach_start: float,
        hold_start: float,
        hold_end: float,
        return_end: float,
    ) -> float:
        if x < approach_start or x > return_end:
            return 0.0
        if x < hold_start:
            return self.quintic_smoothstep(
                (x - approach_start) / max(1e-6, hold_start - approach_start)
            )
        if x <= hold_end:
            return 1.0
        return 1.0 - self.quintic_smoothstep(
            (x - hold_end) / max(1e-6, return_end - hold_end)
        )

    def quintic_smoothstep(self, value: float) -> float:
        t = clamp(value, 0.0, 1.0)
        return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))

    def smoothstep(self, value: float) -> float:
        t = clamp(value, 0.0, 1.0)
        return t * t * (3.0 - 2.0 * t)

    def resolve_speed_limit(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        route: RouteContext | None,
        extra_constraints: list[str],
    ) -> float:
        limits = []
        if decision.target_speed_limit_mps > 0.0:
            limits.append(decision.target_speed_limit_mps)
        elif not decision.need_stop:
            limits.append(self.base_speed_mps)

        if (
            route is not None
            and math.isfinite(float(route.speed_limit_mps))
            and float(route.speed_limit_mps) > 0.0
        ):
            limits.append(float(route.speed_limit_mps))
            extra_constraints.append("HD_MAP_SPEED_LIMIT")

        if scene.perception_health == "DEGRADED" and not self.route_based_plan_can_ignore_centerline_degradation(
            decision,
            scene,
            route,
        ):
            limits.append(self.degraded_speed_mps)
            extra_constraints.append("PERCEPTION_DEGRADED")

        if scene.centerline_valid and scene.centerline_quality < 0.55:
            limits.append(self.degraded_speed_mps)
            extra_constraints.append("CENTERLINE_QUALITY_CAP")

        if scene.obstacle_on_path:
            if not self.obstacle_bypass_speed_can_follow_decision(decision, route):
                limits.append(max(0.0, min(self.degraded_speed_mps, decision.target_speed_limit_mps)))
            extra_constraints.append("OBSTACLE_PRESENT")

        if decision.need_stop:
            stop_distance = float(decision.stop_target_distance)
            has_planned_stop_distance = (
                math.isfinite(stop_distance)
                and 0.0 < stop_distance < INF_DISTANCE
                and bool(limits)
            )
            if not has_planned_stop_distance:
                return 0.0

        if decision.need_stop and not limits:
            return 0.0

        if not limits:
            return 0.0
        return max(0.0, min(limits))

    def obstacle_bypass_speed_can_follow_decision(
        self,
        decision: DecisionCommand,
        route: RouteContext | None,
    ) -> bool:
        return (
            decision.active_behavior == "OBSTACLE"
            and decision.path_request == "BYPASS_PATH"
            and route is not None
            and route.route_projection_valid
            and len(route.local_path_points) >= 2
        )

    def route_based_plan_can_ignore_centerline_degradation(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        route: RouteContext | None,
    ) -> bool:
        route_based_request = (
            decision.active_behavior == "GLOBAL_ROUTE_FOLLOW"
            or (
                decision.active_behavior == "TRAFFIC_LIGHT"
                and decision.path_request == "ROUTE_LOCAL_PATH"
            )
            or self.obstacle_bypass_speed_can_follow_decision(decision, route)
        )
        if not route_based_request:
            return False
        if route is None or not route.route_projection_valid or len(route.local_path_points) < 2:
            return False
        # 정지선 근접 자체는 이상 상태가 아니다. 신호 FSM이 감속·정지·통과를
        # 이미 소유하므로 전역 경로가 유효한 통과 단계의 속도를 다시 낮추지 않는다.
        allowed_events = {"CENTERLINE_INVALID", "STOPLINE_NEAR"}
        if self.obstacle_bypass_speed_can_follow_decision(decision, route):
            allowed_events.add("OBSTACLE_ON_PATH")

        blocking_events = {
            event
            for event in scene.events
            if event not in allowed_events
            and not event.startswith("CENTERLINE_")
        }
        return not blocking_events

    def make_target_speed(
        self,
        decision: DecisionCommand,
        scene: SceneSummary,
        speed_limit: float,
        extra_constraints: list[str],
    ) -> TargetSpeed:
        msg = TargetSpeed()
        msg.header = scene.header
        msg.speed_limit_mps = float(speed_limit)
        msg.need_stop = bool(decision.need_stop)
        msg.stop_target_distance = float(decision.stop_target_distance)
        msg.source_behavior = decision.active_behavior
        msg.constraints = list(dict.fromkeys([*decision.constraints, *extra_constraints]))

        stop_distance = decision.stop_target_distance
        if math.isfinite(stop_distance) and stop_distance < INF_DISTANCE:
            if stop_distance <= 0.0:
                msg.target_speed_mps = 0.0
                return msg
            braking_speed = math.sqrt(max(0.0, 2.0 * self.comfortable_decel_mps2 * stop_distance))
            msg.target_speed_mps = float(clamp(braking_speed, 0.0, speed_limit))
            return msg

        if decision.need_stop:
            msg.target_speed_mps = 0.0
            return msg

        msg.target_speed_mps = float(speed_limit)
        return msg
