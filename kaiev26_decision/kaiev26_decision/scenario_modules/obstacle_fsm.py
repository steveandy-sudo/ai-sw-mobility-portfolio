from __future__ import annotations

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand


class ObstacleFSM:
    def __init__(
        self,
        slow_speed_mps: float,
        return_speed_mps: float | None = None,
        stop_distance_m: float = 0.6,
        clear_distance_m: float = 1.5,
        min_hold_ticks: int = 4,
        fallback_memory_ticks: int = 90,
        vehicle_width_m: float = 1.18,
        obstacle_clearance_m: float = 0.30,
    ) -> None:
        self.slow_speed_mps = slow_speed_mps
        self.return_speed_mps = (
            slow_speed_mps if return_speed_mps is None else return_speed_mps
        )
        self.stop_distance_m = stop_distance_m
        self.clear_distance_m = clear_distance_m
        self.min_hold_ticks = min_hold_ticks
        self.fallback_memory_ticks = fallback_memory_ticks
        self.vehicle_half_width_m = max(0.0, vehicle_width_m * 0.5)
        self.obstacle_clearance_m = max(0.0, obstacle_clearance_m)
        self.hold_ticks = 0
        self.clear_progress_s: float | None = None
        self.state = "CLEAR"

    def reset(self) -> None:
        self.hold_ticks = 0
        self.clear_progress_s = None
        self.state = "CLEAR"

    @property
    def active(self) -> bool:
        return self.clear_progress_s is not None or self.hold_ticks > 0

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "obstacle",
    ) -> DecisionCommand:
        if scene.obstacle_on_path:
            self.remember_obstacle_until_clear(scene, route)
            if (
                scene.front_obstacle_distance <= self.stop_distance_m
                and not self.has_lateral_clearance(scene)
            ):
                self.state = "BLOCKED_STOP"
                return DecisionCommand(
                    active_behavior="OBSTACLE",
                    fsm_state=self.state,
                    selected_reason="obstacle inside stop distance",
                    need_stop=True,
                    target_speed_limit_mps=0.0,
                    path_request="STOP_PATH",
                    stop_target_distance=max(0.0, scene.front_obstacle_distance - 0.8),
                    constraints=["OBSTACLE_BLOCKED"],
                    risk_level="HIGH",
                )

            self.state = "BYPASS_READY"
            return DecisionCommand(
                active_behavior="OBSTACLE",
                fsm_state=self.state,
                selected_reason=reason,
                target_speed_limit_mps=self.slow_speed_mps,
                path_request="BYPASS_PATH",
                constraints=["OBSTACLE_SPEED_CAP", "BYPASS_REQUESTED"],
                risk_level=scene.obstacle_risk_level or "MEDIUM",
            )

        if self.memory_active(route):
            self.state = "BYPASS_HOLD"
            return DecisionCommand(
                active_behavior="OBSTACLE",
                fsm_state=self.state,
                selected_reason=self.memory_reason(route),
                target_speed_limit_mps=self.slow_speed_mps,
                path_request="BYPASS_PATH",
                constraints=["OBSTACLE_MEMORY", "OBSTACLE_SPEED_CAP", "BYPASS_REQUESTED"],
                risk_level="LOW",
            )

        if self.hold_ticks > 0:
            self.hold_ticks -= 1
            self.state = "RETURN"
            return DecisionCommand(
                active_behavior="OBSTACLE",
                fsm_state=self.state,
                selected_reason="obstacle clear debounce",
                target_speed_limit_mps=self.return_speed_mps,
                path_request="CENTERLINE",
                constraints=["OBSTACLE_CLEAR_HOLD"],
                risk_level="LOW",
            )

        self.state = "CLEAR"
        return DecisionCommand(
            active_behavior="OBSTACLE",
            fsm_state=self.state,
            selected_reason="obstacle clear",
            path_request="CENTERLINE",
            risk_level="LOW",
        )

    def has_lateral_clearance(self, scene: SceneSummary) -> bool:
        obstacle_half_width_m = max(0.0, float(scene.front_obstacle_width) * 0.5)
        required_m = (
            self.vehicle_half_width_m
            + obstacle_half_width_m
            + self.obstacle_clearance_m
        )
        return abs(float(scene.front_obstacle_y)) >= required_m

    def remember_obstacle_until_clear(self, scene: SceneSummary, route: RouteContext | None) -> None:
        self.hold_ticks = max(self.hold_ticks, self.min_hold_ticks)
        if route is None or not route.route_projection_valid:
            self.hold_ticks = max(self.hold_ticks, self.fallback_memory_ticks)
            return

        obstacle_x = float(scene.front_obstacle_x)
        if obstacle_x >= 1.0e6:
            obstacle_x = float(scene.front_obstacle_distance)
        obstacle_x = max(0.0, obstacle_x)
        obstacle_length = max(0.5, float(scene.front_obstacle_length))
        front_distance = float(scene.front_obstacle_distance)
        if front_distance >= 1.0e6:
            front_distance = max(0.0, obstacle_x - obstacle_length * 0.5)
        front_distance = max(0.0, front_distance)
        clear_progress_s = float(route.progress_s) + front_distance + obstacle_length + self.clear_distance_m

        if self.clear_progress_s is None:
            self.clear_progress_s = clear_progress_s
        else:
            self.clear_progress_s = max(self.clear_progress_s, clear_progress_s)

    def memory_active(self, route: RouteContext | None) -> bool:
        if self.clear_progress_s is None:
            return False
        if route is None or not route.route_projection_valid:
            self.hold_ticks = max(self.hold_ticks, self.fallback_memory_ticks)
            return True
        if float(route.progress_s) < self.clear_progress_s:
            return True

        self.clear_progress_s = None
        self.hold_ticks = max(self.hold_ticks, self.min_hold_ticks)
        return False

    def memory_reason(self, route: RouteContext | None) -> str:
        if self.clear_progress_s is None or route is None or not route.route_projection_valid:
            return "remembered obstacle until clear"
        remaining_m = max(0.0, self.clear_progress_s - float(route.progress_s))
        return f"remembered obstacle clear in {remaining_m:.1f}m"
