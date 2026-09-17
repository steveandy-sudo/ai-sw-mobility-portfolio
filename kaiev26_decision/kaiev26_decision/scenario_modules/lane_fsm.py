from __future__ import annotations

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand


class LaneFSM:
    def __init__(
        self,
        base_speed_mps: float,
        degraded_speed_mps: float,
        route_rejoin_cross_track_m: float = 0.8,
        route_rejoin_heading_error_rad: float = 0.45,
        route_rejoin_exit_cross_track_m: float = 0.30,
        route_rejoin_exit_heading_error_rad: float = 0.15,
        rejoin_speed_min_mps: float = 1.4,
        rejoin_speed_max_mps: float = 2.6,
        rejoin_slow_cross_track_m: float = 2.5,
        rejoin_slow_heading_error_rad: float = 0.8,
    ) -> None:
        self.base_speed_mps = base_speed_mps
        self.degraded_speed_mps = degraded_speed_mps
        self.route_rejoin_cross_track_m = route_rejoin_cross_track_m
        self.route_rejoin_heading_error_rad = route_rejoin_heading_error_rad
        self.route_rejoin_exit_cross_track_m = min(
            route_rejoin_cross_track_m,
            max(0.0, route_rejoin_exit_cross_track_m),
        )
        self.route_rejoin_exit_heading_error_rad = min(
            route_rejoin_heading_error_rad,
            max(0.0, route_rejoin_exit_heading_error_rad),
        )
        self.rejoin_speed_min_mps = rejoin_speed_min_mps
        self.rejoin_speed_max_mps = max(rejoin_speed_min_mps, rejoin_speed_max_mps)
        self.rejoin_slow_cross_track_m = max(
            route_rejoin_cross_track_m,
            rejoin_slow_cross_track_m,
        )
        self.rejoin_slow_heading_error_rad = max(
            route_rejoin_heading_error_rad,
            rejoin_slow_heading_error_rad,
        )
        self.state = "LANE_GOOD"
        self.rejoin_latched = False

    def request_route_rejoin(self) -> None:
        self.rejoin_latched = True

    def reset(self) -> None:
        self.state = "LANE_GOOD"
        self.rejoin_latched = False

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "lane follow",
    ) -> DecisionCommand:
        if self.route_local_path_valid(route):
            if self.route_rejoin_required(route):
                self.state = "ROUTE_REJOIN"
                constraints = ["GLOBAL_ROUTE_REJOIN"]
                if abs(float(route.cross_track_error)) >= self.route_rejoin_cross_track_m:
                    constraints.append("ROUTE_CROSS_TRACK_ERROR")
                if abs(float(route.heading_error)) >= self.route_rejoin_heading_error_rad:
                    constraints.append("ROUTE_HEADING_ERROR")

                return DecisionCommand(
                    safety_state="DEGRADED",
                    active_behavior="ROUTE_REJOIN",
                    fsm_state=self.state,
                    selected_reason=(
                        "route deviation "
                        f"cte={float(route.cross_track_error):+.2f}m "
                        f"heading={float(route.heading_error):+.2f}rad"
                    ),
                    target_speed_limit_mps=self.route_rejoin_speed(route),
                    path_request="ROUTE_REJOIN_PATH",
                    constraints=constraints,
                    risk_level="MEDIUM",
                )

            self.state = "GLOBAL_ROUTE_FOLLOW"
            constraints = []
            speed_limit = self.base_speed_mps
            safety_state = "NORMAL"
            risk_level = "LOW"
            if not scene.centerline_valid:
                constraints.append("CENTERLINE_LOST_ROUTE_FOLLOW")
                safety_state = "DEGRADED"
                risk_level = "MEDIUM"
            elif scene.centerline_quality < 0.55 or scene.centerline_visible_length < 5.0:
                constraints.append("CENTERLINE_DEGRADED_ROUTE_FOLLOW")
                safety_state = "DEGRADED"
                risk_level = "MEDIUM"

            return DecisionCommand(
                safety_state=safety_state,
                active_behavior="GLOBAL_ROUTE_FOLLOW",
                fsm_state=self.state,
                selected_reason="following global route local path",
                target_speed_limit_mps=speed_limit,
                path_request="ROUTE_LOCAL_PATH",
                constraints=constraints,
                risk_level=risk_level,
            )

        if not scene.centerline_valid:
            self.state = "LANE_LOST"
            return DecisionCommand(
                safety_state="DEGRADED",
                active_behavior="LANE_FOLLOW",
                fsm_state=self.state,
                selected_reason="centerline lost",
                need_stop=True,
                target_speed_limit_mps=0.0,
                path_request="HOLD_PREVIOUS_PATH",
                constraints=["CENTERLINE_LOST"],
                risk_level="HIGH",
            )

        if scene.centerline_quality < 0.45 or scene.centerline_visible_length < 5.0:
            self.state = "LANE_DEGRADED"
            return DecisionCommand(
                safety_state="DEGRADED",
                active_behavior="LANE_FOLLOW",
                fsm_state=self.state,
                selected_reason="centerline degraded",
                target_speed_limit_mps=self.degraded_speed_mps,
                path_request="CENTERLINE",
                constraints=["LOW_CENTERLINE_QUALITY"],
                risk_level="MEDIUM",
            )

        self.state = "LANE_GOOD"
        return DecisionCommand(
            safety_state="NORMAL",
            active_behavior="LANE_FOLLOW",
            fsm_state=self.state,
            selected_reason=reason,
            target_speed_limit_mps=self.base_speed_mps,
            path_request="CENTERLINE",
            risk_level="LOW",
        )

    def route_local_path_valid(self, route: RouteContext | None) -> bool:
        return (
            route is not None
            and route.route_projection_valid
            and len(route.local_path_points) >= 2
        )

    def route_rejoin_required(self, route: RouteContext | None) -> bool:
        if route is None:
            return self.rejoin_latched

        cross_track_error = abs(float(route.cross_track_error))
        heading_error = abs(float(route.heading_error))
        if self.rejoin_latched:
            if (
                cross_track_error <= self.route_rejoin_exit_cross_track_m
                and heading_error <= self.route_rejoin_exit_heading_error_rad
            ):
                self.rejoin_latched = False
        elif (
            cross_track_error >= self.route_rejoin_cross_track_m
            or heading_error >= self.route_rejoin_heading_error_rad
        ):
            self.rejoin_latched = True
        return self.rejoin_latched

    def route_rejoin_speed(self, route: RouteContext) -> float:
        cte_severity = self.normalized_excess(
            abs(float(route.cross_track_error)),
            self.route_rejoin_cross_track_m,
            self.rejoin_slow_cross_track_m,
        )
        heading_severity = self.normalized_excess(
            abs(float(route.heading_error)),
            self.route_rejoin_heading_error_rad,
            self.rejoin_slow_heading_error_rad,
        )
        severity = max(cte_severity, heading_severity)
        speed_range = self.rejoin_speed_max_mps - self.rejoin_speed_min_mps
        return self.rejoin_speed_max_mps - speed_range * severity

    @staticmethod
    def normalized_excess(value: float, start: float, full: float) -> float:
        span = max(1.0e-6, full - start)
        return max(0.0, min(1.0, (value - start) / span))
