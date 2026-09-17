from __future__ import annotations

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand


class RecoveryFSM:
    def __init__(self, degraded_speed_mps: float) -> None:
        self.degraded_speed_mps = degraded_speed_mps
        self.state = "HOLD_PREVIOUS_PATH"

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "recovery",
    ) -> DecisionCommand:
        route_failure = (
            route is None
            or not route.route_projection_valid
            or reason.startswith("zone policy")
        )
        if route_failure or scene.perception_health == "LOST" or not scene.centerline_valid:
            self.state = "STOP_SAFE"
            return DecisionCommand(
                safety_state="DEGRADED",
                active_behavior="RECOVERY",
                fsm_state=self.state,
                selected_reason=reason,
                need_stop=True,
                target_speed_limit_mps=0.0,
                path_request="HOLD_PREVIOUS_PATH",
                constraints=["RECOVERY_STOP_SAFE"],
                risk_level="HIGH",
            )

        self.state = "DEGRADED_DRIVE"
        return DecisionCommand(
            safety_state="DEGRADED",
            active_behavior="RECOVERY",
            fsm_state=self.state,
            selected_reason=reason,
            target_speed_limit_mps=self.degraded_speed_mps,
            path_request="HOLD_PREVIOUS_PATH",
            constraints=["RECOVERY_SPEED_CAP"],
            risk_level="MEDIUM",
        )
