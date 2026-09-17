from __future__ import annotations

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand


class FinishFSM:
    def __init__(self, slow_speed_mps: float = 0.6, stop_distance_m: float = 1.0) -> None:
        self.slow_speed_mps = slow_speed_mps
        self.stop_distance_m = stop_distance_m
        self.state = "APPROACH_FINISH"

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "finish zone",
    ) -> DecisionCommand:
        distance_to_finish = route.distance_to_finish if route is not None else 0.0
        if distance_to_finish <= self.stop_distance_m:
            self.state = "MISSION_COMPLETE"
            return DecisionCommand(
                active_behavior="FINISH",
                fsm_state=self.state,
                selected_reason="finish reached",
                need_stop=True,
                target_speed_limit_mps=0.0,
                path_request="STOP_PATH",
                stop_target_distance=0.0,
                constraints=["FINISH_STOP"],
            )

        self.state = "SLOW_DOWN"
        return DecisionCommand(
            active_behavior="FINISH",
            fsm_state=self.state,
            selected_reason=reason,
            target_speed_limit_mps=self.slow_speed_mps,
            path_request="CENTERLINE",
            stop_target_distance=max(0.0, distance_to_finish),
            constraints=["FINISH_DECEL"],
        )
