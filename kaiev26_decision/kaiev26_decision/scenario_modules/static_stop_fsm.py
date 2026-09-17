from __future__ import annotations

import math

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand, INF_DISTANCE


class StaticStopFSM:
    """Stop at a mapped line, optionally hold for a fixed time, then release."""

    def __init__(
        self,
        base_speed_mps: float,
        approach_distance_m: float,
        vehicle_front_overhang_m: float,
        stopline_clearance_m: float,
        hold_distance_m: float = 0.35,
        stopped_speed_mps: float = 0.1,
        pre_brake_decel_distance_m: float = 6.0,
        brake_entry_speed_mps: float = 2.0,
    ) -> None:
        self.base_speed_mps = max(0.0, base_speed_mps)
        self.approach_distance_m = max(0.0, approach_distance_m)
        self.vehicle_front_overhang_m = max(0.0, vehicle_front_overhang_m)
        self.stopline_clearance_m = max(0.0, stopline_clearance_m)
        self.hold_distance_m = max(0.0, hold_distance_m)
        self.stopped_speed_mps = max(0.0, stopped_speed_mps)
        self.pre_brake_decel_distance_m = max(0.1, pre_brake_decel_distance_m)
        self.brake_entry_speed_mps = max(0.0, brake_entry_speed_mps)
        self.state = "IDLE"
        self.active_event_id = ""
        self.expected_stop_line_id = ""
        self.hold_started_s: float | None = None

    def reset(self) -> None:
        self.state = "IDLE"
        self.active_event_id = ""
        self.expected_stop_line_id = ""
        self.hold_started_s = None

    @property
    def active(self) -> bool:
        return self.state in {
            "DECELERATE",
            "PRE_DECEL",
            "STOP_CONFIRM",
            "HOLD_3S",
            "STOP_HOLD",
            "RELEASE",
        }

    def reset_for_event(self, event_id: str, stop_line_id: str) -> None:
        self.state = "IDLE"
        self.active_event_id = event_id
        self.expected_stop_line_id = stop_line_id
        self.hold_started_s = None

    def stop_target_distance(self, route: RouteContext | None) -> float:
        if route is None:
            return 0.0
        distance = float(route.distance_to_stopline)
        if not math.isfinite(distance) or distance >= INF_DISTANCE:
            return 0.0
        return max(
            0.0,
            distance - self.vehicle_front_overhang_m - self.stopline_clearance_m,
        )

    def expected_line_passed(self, route: RouteContext | None) -> bool:
        return (
            bool(self.expected_stop_line_id)
            and route is not None
            and route.route_projection_valid
            and route.next_stop_line_id != self.expected_stop_line_id
        )

    @staticmethod
    def distance_to_brake_trigger(
        route: RouteContext | None,
        brake_trigger_route_s: float | None,
    ) -> float | None:
        if (
            route is None
            or not route.route_projection_valid
            or brake_trigger_route_s is None
            or not math.isfinite(brake_trigger_route_s)
        ):
            return None
        return float(brake_trigger_route_s) - float(route.progress_s)

    def pre_brake_speed(self, distance_to_trigger_m: float) -> float:
        ratio = max(
            0.0,
            min(1.0, distance_to_trigger_m / self.pre_brake_decel_distance_m),
        )
        entry_speed = min(self.base_speed_mps, self.brake_entry_speed_mps)
        return entry_speed + (self.base_speed_mps - entry_speed) * ratio

    @staticmethod
    def scene_time_s(scene: SceneSummary) -> float:
        return float(scene.header.stamp.sec) + float(scene.header.stamp.nanosec) * 1.0e-9

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "static stopline ahead",
        *,
        event_id: str = "",
        stop_line_id: str = "",
        hold_sec: float = 0.0,
        now_s: float | None = None,
        brake_trigger_route_s: float | None = None,
    ) -> DecisionCommand:
        timed_stop = bool(event_id) and hold_sec > 0.0
        if event_id and event_id != self.active_event_id:
            self.reset_for_event(event_id, stop_line_id)

        if self.state == "RELEASE":
            if self.expected_line_passed(route):
                self.state = "COMPLETE"
                return DecisionCommand(
                    active_behavior="STATIC_STOP",
                    fsm_state=self.state,
                    selected_reason="timed stop completed and stopline passed",
                    target_speed_limit_mps=self.base_speed_mps,
                    path_request="ROUTE_LOCAL_PATH",
                    constraints=["TIMED_STOP_COMPLETE"],
                )
            return DecisionCommand(
                active_behavior="STATIC_STOP",
                fsm_state=self.state,
                selected_reason="releasing after timed stop",
                target_speed_limit_mps=self.base_speed_mps,
                path_request="ROUTE_LOCAL_PATH",
                constraints=["TIMED_STOP_RELEASE"],
            )

        stop_distance = self.stop_target_distance(route)
        trigger_distance = self.distance_to_brake_trigger(
            route, brake_trigger_route_s
        )
        at_line = (
            trigger_distance <= 0.0
            if trigger_distance is not None
            else stop_distance <= self.hold_distance_m
        )
        stopped = (
            vehicle is not None
            and abs(float(vehicle.speed_mps)) <= self.stopped_speed_mps
        )

        if (
            trigger_distance is not None
            and not at_line
            and self.state not in {"STOP_CONFIRM", "HOLD_3S", "STOP_HOLD"}
        ):
            if trigger_distance > self.pre_brake_decel_distance_m:
                self.state = "DECELERATE"
                target_speed = self.base_speed_mps
                constraint = "BRAKE_TRIGGER_APPROACH"
            else:
                self.state = "PRE_DECEL"
                target_speed = self.pre_brake_speed(trigger_distance)
                constraint = "BRAKE_TRIGGER_PRE_DECEL"
            return DecisionCommand(
                active_behavior="STATIC_STOP",
                fsm_state=self.state,
                selected_reason=reason,
                need_stop=False,
                target_speed_limit_mps=target_speed,
                path_request="ROUTE_LOCAL_PATH",
                constraints=[constraint],
                risk_level="LOW",
            )

        if at_line and timed_stop:
            if not stopped:
                self.state = "STOP_CONFIRM"
                self.hold_started_s = None
                return self.stop_command(
                    self.state,
                    "waiting for measured vehicle stop",
                    "TIMED_STOP_CONFIRM",
                )

            current_time_s = self.scene_time_s(scene) if now_s is None else now_s
            if self.hold_started_s is None or current_time_s < self.hold_started_s:
                self.hold_started_s = current_time_s
            elapsed_s = current_time_s - self.hold_started_s
            if elapsed_s + 1.0e-6 >= hold_sec:
                self.state = "RELEASE"
                return DecisionCommand(
                    active_behavior="STATIC_STOP",
                    fsm_state=self.state,
                    selected_reason=f"{hold_sec:.1f}s stop completed; releasing",
                    target_speed_limit_mps=self.base_speed_mps,
                    path_request="ROUTE_LOCAL_PATH",
                    constraints=["TIMED_STOP_RELEASE"],
                )
            self.state = "HOLD_3S"
            return self.stop_command(
                self.state,
                f"holding timed stop {elapsed_s:.1f}/{hold_sec:.1f}s",
                "TIMED_STOP_HOLD",
            )

        if at_line:
            self.state = "STOP_HOLD"
            return self.stop_command(
                self.state,
                "holding at configured test stopline",
                "STATIC_STOPLINE_HOLD",
            )

        self.state = "DECELERATE"
        return DecisionCommand(
            safety_state="NORMAL",
            active_behavior="STATIC_STOP",
            fsm_state=self.state,
            selected_reason=reason,
            need_stop=True,
            target_speed_limit_mps=self.base_speed_mps,
            path_request="STOP_PATH",
            stop_target_distance=stop_distance,
            constraints=["TIMED_STOP_DECEL" if timed_stop else "STATIC_STOPLINE_DECEL"],
            risk_level="LOW",
        )

    @staticmethod
    def stop_command(state: str, reason: str, constraint: str) -> DecisionCommand:
        return DecisionCommand(
            safety_state="NORMAL",
            active_behavior="STATIC_STOP",
            fsm_state=state,
            selected_reason=reason,
            need_stop=True,
            target_speed_limit_mps=0.0,
            path_request="STOP_PATH",
            stop_target_distance=0.0,
            constraints=[constraint],
            risk_level="LOW",
        )
