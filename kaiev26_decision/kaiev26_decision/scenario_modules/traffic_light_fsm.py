from __future__ import annotations

import math

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState

from kaiev26_decision.common import DecisionCommand


class TrafficLightFSM:
    def __init__(
        self,
        base_speed_mps: float,
        approach_distance_m: float,
        stop_hold_distance_m: float = 1.0,
        release_confirm_sec: float = 0.4,
        vehicle_front_overhang_m: float = 1.98,
        stopline_clearance_m: float = 1.0,
        yellow_stop_decel_mps2: float = 2.0,
        yellow_reaction_time_sec: float = 0.25,
        pass_line_jump_m: float = 3.0,
        stopped_speed_mps: float = 0.1,
        pre_brake_decel_distance_m: float = 6.0,
        brake_entry_speed_mps: float = 2.0,
    ) -> None:
        self.base_speed_mps = base_speed_mps
        self.approach_distance_m = approach_distance_m
        self.stop_hold_distance_m = stop_hold_distance_m
        self.release_confirm_sec = max(0.0, release_confirm_sec)
        self.vehicle_front_overhang_m = max(0.0, vehicle_front_overhang_m)
        self.stopline_clearance_m = max(0.0, stopline_clearance_m)
        self.yellow_stop_decel_mps2 = max(0.1, yellow_stop_decel_mps2)
        self.yellow_reaction_time_sec = max(0.0, yellow_reaction_time_sec)
        self.pass_line_jump_m = max(0.5, pass_line_jump_m)
        self.stopped_speed_mps = max(0.0, stopped_speed_mps)
        self.pre_brake_decel_distance_m = max(0.1, pre_brake_decel_distance_m)
        self.brake_entry_speed_mps = max(0.0, brake_entry_speed_mps)
        self.green_since_s: float | None = None
        self.last_stopline_distance: float | None = None
        self.state = "APPROACH"
        self.hard_brake_latched = False

    def reset(self) -> None:
        self.green_since_s = None
        self.last_stopline_distance = None
        self.state = "APPROACH"
        self.hard_brake_latched = False

    @property
    def active(self) -> bool:
        return self.state in {
            "DECELERATE",
            "PRE_DECEL",
            "STOP_CONFIRM",
            "STOP_HOLD",
            "WAIT_GREEN",
            "PASSING",
        }

    def scene_time_s(self, scene: SceneSummary) -> float:
        return float(scene.header.stamp.sec) + float(scene.header.stamp.nanosec) * 1.0e-9

    def finite_stopline_distance(self, scene: SceneSummary) -> float | None:
        distance = float(scene.stopline_distance)
        if not math.isfinite(distance) or distance >= 1.0e5:
            return None
        return max(0.0, distance)

    def remember_stopline(self, scene: SceneSummary) -> None:
        distance = self.finite_stopline_distance(scene)
        if distance is not None:
            self.last_stopline_distance = distance

    def stopline_has_passed(
        self,
        scene: SceneSummary,
        route: RouteContext | None = None,
        expected_stop_line_id: str = "",
    ) -> bool:
        if (
            expected_stop_line_id
            and route is not None
            and route.route_projection_valid
            and route.next_stop_line_id != expected_stop_line_id
        ):
            return True
        current = self.finite_stopline_distance(scene)
        if current is None:
            return self.last_stopline_distance is not None
        if self.last_stopline_distance is None:
            self.last_stopline_distance = current
            return False
        passed = current > self.last_stopline_distance + self.pass_line_jump_m
        self.last_stopline_distance = current
        return passed

    def should_commit_on_entry(
        self,
        scene: SceneSummary,
        vehicle: VehicleState | None,
    ) -> bool:
        stopline_distance = self.finite_stopline_distance(scene)
        if stopline_distance is None:
            return False
        speed_mps = abs(float(vehicle.speed_mps)) if vehicle is not None else 0.0
        front_distance = stopline_distance - self.vehicle_front_overhang_m
        if front_distance <= 0.0 and speed_mps > 0.2:
            return True
        if scene.traffic_light_state != SceneSummary.TRAFFIC_YELLOW:
            return False
        available_distance = max(0.0, front_distance - self.stopline_clearance_m)
        required_distance = (
            speed_mps * self.yellow_reaction_time_sec
            + speed_mps * speed_mps / (2.0 * self.yellow_stop_decel_mps2)
        )
        return required_distance >= available_distance

    def passing_command(self, reason: str) -> DecisionCommand:
        self.hard_brake_latched = False
        return DecisionCommand(
            active_behavior="TRAFFIC_LIGHT",
            fsm_state="PASSING",
            selected_reason=reason,
            need_stop=False,
            target_speed_limit_mps=self.base_speed_mps,
            path_request="ROUTE_LOCAL_PATH",
            constraints=["SIGNAL_PASS_COMMITTED"],
            risk_level="MEDIUM",
        )

    def stop_distance_before_line(self, stopline_distance: float) -> float:
        return max(
            0.0,
            float(stopline_distance) - self.vehicle_front_overhang_m - self.stopline_clearance_m,
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
    def hard_brake_command(state: str, reason: str) -> DecisionCommand:
        return DecisionCommand(
            active_behavior="TRAFFIC_LIGHT",
            fsm_state=state,
            selected_reason=reason,
            need_stop=True,
            target_speed_limit_mps=0.0,
            path_request="STOP_PATH",
            stop_target_distance=0.0,
            constraints=["SIGNAL_HARD_BRAKE"],
            risk_level="MEDIUM",
        )

    def update(
        self,
        scene: SceneSummary,
        route: RouteContext | None,
        vehicle: VehicleState | None,
        reason: str = "traffic light",
        *,
        expected_stop_line_id: str = "",
        arrow_maneuver: str = "LEFT",
        brake_trigger_route_s: float | None = None,
    ) -> DecisionCommand:
        is_go_light = (
            scene.traffic_light_state == SceneSummary.TRAFFIC_GREEN
            or (
                scene.traffic_light_state == SceneSummary.TRAFFIC_ARROW
                and route is not None
                and arrow_maneuver in {"LEFT", "RIGHT"}
                and route.next_maneuver == arrow_maneuver
            )
        )
        trigger_distance = self.distance_to_brake_trigger(
            route, brake_trigger_route_s
        )

        if self.state == "PASSING":
            if self.stopline_has_passed(scene, route, expected_stop_line_id):
                self.state = "START"
                self.hard_brake_latched = False
                self.last_stopline_distance = None
                return DecisionCommand(
                    active_behavior="TRAFFIC_LIGHT",
                    fsm_state=self.state,
                    selected_reason="stopline passed",
                    target_speed_limit_mps=self.base_speed_mps,
                    path_request="ROUTE_LOCAL_PATH",
                    constraints=["SIGNAL_PASS_COMPLETE"],
                )
            return self.passing_command("finishing committed intersection entry")

        if (
            self.active
            and not self.hard_brake_latched
            and expected_stop_line_id
            and route is not None
            and route.route_projection_valid
            and route.next_stop_line_id != expected_stop_line_id
        ):
            self.state = "START"
            self.hard_brake_latched = False
            self.green_since_s = None
            self.last_stopline_distance = None
            return DecisionCommand(
                active_behavior="TRAFFIC_LIGHT",
                fsm_state=self.state,
                selected_reason="stopline already passed; do not stop in intersection",
                target_speed_limit_mps=self.base_speed_mps,
                path_request="ROUTE_LOCAL_PATH",
                constraints=["SIGNAL_PASS_COMPLETE"],
            )

        if not self.active and self.should_commit_on_entry(scene, vehicle):
            self.state = "PASSING"
            self.remember_stopline(scene)
            return self.passing_command("stopping is no longer physically safe")

        if self.state == "STOP_CONFIRM" or self.hard_brake_latched:
            speed_mps = abs(float(vehicle.speed_mps)) if vehicle is not None else math.inf
            if speed_mps > self.stopped_speed_mps:
                self.state = "STOP_CONFIRM"
                self.hard_brake_latched = True
                return self.hard_brake_command(
                    self.state, "hard brake latched until measured stop"
                )
            self.state = "WAIT_GREEN"

        if self.active:
            if is_go_light:
                now_s = self.scene_time_s(scene)
                if self.green_since_s is None or now_s < self.green_since_s:
                    self.green_since_s = now_s
                if now_s - self.green_since_s >= self.release_confirm_sec - 1.0e-6:
                    self.state = "PASSING"
                    self.remember_stopline(scene)
                    return self.passing_command("green confirmed; crossing stopline")
            else:
                self.green_since_s = None

            if self.state in {"STOP_HOLD", "WAIT_GREEN"}:
                self.state = "WAIT_GREEN"
                if self.hard_brake_latched:
                    command = self.hard_brake_command(
                        self.state, "holding hard brake until green"
                    )
                    return command
                return DecisionCommand(
                    active_behavior="TRAFFIC_LIGHT",
                    fsm_state=self.state,
                    selected_reason="holding stop until green",
                    need_stop=True,
                    target_speed_limit_mps=0.0,
                    path_request="STOP_PATH",
                    stop_target_distance=self.stop_distance_before_line(scene.stopline_distance),
                    constraints=["RED_LIGHT_HOLD"],
                    risk_level="MEDIUM",
                )

        if trigger_distance is not None:
            self.remember_stopline(scene)
            if trigger_distance <= 0.0:
                self.state = "STOP_CONFIRM"
                self.hard_brake_latched = True
                return self.hard_brake_command(
                    self.state, "configured brake waypoint reached"
                )
            if trigger_distance <= self.pre_brake_decel_distance_m:
                self.state = "PRE_DECEL"
                return DecisionCommand(
                    active_behavior="TRAFFIC_LIGHT",
                    fsm_state=self.state,
                    selected_reason=reason,
                    need_stop=False,
                    target_speed_limit_mps=self.pre_brake_speed(trigger_distance),
                    path_request="ROUTE_LOCAL_PATH",
                    constraints=["SIGNAL_BRAKE_TRIGGER_PRE_DECEL"],
                    risk_level="MEDIUM",
                )
            self.state = "DECELERATE"
            return DecisionCommand(
                active_behavior="TRAFFIC_LIGHT",
                fsm_state=self.state,
                selected_reason=reason,
                need_stop=False,
                target_speed_limit_mps=self.base_speed_mps,
                path_request="ROUTE_LOCAL_PATH",
                constraints=["SIGNAL_BRAKE_TRIGGER_APPROACH"],
                risk_level="MEDIUM",
            )

        if not self.active:
            self.green_since_s = None

        stop_target_distance = self.stop_distance_before_line(scene.stopline_distance)
        self.remember_stopline(scene)
        if stop_target_distance <= self.stop_hold_distance_m:
            self.state = "STOP_HOLD"
            return DecisionCommand(
                active_behavior="TRAFFIC_LIGHT",
                fsm_state=self.state,
                selected_reason="stopline reached before signal release",
                need_stop=True,
                target_speed_limit_mps=0.0,
                path_request="STOP_PATH",
                stop_target_distance=0.0,
                constraints=["STOPLINE_HOLD"],
                risk_level="MEDIUM",
            )

        self.state = "DECELERATE"
        return DecisionCommand(
            active_behavior="TRAFFIC_LIGHT",
            fsm_state=self.state,
            selected_reason=reason,
            need_stop=True,
            target_speed_limit_mps=self.base_speed_mps,
            path_request="STOP_PATH",
            stop_target_distance=stop_target_distance,
            constraints=["STOPLINE_DECEL"],
            risk_level="MEDIUM",
        )
