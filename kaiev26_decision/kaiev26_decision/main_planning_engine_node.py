#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import replace
import time

from kaiev26_msgs.msg import (
    BehaviorDecision,
    Centerline,
    PerceptionObjectArray,
    RouteContext,
    SceneSummary,
    TargetSpeed,
    VehicleState,
)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_msgs.msg import Bool

from kaiev26_decision.aeb_logic import AEBLogic
from kaiev26_decision.common import DecisionCommand, INF_DISTANCE, ScenarioChoice, spin_until_shutdown, stamp_age_sec
from kaiev26_decision.planning_core import PlanningCore
from kaiev26_decision.priority_selector import PrioritySelector
from kaiev26_decision.scenario_modules.finish_fsm import FinishFSM
from kaiev26_decision.scenario_modules.lane_fsm import LaneFSM
from kaiev26_decision.scenario_modules.obstacle_fsm import ObstacleFSM
from kaiev26_decision.scenario_modules.recovery_fsm import RecoveryFSM
from kaiev26_decision.scenario_modules.static_stop_fsm import StaticStopFSM
from kaiev26_decision.scenario_modules.traffic_light_fsm import TrafficLightFSM
from kaiev26_decision.zone_policy import ZonePolicy, load_zone_policy


class MainPlanningEngineNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_main_planning_engine")
        self.declare_parameter("scene_summary_topic", "/planning/scene_summary")
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("vehicle_state_topic", "/vehicle/state")
        self.declare_parameter("objects_topic", "/perception/objects")
        self.declare_parameter("target_path_topic", "/planning/target_path")
        self.declare_parameter("target_speed_topic", "/planning/target_speed")
        self.declare_parameter("behavior_decision_topic", "/planning/behavior_decision")
        self.declare_parameter("debug_active_behavior_topic", "/debug/active_behavior")
        self.declare_parameter("debug_fsm_state_topic", "/debug/fsm_state")
        self.declare_parameter("debug_planning_latency_topic", "/debug/planning_latency")
        self.declare_parameter("mission_policy_config", "")
        self.declare_parameter("mission_mode", "all")
        self.declare_parameter("test_stop_hold_sec", 3.0)
        self.declare_parameter("require_readiness", False)
        self.declare_parameter("readiness_topic", "/planning/route_ready")
        self.declare_parameter("base_speed_mps", 8.0)
        self.declare_parameter("degraded_speed_mps", 8.0)
        self.declare_parameter("static_stop_enabled", False)
        self.declare_parameter("static_stop_stopped_speed_mps", 0.1)
        self.declare_parameter("obstacle_slow_speed_mps", 2.2)
        self.declare_parameter("obstacle_return_speed_mps", 2.6)
        self.declare_parameter("stopline_approach_distance_m", 18.0)
        self.declare_parameter("traffic_release_confirm_sec", 0.4)
        self.declare_parameter("yellow_stop_decel_mps2", 2.0)
        self.declare_parameter("yellow_reaction_time_sec", 0.25)
        self.declare_parameter("signal_pass_line_jump_m", 3.0)
        self.declare_parameter("comfortable_decel_mps2", 1.4)
        self.declare_parameter("pre_brake_decel_distance_m", 6.0)
        self.declare_parameter("brake_entry_speed_mps", 2.0)
        self.declare_parameter("vehicle_front_overhang_m", 1.98)
        self.declare_parameter("vehicle_width_m", 1.18)
        self.declare_parameter("obstacle_clearance_m", 0.30)
        self.declare_parameter("obstacle_lane_departure_allowed", True)
        self.declare_parameter("obstacle_side_deadband_m", 0.30)
        self.declare_parameter("obstacle_preferred_side", "left")
        self.declare_parameter("obstacle_transition_length_m", 10.0)
        self.declare_parameter("obstacle_return_length_m", 10.0)
        self.declare_parameter("stopline_clearance_m", 1.0)
        self.declare_parameter("route_rejoin_cross_track_m", 0.8)
        self.declare_parameter("route_rejoin_heading_error_rad", 0.45)
        self.declare_parameter("route_rejoin_exit_cross_track_m", 0.30)
        self.declare_parameter("route_rejoin_exit_heading_error_rad", 0.15)
        self.declare_parameter("rejoin_speed_min_mps", 1.4)
        self.declare_parameter("rejoin_speed_max_mps", 2.6)
        self.declare_parameter("rejoin_slow_cross_track_m", 2.5)
        self.declare_parameter("rejoin_slow_heading_error_rad", 0.8)
        self.declare_parameter("planning_deadline_ms", 50.0)
        self.declare_parameter("route_stale_sec", 0.5)
        self.declare_parameter("vehicle_stale_sec", 0.25)
        self.declare_parameter("decision_policy", "brake_inspection")
        self.declare_parameter("aeb_stale_objects_sec", 0.25)
        self.declare_parameter("aeb_obstacle_path_half_width_m", 1.35)
        self.declare_parameter("aeb_warning_ttc_sec", 2.0)
        self.declare_parameter("aeb_brake_ttc_sec", 0.8)
        self.declare_parameter("aeb_warning_distance_m", 5.0)
        self.declare_parameter("aeb_emergency_distance_m", 1.4)
        self.declare_parameter("aeb_bypass_brake_ttc_sec", 0.25)
        self.declare_parameter("aeb_bypass_emergency_distance_m", 0.6)
        self.declare_parameter("aeb_release_clear_ticks", 8)

        self.base_speed_mps = float(self.get_parameter("base_speed_mps").value)
        self.require_readiness = bool(
            self.get_parameter("require_readiness").value
        )
        self.route_ready = not self.require_readiness
        self.mission_mode = str(
            self.get_parameter("mission_mode").value
        ).strip().lower()
        if self.mission_mode not in {"all", "stopline", "avoidance"}:
            raise ValueError(
                f"mission_mode must be all, stopline, or avoidance; got "
                f"[{self.mission_mode}]"
            )
        self.degraded_speed_mps = float(self.get_parameter("degraded_speed_mps").value)
        self.static_stop_enabled = bool(
            self.get_parameter("static_stop_enabled").value
        )
        self.static_stop_stopped_speed_mps = float(
            self.get_parameter("static_stop_stopped_speed_mps").value
        )
        self.obstacle_slow_speed_mps = float(self.get_parameter("obstacle_slow_speed_mps").value)
        self.obstacle_return_speed_mps = float(
            self.get_parameter("obstacle_return_speed_mps").value
        )
        self.stopline_approach_distance_m = float(self.get_parameter("stopline_approach_distance_m").value)
        self.traffic_release_confirm_sec = float(
            self.get_parameter("traffic_release_confirm_sec").value
        )
        self.yellow_stop_decel_mps2 = float(
            self.get_parameter("yellow_stop_decel_mps2").value
        )
        self.yellow_reaction_time_sec = float(
            self.get_parameter("yellow_reaction_time_sec").value
        )
        self.signal_pass_line_jump_m = float(
            self.get_parameter("signal_pass_line_jump_m").value
        )
        self.comfortable_decel_mps2 = float(self.get_parameter("comfortable_decel_mps2").value)
        self.pre_brake_decel_distance_m = float(
            self.get_parameter("pre_brake_decel_distance_m").value
        )
        self.brake_entry_speed_mps = float(
            self.get_parameter("brake_entry_speed_mps").value
        )
        self.vehicle_front_overhang_m = float(self.get_parameter("vehicle_front_overhang_m").value)
        self.vehicle_width_m = float(self.get_parameter("vehicle_width_m").value)
        self.obstacle_clearance_m = float(self.get_parameter("obstacle_clearance_m").value)
        self.obstacle_lane_departure_allowed = bool(
            self.get_parameter("obstacle_lane_departure_allowed").value
        )
        self.obstacle_side_deadband_m = float(
            self.get_parameter("obstacle_side_deadband_m").value
        )
        self.obstacle_preferred_side = str(
            self.get_parameter("obstacle_preferred_side").value
        )
        self.obstacle_transition_length_m = float(
            self.get_parameter("obstacle_transition_length_m").value
        )
        self.obstacle_return_length_m = float(
            self.get_parameter("obstacle_return_length_m").value
        )
        self.stopline_clearance_m = float(self.get_parameter("stopline_clearance_m").value)
        self.route_rejoin_cross_track_m = float(self.get_parameter("route_rejoin_cross_track_m").value)
        self.route_rejoin_heading_error_rad = float(self.get_parameter("route_rejoin_heading_error_rad").value)
        self.route_rejoin_exit_cross_track_m = float(
            self.get_parameter("route_rejoin_exit_cross_track_m").value
        )
        self.route_rejoin_exit_heading_error_rad = float(
            self.get_parameter("route_rejoin_exit_heading_error_rad").value
        )
        self.rejoin_speed_min_mps = float(self.get_parameter("rejoin_speed_min_mps").value)
        self.rejoin_speed_max_mps = float(self.get_parameter("rejoin_speed_max_mps").value)
        self.rejoin_slow_cross_track_m = float(
            self.get_parameter("rejoin_slow_cross_track_m").value
        )
        self.rejoin_slow_heading_error_rad = float(
            self.get_parameter("rejoin_slow_heading_error_rad").value
        )
        self.planning_deadline_ms = float(self.get_parameter("planning_deadline_ms").value)
        self.route_stale_sec = float(self.get_parameter("route_stale_sec").value)
        self.vehicle_stale_sec = float(self.get_parameter("vehicle_stale_sec").value)
        policy_path = str(self.get_parameter("mission_policy_config").value).strip()
        self.zone_policy: ZonePolicy | None = (
            load_zone_policy(policy_path) if policy_path else None
        )

        self.latest_route_context: RouteContext | None = None
        self.latest_vehicle_state: VehicleState | None = None
        self.latest_objects: PerceptionObjectArray | None = None
        self.last_active_behavior = "LANE_FOLLOW"
        self.latched_mission_choice: ScenarioChoice | None = None
        self.manual_override_active = False
        self.aeb = AEBLogic(
            enabled=(
                self.mission_mode != "stopline"
                and str(self.get_parameter("decision_policy").value)
                == "brake_inspection"
            ),
            stale_objects_sec=float(
                self.get_parameter("aeb_stale_objects_sec").value
            ),
            obstacle_path_half_width_m=float(
                self.get_parameter("aeb_obstacle_path_half_width_m").value
            ),
            warning_ttc_sec=float(
                self.get_parameter("aeb_warning_ttc_sec").value
            ),
            brake_ttc_sec=float(self.get_parameter("aeb_brake_ttc_sec").value),
            warning_distance_m=float(
                self.get_parameter("aeb_warning_distance_m").value
            ),
            emergency_distance_m=float(
                self.get_parameter("aeb_emergency_distance_m").value
            ),
            bypass_brake_ttc_sec=float(
                self.get_parameter("aeb_bypass_brake_ttc_sec").value
            ),
            bypass_emergency_distance_m=float(
                self.get_parameter("aeb_bypass_emergency_distance_m").value
            ),
            release_clear_ticks=int(
                self.get_parameter("aeb_release_clear_ticks").value
            ),
        )

        self.selector = PrioritySelector(
            self.stopline_approach_distance_m,
            static_stop_enabled=self.static_stop_enabled,
            zone_policy=self.zone_policy,
            mission_mode=self.mission_mode,
            test_stop_hold_sec=float(
                self.get_parameter("test_stop_hold_sec").value
            ),
        )
        self.lane_fsm = LaneFSM(
            self.base_speed_mps,
            self.degraded_speed_mps,
            self.route_rejoin_cross_track_m,
            self.route_rejoin_heading_error_rad,
            self.route_rejoin_exit_cross_track_m,
            self.route_rejoin_exit_heading_error_rad,
            self.rejoin_speed_min_mps,
            self.rejoin_speed_max_mps,
            self.rejoin_slow_cross_track_m,
            self.rejoin_slow_heading_error_rad,
        )
        self.traffic_light_fsm = TrafficLightFSM(
            self.base_speed_mps,
            self.stopline_approach_distance_m,
            release_confirm_sec=self.traffic_release_confirm_sec,
            vehicle_front_overhang_m=self.vehicle_front_overhang_m,
            stopline_clearance_m=self.stopline_clearance_m,
            yellow_stop_decel_mps2=self.yellow_stop_decel_mps2,
            yellow_reaction_time_sec=self.yellow_reaction_time_sec,
            pass_line_jump_m=self.signal_pass_line_jump_m,
            stopped_speed_mps=self.static_stop_stopped_speed_mps,
            pre_brake_decel_distance_m=self.pre_brake_decel_distance_m,
            brake_entry_speed_mps=self.brake_entry_speed_mps,
        )
        self.obstacle_fsm = ObstacleFSM(
            self.obstacle_slow_speed_mps,
            return_speed_mps=self.obstacle_return_speed_mps,
            vehicle_width_m=self.vehicle_width_m,
            obstacle_clearance_m=self.obstacle_clearance_m,
        )
        self.finish_fsm = FinishFSM()
        self.recovery_fsm = RecoveryFSM(self.degraded_speed_mps)
        self.static_stop_fsm = StaticStopFSM(
            self.base_speed_mps,
            self.stopline_approach_distance_m,
            self.vehicle_front_overhang_m,
            self.stopline_clearance_m,
            stopped_speed_mps=self.static_stop_stopped_speed_mps,
            pre_brake_decel_distance_m=self.pre_brake_decel_distance_m,
            brake_entry_speed_mps=self.brake_entry_speed_mps,
        )
        self.planning_core = PlanningCore(
            self.base_speed_mps,
            self.degraded_speed_mps,
            comfortable_decel_mps2=self.comfortable_decel_mps2,
            vehicle_front_overhang_m=self.vehicle_front_overhang_m,
            vehicle_width_m=self.vehicle_width_m,
            obstacle_clearance_m=self.obstacle_clearance_m,
            obstacle_lane_departure_allowed=self.obstacle_lane_departure_allowed,
            obstacle_side_deadband_m=self.obstacle_side_deadband_m,
            obstacle_preferred_side=self.obstacle_preferred_side,
            obstacle_transition_length_m=self.obstacle_transition_length_m,
            obstacle_return_length_m=self.obstacle_return_length_m,
            stopline_clearance_m=self.stopline_clearance_m,
        )

        self.target_path_pub = self.create_publisher(
            Centerline,
            str(self.get_parameter("target_path_topic").value),
            10,
        )
        self.target_speed_pub = self.create_publisher(
            TargetSpeed,
            str(self.get_parameter("target_speed_topic").value),
            10,
        )
        self.behavior_decision_pub = self.create_publisher(
            BehaviorDecision,
            str(self.get_parameter("behavior_decision_topic").value),
            10,
        )
        self.debug_active_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_active_behavior_topic").value),
            10,
        )
        self.debug_fsm_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_fsm_state_topic").value),
            10,
        )
        self.debug_latency_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("debug_planning_latency_topic").value),
            10,
        )

        self.create_subscription(
            SceneSummary,
            str(self.get_parameter("scene_summary_topic").value),
            self.on_scene_summary,
            10,
        )
        self.create_subscription(
            RouteContext,
            str(self.get_parameter("route_context_topic").value),
            self.on_route_context,
            10,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle_state,
            10,
        )
        self.create_subscription(
            PerceptionObjectArray,
            str(self.get_parameter("objects_topic").value),
            self.on_objects,
            10,
        )
        if self.mission_mode == "stopline":
            self.create_timer(0.05, self.publish_route_only_scene)
        if self.require_readiness:
            self.create_subscription(
                Bool,
                str(self.get_parameter("readiness_topic").value),
                self.on_readiness,
                10,
            )

    def on_route_context(self, msg: RouteContext) -> None:
        self.latest_route_context = msg

    def on_vehicle_state(self, msg: VehicleState) -> None:
        self.latest_vehicle_state = msg

    def on_objects(self, msg: PerceptionObjectArray) -> None:
        self.latest_objects = msg

    def on_readiness(self, msg: Bool) -> None:
        self.route_ready = bool(msg.data)

    def on_scene_summary(self, scene: SceneSummary) -> None:
        if self.mission_mode == "stopline":
            return
        self.process_scene(scene)

    def publish_route_only_scene(self) -> None:
        route = self.latest_route_context
        if route is None:
            return
        scene = SceneSummary()
        scene.header.stamp = self.get_clock().now().to_msg()
        scene.header.frame_id = route.header.frame_id or "base_footprint"
        scene.stopline_distance = float(route.distance_to_stopline)
        scene.traffic_light_state = SceneSummary.TRAFFIC_UNKNOWN
        scene.front_obstacle_distance = INF_DISTANCE
        scene.front_obstacle_x = INF_DISTANCE
        scene.obstacle_risk_level = "LOW"
        scene.perception_health = "GOOD"
        self.process_scene(scene)

    def process_scene(self, scene: SceneSummary) -> None:
        start = time.perf_counter()
        decision = (
            self.run_planning_tick(scene)
            if self.route_ready
            else self.emergency_stop(scene, "ROUTE_NOT_READY")
        )
        extra_constraints = self.collect_constraints(scene)
        target_path, target_speed = self.planning_core.make_plan(
            decision,
            scene,
            self.latest_route_context,
            extra_constraints,
        )

        latency_ms = (time.perf_counter() - start) * 1000.0
        if latency_ms > self.planning_deadline_ms:
            target_speed.target_speed_mps = min(target_speed.target_speed_mps, self.degraded_speed_mps)
            target_speed.constraints.append("PLANNING_DEADLINE_EXCEEDED")

        self.behavior_decision_pub.publish(self.to_behavior_decision_msg(scene, decision))
        self.target_path_pub.publish(target_path)
        self.target_speed_pub.publish(target_speed)
        self.publish_debug(decision, latency_ms)
        self.last_active_behavior = decision.active_behavior

    def run_planning_tick(self, scene: SceneSummary) -> DecisionCommand:
        route = self.latest_route_context
        vehicle = self.latest_vehicle_state
        # E-stop 상태는 단일 출처인 통합 차량 상태에서 읽는다.
        estop_active = vehicle is not None and bool(vehicle.estop_active)
        if estop_active:
            return self.emergency_stop(scene, "ESTOP")

        manual_command = self.update_drive_mode(route, vehicle)
        if manual_command is not None:
            return manual_command

        bypass_active = (
            self.last_active_behavior == "OBSTACLE"
            and self.obstacle_fsm.state in {"BYPASS_READY", "BYPASS_HOLD"}
        )
        if self.aeb.brake_required(
            self.get_clock().now(),
            self.latest_objects,
            vehicle,
            bypass_active=bypass_active,
        ):
            return self.emergency_stop(scene, "AEB")

        choice = self.policy_input_health_choice()
        if choice is None:
            choice = self.selector.choose(scene, route, vehicle, estop_active)
        choice = self.apply_scenario_hysteresis(choice)

        if choice.behavior == "RECOVERY":
            decision = self.recovery_fsm.update(scene, route, vehicle, choice.reason)
        elif choice.behavior == "OBSTACLE":
            decision = self.obstacle_fsm.update(scene, route, vehicle, choice.reason)
        elif choice.behavior == "TRAFFIC_LIGHT":
            decision = self.traffic_light_fsm.update(
                scene,
                route,
                vehicle,
                choice.reason,
                expected_stop_line_id=choice.stop_line_id,
                arrow_maneuver=choice.arrow_maneuver or "LEFT",
                brake_trigger_route_s=choice.brake_trigger_route_s,
            )
        elif choice.behavior == "STATIC_STOP":
            decision = self.static_stop_fsm.update(
                scene,
                route,
                vehicle,
                choice.reason,
                event_id=choice.event_id,
                stop_line_id=choice.stop_line_id,
                hold_sec=choice.hold_sec,
                now_s=self.get_clock().now().nanoseconds * 1.0e-9,
                brake_trigger_route_s=choice.brake_trigger_route_s,
            )
        elif choice.behavior == "FINISH":
            decision = self.finish_fsm.update(scene, route, vehicle, choice.reason)
        elif choice.behavior == "EMERGENCY":
            decision = self.emergency_stop(scene, choice.reason)
        else:
            decision = self.lane_fsm.update(scene, route, vehicle, choice.reason)

        if "TIMED_STOP_COMPLETE" in decision.constraints or "SIGNAL_PASS_COMPLETE" in decision.constraints:
            self.selector.complete_event(choice.event_id)
            self.latched_mission_choice = None
        return self.annotate_mission_choice(decision, choice)

    def update_drive_mode(
        self,
        route: RouteContext | None,
        vehicle: VehicleState | None,
    ) -> DecisionCommand | None:
        manual_selected = (
            vehicle is not None and vehicle.mode == VehicleState.MODE_MANUAL
        )
        if manual_selected:
            if not self.manual_override_active:
                self.reset_transient_planning_state()
            self.manual_override_active = True
            route_available = (
                route is not None
                and route.route_projection_valid
                and len(route.local_path_points) >= 2
            )
            return DecisionCommand(
                safety_state="NORMAL",
                active_behavior="MANUAL_OVERRIDE",
                fsm_state="MANUAL_CONTROL",
                selected_reason="physical manual mode selected",
                target_speed_limit_mps=abs(float(vehicle.speed_mps)),
                path_request=(
                    "ROUTE_LOCAL_PATH" if route_available else "HOLD_PREVIOUS_PATH"
                ),
                constraints=["MANUAL_MODE", "AUTONOMY_SUSPENDED"],
                risk_level="LOW",
            )

        if self.manual_override_active:
            self.reset_transient_planning_state()
            self.lane_fsm.request_route_rejoin()
            self.manual_override_active = False
        return None

    def reset_transient_planning_state(self) -> None:
        self.latched_mission_choice = None
        self.selector.reset_transient_state()
        self.lane_fsm.reset()
        self.traffic_light_fsm.reset()
        self.static_stop_fsm.reset()
        self.obstacle_fsm.reset()
        self.planning_core.reset_transient_state()

    def policy_input_health_choice(self) -> ScenarioChoice | None:
        if self.zone_policy is None:
            return None
        now = self.get_clock().now()
        if self.latest_route_context is None:
            return ScenarioChoice("RECOVERY", "zone policy waiting for route context")
        if stamp_age_sec(now, self.latest_route_context.header.stamp) > self.route_stale_sec:
            return ScenarioChoice("RECOVERY", "zone policy route context stale")
        if self.latest_vehicle_state is None:
            return ScenarioChoice("RECOVERY", "zone policy waiting for vehicle state")
        if stamp_age_sec(now, self.latest_vehicle_state.header.stamp) > self.vehicle_stale_sec:
            return ScenarioChoice("RECOVERY", "zone policy vehicle state stale")
        return None

    def apply_scenario_hysteresis(self, choice: ScenarioChoice) -> ScenarioChoice:
        if choice.behavior in {"EMERGENCY", "RECOVERY", "OBSTACLE"}:
            return choice
        if choice.behavior in {"TRAFFIC_LIGHT", "STATIC_STOP"}:
            self.latched_mission_choice = choice
        if self.obstacle_fsm.active:
            return ScenarioChoice("OBSTACLE", "obstacle debounce hold")
        if self.traffic_light_fsm.active:
            return self.latched_mission_choice or ScenarioChoice(
                "TRAFFIC_LIGHT", "traffic light hold"
            )
        if self.static_stop_fsm.active:
            return self.latched_mission_choice or ScenarioChoice(
                "STATIC_STOP", "static stopline hold"
            )
        self.latched_mission_choice = None
        return choice

    @staticmethod
    def annotate_mission_choice(
        decision: DecisionCommand,
        choice: ScenarioChoice,
    ) -> DecisionCommand:
        annotations = []
        if choice.stage_id:
            annotations.append(f"ZONE_POLICY:{choice.stage_id}")
        if choice.event_id:
            annotations.append(f"EVENT:{choice.event_id}")
        if choice.brake_trigger_waypoint is not None:
            annotations.append(f"BRAKE_TRIGGER_WP:{choice.brake_trigger_waypoint}")
        if not annotations:
            return decision
        return replace(
            decision,
            constraints=list(decision.constraints) + annotations,
        )

    def emergency_stop(self, scene: SceneSummary, reason: str) -> DecisionCommand:
        return DecisionCommand(
            safety_state="EMERGENCY",
            active_behavior="EMERGENCY",
            fsm_state="STOP",
            selected_reason=reason,
            need_stop=True,
            target_speed_limit_mps=0.0,
            path_request="STOP_PATH",
            stop_target_distance=0.0,
            constraints=[reason],
            risk_level="CRITICAL",
        )

    def collect_constraints(self, scene: SceneSummary) -> list[str]:
        constraints: list[str] = []
        now = self.get_clock().now()

        if self.latest_route_context is None:
            constraints.append("ROUTE_CONTEXT_MISSING")
        else:
            route_age = stamp_age_sec(now, self.latest_route_context.header.stamp)
            if route_age > self.route_stale_sec:
                constraints.append("ROUTE_CONTEXT_STALE")

        if self.latest_vehicle_state is None:
            constraints.append("VEHICLE_STATE_MISSING")
        else:
            vehicle_age = stamp_age_sec(now, self.latest_vehicle_state.header.stamp)
            if vehicle_age > self.vehicle_stale_sec:
                constraints.append("VEHICLE_STATE_STALE")

        if (
            self.selector.signal_control_enabled(self.latest_route_context)
            and scene.traffic_light_state
            in {SceneSummary.TRAFFIC_RED, SceneSummary.TRAFFIC_YELLOW}
        ):
            constraints.append("SIGNAL_STOP_CONSTRAINT")

        if self.aeb.state == "WARNING":
            constraints.append("AEB_WARNING")

        return constraints

    def to_behavior_decision_msg(self, scene: SceneSummary, decision: DecisionCommand) -> BehaviorDecision:
        msg = BehaviorDecision()
        msg.header = scene.header
        msg.safety_state = decision.safety_state
        msg.active_behavior = decision.active_behavior
        msg.fsm_state = decision.fsm_state
        msg.selected_reason = decision.selected_reason
        msg.need_stop = bool(decision.need_stop)
        msg.target_speed_limit_mps = float(decision.target_speed_limit_mps)
        msg.path_request = decision.path_request
        msg.stop_target_distance = float(decision.stop_target_distance if decision.stop_target_distance < INF_DISTANCE else INF_DISTANCE)
        msg.constraints = list(decision.constraints)
        msg.risk_level = decision.risk_level
        return msg

    def publish_debug(self, decision: DecisionCommand, latency_ms: float) -> None:
        active = String()
        active.data = f"{decision.active_behavior}: {decision.selected_reason}"
        self.debug_active_pub.publish(active)

        fsm = String()
        fsm.data = decision.fsm_state
        self.debug_fsm_pub.publish(fsm)

        latency = Float32()
        latency.data = float(latency_ms)
        self.debug_latency_pub.publish(latency)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MainPlanningEngineNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
