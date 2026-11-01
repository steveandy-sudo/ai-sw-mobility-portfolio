#!/usr/bin/env python3
"""Main planning engine: world model, event tracker, modules and the arbiter, ticked on a timer.

Each tick every module is called and returns only speed caps, stop requests and path requests
(arbiter.OUTPUT_OWNERSHIP). The arbiter takes the minimum cap, stops on any stop request and picks
the path owner. Stops reach TargetSpeed only through common.stop_command.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from kaiev26_msgs.msg import (
    BehaviorDecision,
    Centerline,
    PerceptionObjectArray,
    RouteContext,
    TargetSpeed,
    TrafficLightObservation,
    TrafficLightObservationArray,
    TrafficSignalPhaseArray,
    VehicleState,
)
from kaiev26_motion_control.input_loss import InputLossLatch
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, Float32, Header, String

from kaiev26_decision.aeb_logic import AEBLogic
from kaiev26_decision.arbiter import ModuleOutput, arbitrate
from kaiev26_decision.common import (
    INF_DISTANCE,
    StoppingModel,
    require_finite_parameter,
    spin_until_shutdown,
    stamp_age_sec,
    stop_command,
)
from kaiev26_decision.curve_speed_application import curve_speed_cap, speed_zone_cap
from kaiev26_decision.curve_speed_policy import (
    load_curve_speed_policy,
    resolve_curve_approach_decel_override,
    resolve_curve_speed_scale_override,
)
from kaiev26_decision.event_permission import (
    ARROW,
    GREEN,
    RED,
    STOPPED_PHASES,
    UNKNOWN,
    YELLOW,
    Arrival,
    EventPermission,
    OwnLight,
    PhaseHint,
)
from kaiev26_decision.mission_events import EventSettings, build_stop_events, obstacle_zone_finish_check
from kaiev26_decision.obstacle_behavior import PRE_SLOW, ObstacleBehavior, IDLE as OBSTACLE_IDLE
from kaiev26_decision.obstacle_observation import ObstacleObservationBuffer, stamp_seconds
from kaiev26_decision.obstacle_sequence import ObstacleSequencePlanner
from kaiev26_decision.obstacle_world import ObstacleWorld, ObstacleZone, Pose2D, route_pose
from kaiev26_decision.perception_gateway_node import OBSERVATION_QOS, lights_by_map_id
from kaiev26_decision.route_zone_manager_node import load_landmark_stoplines, load_polyline_route
from kaiev26_decision.signal_phase_tracker import (
    TRAFFIC_ARROW,
    TRAFFIC_GREEN,
    TRAFFIC_RED,
    TRAFFIC_UNKNOWN,
    TRAFFIC_YELLOW,
)
from kaiev26_decision.signal_schedule import (
    SOURCE_CAMERA,
    SOURCE_DISTRUST,
    SOURCE_LIMIT,
    SOURCE_MARGIN,
    SOURCE_SCHEDULE,
    SOURCE_UNSURE,
    ScheduleChangeClock,
    ScheduleConfig,
    ScheduleState,
    SignalSourceSelector,
)
from kaiev26_decision.stop_point import (
    CLEARED,
    FREE,
    HELD,
    KIND_FINISH,
    KIND_SIGNAL,
    EventTracker,
    StopPointMachine,
)
from kaiev26_decision.zone_policy import ZonePolicy, ZonePolicyRuntime, load_zone_policy


LIGHT_STATES = {
    TrafficLightObservation.STATE_RED: RED,
    TrafficLightObservation.STATE_YELLOW: YELLOW,
    TrafficLightObservation.STATE_GREEN: GREEN,
    TrafficLightObservation.STATE_ARROW: ARROW,
}
# OwnLight states as the signal_schedule codes, and back.
LIGHT_CODES = {
    UNKNOWN: TRAFFIC_UNKNOWN,
    RED: TRAFFIC_RED,
    YELLOW: TRAFFIC_YELLOW,
    GREEN: TRAFFIC_GREEN,
    ARROW: TRAFFIC_ARROW,
}
CODE_LIGHTS = {code: state for state, code in LIGHT_CODES.items()}
# Running past the route end ends the route projection. Once the finish brake is latched this is
# the planned overrun (finish_overrun_m), not an input loss (finish_brake_holds).
ROUTE_END_PROBLEMS = frozenset({"ROUTE_PROJECTION_INVALID"})
# TargetSpeed constraint: the planned stop holds (need_stop stays) but motion control may release
# the brake servo; the car has stood still long enough (standstill_brake_release).
STANDSTILL_BRAKE_RELEASE = "STANDSTILL_BRAKE_RELEASE"


class MainPlanningEngineNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_main_planning_engine")
        declare = self.declare_parameter
        declare("route_context_topic", "/planning/route_context")
        declare("vehicle_state_topic", "/vehicle/state")
        declare("objects_topic", "/perception/objects")
        declare("traffic_lights_topic", "/perception/traffic_lights")
        declare("signal_phases_topic", "/planning/traffic_signal_phases")
        declare("obstacle_odometry_topic", "/localization/odometry")
        declare("target_path_topic", "/planning/target_path")
        declare("target_speed_topic", "/planning/target_speed")
        declare("behavior_decision_topic", "/planning/behavior_decision")
        declare("debug_active_behavior_topic", "/debug/active_behavior")
        declare("debug_fsm_state_topic", "/debug/fsm_state")
        declare("debug_planning_latency_topic", "/debug/planning_latency")
        # Age [s] of the route context each planning tick used; -1 when none was received.
        declare("debug_route_context_age_topic", "/debug/route_context_age")
        # Age [s] of the vehicle state each tick held, MANUAL too; -1 when none was received.
        declare("debug_vehicle_state_age_topic", "/debug/vehicle_state_age")
        declare("signal_diagnostics_topic", "/diagnostics")
        declare("mission_policy_config", "")
        declare("landmark_config", "")
        declare("route_config", "")
        declare(
            "curve_speed_scale",
            -1.0,
            ParameterDescriptor(read_only=True, description="Startup curve speed scale; -1 uses route YAML"),
        )
        # False drops the curve cap entirely: curves run at the straight course speed.
        declare(
            "curve_speed_enabled",
            True,
            ParameterDescriptor(read_only=True, description="Curve speed cap; false is off"),
        )
        # Cap each curve zone at sqrt(lateral accel * radius) and slow into it at the car's real
        # deceleration (decision_pipeline.yaml). 0 / -1 keep the route YAML behaviour.
        declare(
            "curve_lateral_accel_mps2",
            0.0,
            ParameterDescriptor(read_only=True, description="Curve radius cap; 0 is off"),
        )
        declare(
            "curve_approach_decel_mps2",
            -1.0,
            ParameterDescriptor(read_only=True, description="Curve approach decel; -1 uses route YAML"),
        )
        declare("mission_mode", "all")
        declare("test_stop_hold_sec", 3.0)
        declare("require_readiness", False)
        declare("readiness_topic", "/planning/route_ready")
        declare("base_speed_mps", 8.0)
        declare("planning_rate_hz", 30.0)
        declare("planning_deadline_ms", 50.0)
        declare("route_stale_sec", 0.5)
        # In the obstacle stage a route context up to this old still drives the bypass instead of
        # the input-loss rule (2026-10-17 145703/114251: on the car the route the engine used was
        # 0.3-0.505 s old on bypass search ticks, and every tick past 0.5 s stopped the car).
        # Older than this the input-loss rule (D2) applies as elsewhere.
        declare("obstacle_route_stale_sec", 1.0)
        declare("vehicle_stale_sec", 0.25)
        declare("input_loss_brake_delay_sec", 1.0)
        declare("static_stop_stopped_speed_mps", 0.1)
        declare("traffic_lights_stale_sec", 0.35)
        declare("signal_phase_stale_sec", 0.5)
        declare("traffic_release_confirm_sec", 0.4)
        declare("intersection_escape_unknown_confirm_sec", 1.0)
        declare("signal_unknown_after_permission_confirm_sec", 0.3)
        declare("signal_unknown_before_permission_grace_sec", 0.3)
        declare("signal_unknown_wait_limit_sec", 110.0)
        # Approaching on a scheduled permission, pass only when the front bumper reaches the line
        # this long before the permission ends, decided once at the brake point or the commit
        # distance, whichever comes first; else stop at the brake point (event_permission).
        declare("signal_arrival_margin_sec", 1.0)
        # Signal input from the aligned phase schedule, the camera only rejects it (signal_schedule).
        declare("signal_schedule_transition_margin_sec", 1.0)
        declare("signal_schedule_reject_frames", 10)
        declare("signal_schedule_reject_sec", 0.5)
        declare("signal_schedule_distrust_opposite_sec", 3.0)
        declare("signal_schedule_distrust_go_sec", 2.0)
        # Comma-separated light ids or numbers that decide on the camera only (e.g. "318").
        # Stopped at the event, a scheduled permission launches only when sure (probability and
        # time left), and the schedule holds back a permission the camera reads for at most the
        # limit per stop (2026-10-17 kty 17d49de). Zero limit: none.
        declare("signal_schedule_launch_min_probability", 0.9)
        declare("signal_schedule_launch_min_remaining_sec", 4.0)
        declare("signal_schedule_withhold_limit_sec", 100.0)
        declare("pre_brake_decel_distance_m", 8.0)
        declare("brake_entry_speed_mps", 2.0)
        declare("brake_delay_sec", 0.70)
        declare("brake_decel_mps2", 3.9)
        declare("stopline_brake_margin_m", 1.0)
        declare("finish_approach_distance_m", 20.0)
        declare("finish_slow_speed_mps", 0.6)
        declare("finish_stop_distance_m", 1.0)
        declare("finish_overrun_m", 5.0)
        # Surveyed finish line (policy finish_line_route_s): the surveyed point of the car (the GNSS
        # antenna, tf base_link -> gps x) ahead of the rear axle, and how far past the line it must
        # be before the brake.
        declare("finish_line_reference_forward_m", 0.426)
        declare("finish_line_pass_margin_m", 2.0)
        # Planned stop held this long at standstill: motion control may release the brake servo
        # (STANDSTILL_BRAKE_RELEASE). 0 keeps the brake for the whole hold.
        declare("standstill_brake_release_sec", 2.0)
        declare("vehicle_front_overhang_m", 1.98)
        declare("vehicle_width_m", 1.18)
        declare("objects_stale_sec", 0.5)
        declare("obstacle_fallback_speed_mps", 1.6)
        declare("obstacle_bypass_speed_mps", 3.0)
        # Forced (best-effort) bypass path; the fallback speed when not set (< 0).
        declare("obstacle_forced_speed_mps", -1.0)
        declare("obstacle_curve_speed_window_sec", 0.5)
        declare("obstacle_lateral_accel_mps2", 1.4)
        declare("obstacle_max_steering_rate_deg_s", 0.0)
        declare("obstacle_steering_gradient_window_m", 1.0)
        declare("obstacle_clearance_m", 0.50)
        declare("obstacle_centerline_departure_m", 0.5)
        declare("obstacle_transition_length_m", 12.0)
        declare("obstacle_return_length_m", 12.0)
        declare("obstacle_row_memory_sec", 0.8)
        declare("obstacle_row_track_match_m", 1.25)
        declare("obstacle_row_history_size", 7)
        declare("obstacle_slow_clearance_m", 0.25)
        declare("obstacle_route_half_width_m", 2.0)
        declare("obstacle_row_pair_nominal_gap_m", 6.0)
        declare("obstacle_right_passage_margin_m", 0.3)
        # Past the zone end, a committed bypass releases the zone speed cap once the rear axle is
        # this far past its last row and the bypass path is this close to the route.
        declare("obstacle_release_past_row_m", 3.0)
        declare("obstacle_release_offset_m", 0.5)
        declare("obstacle_association_m", 1.0)
        declare("obstacle_confirm_frames", 10)
        declare("obstacle_unconfirmed_memory_sec", 0.8)
        declare("obstacle_max_bottom_height_m", 2.0)
        declare("obstacle_min_top_height_m", 0.2)
        declare("decision_policy", "brake_inspection")
        declare("aeb_stale_objects_sec", 0.25)
        declare("aeb_obstacle_path_half_width_m", 1.35)
        declare("aeb_warning_ttc_sec", 2.0)
        declare("aeb_brake_ttc_sec", 0.8)
        declare("aeb_warning_distance_m", 5.0)
        declare("aeb_emergency_distance_m", 1.4)
        declare("aeb_bypass_brake_ttc_sec", 0.25)
        declare("aeb_bypass_emergency_distance_m", 0.6)
        declare("aeb_release_clear_ticks", 8)
        p = self.param

        self.mission_mode = str(p("mission_mode")).strip().lower()
        self.base_speed_mps = float(p("base_speed_mps"))
        self.standstill_brake_release_sec = require_finite_parameter(
            "standstill_brake_release_sec", p("standstill_brake_release_sec"), minimum=0.0)
        self.standstill_since_sec: float | None = None
        self.route_stale_sec = float(p("route_stale_sec"))
        self.obstacle_route_stale_sec = max(
            self.route_stale_sec,
            require_finite_parameter("obstacle_route_stale_sec", p("obstacle_route_stale_sec"), minimum=0.0),
        )
        self.vehicle_stale_sec = float(p("vehicle_stale_sec"))
        self.light_stale_sec = float(p("traffic_lights_stale_sec"))
        self.phase_stale_sec = float(p("signal_phase_stale_sec"))
        self.objects_stale_sec = float(p("objects_stale_sec"))
        self.planning_deadline_ms = float(p("planning_deadline_ms"))
        self.front_overhang_m = float(p("vehicle_front_overhang_m"))
        stopped_speed = float(p("static_stop_stopped_speed_mps"))
        self.require_readiness = bool(p("require_readiness"))
        self.route_ready = not self.require_readiness
        # The readiness gate holds the start only. Once it passed, losing it warns.
        self.route_ready_seen = self.route_ready

        route_config = str(p("route_config"))
        self.route_geometry, _ = load_polyline_route(route_config)
        policy_path = str(p("mission_policy_config")).strip()
        self.zone_policy: ZonePolicy | None = load_zone_policy(policy_path) if policy_path else None
        self.policy_runtime = ZonePolicyRuntime(self.zone_policy) if self.zone_policy else None
        landmark_path = str(p("landmark_config")).strip()
        stopline_s = {}
        if landmark_path:
            loaded, _ = load_landmark_stoplines(self.route_geometry, landmark_path)
            stopline_s = {item.landmark_id: item.progress_s for item in loaded}
        settings = EventSettings(
            default_brake_margin_m=require_finite_parameter(
                "stopline_brake_margin_m", p("stopline_brake_margin_m"), minimum=0.0),
            test_stop_hold_sec=float(p("test_stop_hold_sec")),
            finish_approach_m=require_finite_parameter(
                "finish_approach_distance_m", p("finish_approach_distance_m"), minimum=0.1),
            finish_slow_speed_mps=require_finite_parameter(
                "finish_slow_speed_mps", p("finish_slow_speed_mps"), minimum=0.0),
            finish_stop_distance_m=require_finite_parameter(
                "finish_stop_distance_m", p("finish_stop_distance_m"), minimum=0.0),
            finish_overrun_m=require_finite_parameter("finish_overrun_m", p("finish_overrun_m"), minimum=0.0),
            finish_line_reference_forward_m=require_finite_parameter(
                "finish_line_reference_forward_m", p("finish_line_reference_forward_m")),
            finish_line_pass_margin_m=require_finite_parameter(
                "finish_line_pass_margin_m", p("finish_line_pass_margin_m"), minimum=0.0),
        )
        stopping_model = StoppingModel(
            brake_delay_sec=require_finite_parameter("brake_delay_sec", p("brake_delay_sec"), minimum=0.0),
            brake_decel_mps2=require_finite_parameter(
                "brake_decel_mps2", p("brake_decel_mps2"), minimum=0.0, minimum_exclusive=True),
        )
        stage_speeds = [
            stage.course_speed_mps for stage in (self.zone_policy.stages if self.zone_policy else ())
            if stage.course_speed_mps is not None
        ]
        if stage_speeds:
            # The finish watch starts before the finish brake point at the fastest stage speed;
            # entered past it, the car brakes at once (13 m/s: 27.8 m before the line, past the
            # 20 m default). 2 m covers a planning tick. A surveyed finish line has its own
            # fixed brake station and watch (build_stop_events).
            brake_gap = (
                self.front_overhang_m + stopping_model.distance_m(max(stage_speeds)) - settings.finish_overrun_m)
            settings = replace(settings, finish_approach_m=max(settings.finish_approach_m, brake_gap + 2.0))
        events = build_stop_events(
            self.zone_policy, stopline_s, self.route_geometry.total_length, self.mission_mode, settings)
        finish_event = events[-1]
        if finish_event.brake_s is not None:
            self.get_logger().info(
                f"finish brake at rear-axle route s {finish_event.brake_s:.2f} m (surveyed line, "
                f"route end {finish_event.line_s:.2f} m); no finish slow-down before it"
            )
        self.tracker = EventTracker(events, {
            "front_overhang_m": self.front_overhang_m,
            "stopping_model": stopping_model,
            "stopped_speed_mps": stopped_speed,
            "pre_decel_window_m": float(p("pre_brake_decel_distance_m")),
            "pre_decel_cap_mps": min(self.base_speed_mps, float(p("brake_entry_speed_mps"))),
        })
        self.permission = EventPermission(
            confirm_sec=float(p("traffic_release_confirm_sec")),
            lapse_hold_sec=require_finite_parameter(
                "signal_unknown_after_permission_confirm_sec",
                p("signal_unknown_after_permission_confirm_sec"), minimum=0.0),
            unknown_grace_sec=require_finite_parameter(
                "signal_unknown_before_permission_grace_sec",
                p("signal_unknown_before_permission_grace_sec"), minimum=0.0),
            escape_sec=float(p("intersection_escape_unknown_confirm_sec")),
            unknown_wait_limit_sec=require_finite_parameter(
                "signal_unknown_wait_limit_sec", p("signal_unknown_wait_limit_sec")),
            stopped_speed_mps=stopped_speed,
            arrival_margin_sec=require_finite_parameter(
                "signal_arrival_margin_sec", p("signal_arrival_margin_sec"), minimum=0.0),
        )
        self.signal_selector = SignalSourceSelector(ScheduleConfig(
            transition_margin_sec=float(p("signal_schedule_transition_margin_sec")),
            reject_frames=int(p("signal_schedule_reject_frames")),
            reject_sec=float(p("signal_schedule_reject_sec")),
            distrust_opposite_sec=float(p("signal_schedule_distrust_opposite_sec")),
            distrust_go_sec=float(p("signal_schedule_distrust_go_sec")),
            launch_min_probability=float(p("signal_schedule_launch_min_probability")),
            launch_min_remaining_sec=float(p("signal_schedule_launch_min_remaining_sec")),
            withhold_limit_sec=float(p("signal_schedule_withhold_limit_sec")),
        ))
        self.input_latch = InputLossLatch(float(p("input_loss_brake_delay_sec")), stopped_speed)
        self.curve_policy = load_curve_speed_policy(
            route_config,
            speed_scale_override=resolve_curve_speed_scale_override(p("curve_speed_scale")),
            lateral_accel_mps2=float(p("curve_lateral_accel_mps2")),
            approach_decel_override_mps2=resolve_curve_approach_decel_override(p("curve_approach_decel_mps2")),
        ) if bool(p("curve_speed_enabled")) else None
        self.speed_zones = self.zone_policy.speed_zones if self.zone_policy else ()

        zone_policy = self.zone_policy.obstacle_zone if self.zone_policy else None
        self.obstacle_mode_enabled = self.mission_mode in {"all", "avoidance"}
        self.obstacle_zone = (
            ObstacleZone(zone_policy, self.route_geometry)
            if zone_policy is not None and self.obstacle_mode_enabled else None
        )
        planner = ObstacleSequencePlanner(
            vehicle_width_m=float(p("vehicle_width_m")),
            front_overhang_m=self.front_overhang_m,
            clearance_m=float(p("obstacle_clearance_m")),
            max_steering_rate_deg_s=float(p("obstacle_max_steering_rate_deg_s")),
            steering_gradient_window_m=float(p("obstacle_steering_gradient_window_m")),
            transition_length_m=float(p("obstacle_transition_length_m")),
            return_length_m=float(p("obstacle_return_length_m")),
            row_memory_sec=float(p("obstacle_row_memory_sec")),
            row_track_match_m=float(p("obstacle_row_track_match_m")),
            row_history_size=int(p("obstacle_row_history_size")),
            route_half_width_m=float(p("obstacle_route_half_width_m")),
            row_pair_nominal_gap_m=float(p("obstacle_row_pair_nominal_gap_m")),
            centerline_departure_m=float(p("obstacle_centerline_departure_m")),
            right_passage_margin_m=float(p("obstacle_right_passage_margin_m")),
        )
        self.obstacle = ObstacleBehavior(
            planner,
            zone=self.obstacle_zone,
            # Zone speed and pre-slow come with the zone polygon from the course policy (D10).
            zone_speed_mps=zone_policy.speed_cap_mps if self.obstacle_zone is not None else 3.0,
            pre_slow_decel_mps2=zone_policy.pre_slow_decel_mps2 if self.obstacle_zone is not None else 0.5,
            bypass_speed_mps=require_finite_parameter(
                "obstacle_bypass_speed_mps", p("obstacle_bypass_speed_mps"), minimum=0.1),
            fallback_speed_mps=float(p("obstacle_fallback_speed_mps")),
            forced_speed_mps=(float(p("obstacle_forced_speed_mps"))
                              if float(p("obstacle_forced_speed_mps")) > 0.0 else None),
            slow_clearance_m=float(p("obstacle_slow_clearance_m")),
            curve_speed_window_sec=float(p("obstacle_curve_speed_window_sec")),
            lateral_accel_mps2=float(p("obstacle_lateral_accel_mps2")),
            front_overhang_m=self.front_overhang_m,
            route_length_m=self.route_geometry.total_length,
            release_past_row_m=require_finite_parameter(
                "obstacle_release_past_row_m", p("obstacle_release_past_row_m"), minimum=0.0),
            release_offset_m=require_finite_parameter(
                "obstacle_release_offset_m", p("obstacle_release_offset_m"), minimum=0.0),
        )
        self.world = ObstacleWorld(
            self.route_geometry, self.obstacle_zone,
            association_m=float(p("obstacle_association_m")),
            rear_overhang_m=max(0.0, 2.60 - self.front_overhang_m),
            confirm_frames=int(p("obstacle_confirm_frames")),
            unconfirmed_memory_sec=require_finite_parameter(
                "obstacle_unconfirmed_memory_sec", p("obstacle_unconfirmed_memory_sec"), minimum=0.0),
            max_bottom_height_m=require_finite_parameter(
                "obstacle_max_bottom_height_m", p("obstacle_max_bottom_height_m"), minimum=0.0),
            min_top_height_m=require_finite_parameter(
                "obstacle_min_top_height_m", p("obstacle_min_top_height_m")),
        )
        self.odometry = ObstacleObservationBuffer()
        if self.obstacle_zone is not None and not self.obstacle_zone.fixed_lane:
            # D18: a bypass that may start at the zone end must be back on the route before the
            # finish brakes. The brake point is the finish machine's own, at the zone speed cap.
            # A fixed-lane test zone runs to the route end; there the bypass planner only plans
            # rows whose full return fits before it (ObstacleBehavior complete_return).
            finish = StopPointMachine(self.tracker.events[-1], **self.tracker.machine_options)
            finish_brake_s = finish.brake_point_s(zone_policy.speed_cap_mps)
            problem = obstacle_zone_finish_check(
                self.obstacle_zone.exit_s, float(p("obstacle_return_length_m")), finish_brake_s)
            if problem:
                self.get_logger().error(f"refusing to start: {problem}")
                raise ValueError(problem)
            self.get_logger().info(
                f"obstacle zone route s {self.obstacle_zone.entry_s:.1f}-{self.obstacle_zone.exit_s:.1f} m, "
                f"finish brake point {finish_brake_s:.1f} m"
            )
        self.aeb = AEBLogic(
            enabled=self.obstacle_mode_enabled and str(p("decision_policy")) == "brake_inspection",
            stale_objects_sec=float(p("aeb_stale_objects_sec")),
            obstacle_path_half_width_m=float(p("aeb_obstacle_path_half_width_m")),
            warning_ttc_sec=float(p("aeb_warning_ttc_sec")),
            brake_ttc_sec=float(p("aeb_brake_ttc_sec")),
            warning_distance_m=float(p("aeb_warning_distance_m")),
            emergency_distance_m=float(p("aeb_emergency_distance_m")),
            bypass_brake_ttc_sec=float(p("aeb_bypass_brake_ttc_sec")),
            bypass_emergency_distance_m=float(p("aeb_bypass_emergency_distance_m")),
            release_clear_ticks=int(p("aeb_release_clear_ticks")),
        )

        self.latest_route_context: RouteContext | None = None
        self.latest_vehicle_state: VehicleState | None = None
        self.latest_objects: PerceptionObjectArray | None = None
        self.latest_lights: TrafficLightObservationArray | None = None
        self.latest_phases: TrafficSignalPhaseArray | None = None
        self.schedule_change_clock = ScheduleChangeClock(self.phase_stale_sec)
        self.previous_path: Centerline | None = None
        self.manual_override_active = False
        self.last_need_stop = False
        self.phase_block_reported = False

        publisher = self.create_publisher
        self.target_path_pub = publisher(Centerline, str(p("target_path_topic")), 10)
        self.target_speed_pub = publisher(TargetSpeed, str(p("target_speed_topic")), 10)
        self.behavior_decision_pub = publisher(BehaviorDecision, str(p("behavior_decision_topic")), 10)
        self.debug_active_pub = publisher(String, str(p("debug_active_behavior_topic")), 10)
        self.debug_fsm_pub = publisher(String, str(p("debug_fsm_state_topic")), 10)
        self.debug_latency_pub = publisher(Float32, str(p("debug_planning_latency_topic")), 10)
        self.debug_route_age_pub = publisher(Float32, str(p("debug_route_context_age_topic")), 10)
        self.debug_vehicle_age_pub = publisher(Float32, str(p("debug_vehicle_state_age_topic")), 10)
        self.signal_diagnostics_pub = publisher(DiagnosticArray, str(p("signal_diagnostics_topic")), 10)

        subscribe = self.create_subscription
        # Depth 1 (the newest only): an executor pass runs the overdue tick and then takes one
        # message per subscription, the oldest queued. With depth 10, ticks longer than the 50 ms
        # input period fill the queue and every tick reads inputs up to 0.5 s old, which the stale
        # checks then report (10/17 14:57 A': "route context stale" x55 with no gap in the topic).
        # Best effort: a RELIABLE depth-1 reader could lock into receiving every sample 0.3-0.8 s
        # late for the life of the process while new readers of the same writer got it in 1 ms
        # (10/23-24 route context, 10/25 09:36 and 10:27 vehicle state: 0.4 s sawtooth, one update
        # per ~13 ticks). Only the newest sample matters here, so a lost one is just skipped.
        latest_only = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        subscribe(RouteContext, str(p("route_context_topic")), self.on_route_context, latest_only)
        subscribe(VehicleState, str(p("vehicle_state_topic")), self.on_vehicle_state, latest_only)
        subscribe(PerceptionObjectArray, str(p("objects_topic")), self.on_objects, 10)
        subscribe(TrafficLightObservationArray, str(p("traffic_lights_topic")), self.on_lights, OBSERVATION_QOS)
        subscribe(TrafficSignalPhaseArray, str(p("signal_phases_topic")), self.on_phases, 10)
        subscribe(Odometry, str(p("obstacle_odometry_topic")), self.odometry.add_odometry, qos_profile_sensor_data)
        if self.require_readiness:
            subscribe(Bool, str(p("readiness_topic")), self.on_readiness, 10)
        self.create_timer(1.0 / max(1.0, float(p("planning_rate_hz"))), self.on_timer)

    def param(self, name: str):
        return self.get_parameter(name).value

    def on_route_context(self, msg: RouteContext) -> None:
        self.latest_route_context = msg

    def on_vehicle_state(self, msg: VehicleState) -> None:
        self.latest_vehicle_state = msg

    def on_objects(self, msg: PerceptionObjectArray) -> None:
        self.latest_objects = msg

    def on_lights(self, msg: TrafficLightObservationArray) -> None:
        self.latest_lights = msg

    def on_phases(self, msg: TrafficSignalPhaseArray) -> None:
        self.latest_phases = msg
        stamp = stamp_seconds(msg.header.stamp)
        for phase in msg.phases:
            self.schedule_change_clock.observe(
                stamp, str(phase.light_id), bool(phase.aligned),
                (bool(phase.permitted_now), int(phase.predicted_lamps)),
            )

    def on_readiness(self, msg: Bool) -> None:
        ready = bool(msg.data)
        if self.route_ready_seen and self.route_ready and not ready:
            self.get_logger().warn("route readiness lost while driving; driving continues")
        elif self.route_ready_seen and not self.route_ready and ready:
            self.get_logger().info("route readiness restored")
        self.route_ready = ready
        self.route_ready_seen = self.route_ready_seen or ready

    # World model ----------------------------------------------------------------------------

    @staticmethod
    def fresh(msg, limit_sec: float, now) -> bool:
        return msg is not None and 0.0 <= stamp_age_sec(now, msg.header.stamp) <= limit_sec

    def own_light(self, event, now) -> OwnLight:
        """Perception's reading of the event's own lights; UNKNOWN when none is fresh."""
        lights = self.latest_lights
        if not self.fresh(lights, self.light_stale_sec, now):
            return OwnLight()
        seen = lights_by_map_id(lights.lights, event.light_ids)
        if not seen:
            return OwnLight()
        light = max(seen.values(), key=lambda item: float(item.confidence))
        return OwnLight(
            LIGHT_STATES.get(int(light.state), UNKNOWN),
            bool(int(light.lamps) & TrafficLightObservation.LAMP_ARROW),
        )

    def own_phases(self, event, now) -> list:
        """Fresh signal phase tracker estimates of the event's own lights, in message order."""
        phases = self.latest_phases
        if not self.fresh(phases, self.phase_stale_sec, now):
            return []
        return [
            phase for phase in phases.phases
            if phase.on_route and phase.light_id in event.light_ids and phase.route_event_id == event.event_id
        ]

    @staticmethod
    def phase_hint(own_phases) -> PhaseHint | None:
        if not own_phases:
            return None
        return PhaseHint(bool(own_phases[0].aligned), bool(own_phases[0].permitted_now))

    def own_schedule(self, own_phases) -> ScheduleState | None:
        """The first aligned estimate of an own light as a schedule, else None, with the time since
        its last change from the estimate stream (signal_schedule.ScheduleChangeClock)."""
        aligned = [phase for phase in own_phases if bool(phase.aligned)]
        if not aligned:
            return None
        phase = aligned[0]
        until = min(float(phase.phase_remaining_sec), float(phase.permission_change_sec))
        key = (bool(phase.permitted_now), int(phase.predicted_lamps))
        clock = getattr(self, "schedule_change_clock", None)
        elapsed = (
            clock.elapsed(str(phase.light_id), key, stamp_seconds(self.latest_phases.header.stamp))
            if clock is not None and self.latest_phases is not None
            else math.nan
        )
        return ScheduleState(
            key[0], key[1], until,
            float(phase.permission_probability), float(phase.permission_change_sec),
            elapsed_sec=elapsed,
        )

    def own_light_stamp(self, now) -> float | None:
        """Stamp of the latest fresh light observation, to count camera frames once."""
        lights = self.latest_lights
        if not self.fresh(lights, self.light_stale_sec, now):
            return None
        return stamp_seconds(lights.header.stamp)

    def signal_permission(self, now_sec, now, event, phase, s_rear, speed):
        """Event permission of a signal event on the input signal_schedule selects.

        An aligned, fresh phase estimate of the own light replaces the camera reading; the camera
        only rejects it (sure opposite, distrust) or decides near a scheduled change, where a
        schedule that does not permit withholds a launch from a held stop. The confirmation rules
        stay as they are. Stopped at the event, a scheduled permission launches only when sure, and
        the schedule holds back the camera's permission for a limited time (signal_schedule). With
        the camera deciding, stopped past the line with the own light UNKNOWN, the estimate's
        permission replaces UNKNOWN as before (not when distrusted, past the hold limit or turned
        off for the light).
        """
        light = self.own_light(event, now)
        own_phases = self.own_phases(event, now)
        schedule = self.own_schedule(own_phases)
        stopped = speed is not None and math.isfinite(speed) and abs(speed) <= self.permission.stopped_speed_mps
        selection = self.signal_selector.update(
            now_sec, event.event_id, str(event.arrow or "NONE").upper(),
            LIGHT_CODES.get(light.state, TRAFFIC_UNKNOWN), light.arrow_active,
            schedule, self.own_light_stamp(now),
            stopped_at_line=stopped and phase in STOPPED_PHASES,
        )
        if selection.started:
            self.get_logger().warn(f"signal input: event={event.event_id or '-'} {selection.reason}")
        if selection.source == SOURCE_SCHEDULE:
            light = OwnLight(CODE_LIGHTS.get(int(selection.state), UNKNOWN), bool(selection.arrow_active))
        dropped = {SOURCE_DISTRUST, SOURCE_LIMIT, SOURCE_CAMERA}
        hint = self.phase_hint(own_phases) if selection.source not in dropped else None
        verdict_before = self.permission.verdict
        permission = self.permission.update(
            now_sec, event, phase, s_rear=s_rear, speed_mps=speed,
            light=light, hint=hint, phase_forbids=selection.schedule_forbids,
            arrival=self.arrival(event, s_rear, speed, schedule, selection.source, now),
        )
        if self.permission.verdict != verdict_before:
            self.get_logger().info(
                f"signal event={event.event_id or '-'} {self.permission.verdict_reason}; {selection.reason}"
            )
        reason = f"{permission.reason}; {selection.reason}" if permission.reason else permission.reason
        return replace(permission, reason=reason, tags=(*permission.tags, selection.tag))

    def arrival(self, event, s_rear, speed, schedule, source, now) -> Arrival | None:
        """The approach for the arrival verdict (event_permission). The permission end counts only
        while the schedule is trusted (it decides, or the camera decides near its change or at an
        unsure launch) and permits; it is aged from the estimate's stamp to now."""
        machine = self.tracker.machine
        if machine is None or machine.event is not event or speed is None:
            return None
        left = math.nan
        trusted = source in {SOURCE_SCHEDULE, SOURCE_MARGIN, SOURCE_UNSURE}
        if trusted and schedule is not None and schedule.permitted and self.latest_phases is not None:
            age = stamp_age_sec(now, self.latest_phases.header.stamp)
            left = float(schedule.permission_change_sec) - age
        rear_gap = event.line_s - s_rear
        return Arrival(
            front_gap_m=rear_gap - self.front_overhang_m,
            speed_mps=speed,
            decision_point_reached=machine.brake_point_reached(rear_gap, speed) or rear_gap <= event.commit_m,
            permission_left_sec=left,
        )

    def input_problems(self, now) -> list[str]:
        """Route context and projection, vehicle state and the start readiness; never perception."""
        route, vehicle = self.latest_route_context, self.latest_vehicle_state
        reasons = []
        if not self.route_ready and not self.route_ready_seen:
            reasons.append("ROUTE_NOT_READY")
        if route is None:
            reasons.append("ROUTE_CONTEXT_MISSING")
        elif not self.fresh(route, self.route_stale_limit_sec(route), now):
            reasons.append("ROUTE_CONTEXT_STALE")
        elif (
            not route.route_projection_valid
            or not math.isfinite(float(route.progress_s))
            or len(route.local_path_points) < 2
        ):
            reasons.append("ROUTE_PROJECTION_INVALID")
        if vehicle is None:
            reasons.append("VEHICLE_STATE_MISSING")
        elif not self.fresh(vehicle, self.vehicle_stale_sec, now):
            reasons.append("VEHICLE_STATE_STALE")
        elif not math.isfinite(float(vehicle.speed_mps)):
            reasons.append("VEHICLE_SPEED_INVALID")
        return reasons

    def route_stale_limit_sec(self, route) -> float:
        """Route context age limit: obstacle_route_stale_sec in an obstacle stage of a course with
        an obstacle zone, route_stale_sec elsewhere."""
        if self.obstacle_zone is None or self.policy_runtime is None:
            return self.route_stale_sec
        stage = self.policy_runtime.stage_for(route.current_zone)
        if stage is None or not stage.obstacle_enabled:
            return self.route_stale_sec
        return self.obstacle_route_stale_sec

    def route_age_tags(self, now) -> list[str]:
        """ROUTE_CONTEXT_AGE_TOLERATED while a route older than route_stale_sec is still used."""
        route = self.latest_route_context
        if route is None or self.fresh(route, self.route_stale_sec, now):
            return []
        if not self.fresh(route, self.route_stale_limit_sec(route), now):
            return []
        self.get_logger().warn(
            f"route context {stamp_age_sec(now, route.header.stamp):.2f} s old in the obstacle stage; "
            "the bypass continues",
            throttle_duration_sec=1.0,
        )
        return ["ROUTE_CONTEXT_AGE_TOLERATED"]

    def finish_brake_holds(self, problems) -> bool:
        """The finish brake is latched (D11) and the only problem is the route projection ending.

        The finish brakes so the front bumper stops up to finish_overrun_m past the line at the
        route end, so the rear axle runs off the route while stopping. The stop needs no route:
        the machine keeps its brake and turns HELD on the measured speed (StopPointMachine
        hold_output), so this is the finish, not an input loss (2026-10-20 Gazebo: RECOVERY
        INPUT_LOSS_BRAKE for 2.5-2.7 s on every finish). Any other problem stays an input loss.
        """
        current = self.tracker.current
        return (
            bool(problems)
            and set(problems) <= ROUTE_END_PROBLEMS
            and current is not None
            and current.kind == KIND_FINISH
            and self.tracker.phase in STOPPED_PHASES
        )

    def observe_objects(self, route: RouteContext) -> None:
        objects = self.latest_objects
        if objects is None:
            return
        frame = objects.header.frame_id.lstrip("/")
        stamp = stamp_seconds(objects.header.stamp)
        pose = None
        if frame == "map":
            pose = Pose2D(0.0, 0.0, 0.0)
        elif self.odometry.frames == ("map", frame):
            sample = self.odometry.pose_at(stamp)
            if sample is not None:
                pose = Pose2D(sample.x, sample.y, sample.yaw)
        if pose is None and frame == route.header.frame_id.lstrip("/"):
            pose = route_pose(self.route_geometry, route)
        self.world.observe(objects, pose, stamp)

    # Tick -----------------------------------------------------------------------------------

    def on_timer(self) -> None:
        start = time.perf_counter()
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1.0e-9
        vehicle = self.latest_vehicle_state
        if vehicle is not None and vehicle.mode == VehicleState.MODE_MANUAL:
            self.publish_manual(now, vehicle)
            return
        if self.manual_override_active:
            self.reset_transient_state()
            self.manual_override_active = False

        problems = self.input_problems(now)
        inputs_ok = not problems
        finish_overrun = self.finish_brake_holds(problems)
        speed = abs(float(vehicle.speed_mps)) if vehicle is not None else None
        route = self.latest_route_context
        outputs = []

        loss = self.input_latch.update(
            now_sec, inputs_ok or finish_overrun, speed, brake_engaged=self.last_need_stop)
        input_out = ModuleOutput("input_state", tags=list(problems) + (self.route_age_tags(now) if inputs_ok else []))
        estop = vehicle is not None and bool(vehicle.estop_active)
        if estop:
            input_out.stop_distance_m = 0.0
            input_out.tags.append("ESTOP")
        if loss.stale:
            input_out.cap_mps = 0.0
            input_out.tags.append("INPUT_STALE_SPEED_ZERO")
        if loss.brake:
            input_out.stop_distance_m = 0.0
            input_out.tags.append("INPUT_LOSS_BRAKE")
        outputs.append(input_out)

        if inputs_ok:
            s_rear = float(route.progress_s)

            def permission_for(event, phase):
                if event.kind == KIND_SIGNAL:
                    return self.signal_permission(now_sec, now, event, phase, s_rear, speed)
                return self.permission.update(now_sec, event, phase, s_rear=s_rear, speed_mps=speed)

            stop_point = self.tracker.update(s_rear, speed, permission_for)
        else:
            vehicle_ok = not any(problem.startswith("VEHICLE_") for problem in problems)
            stop_point = self.tracker.hold_output(
                speed if vehicle_ok else None,
                "finish brake held past the route end" if finish_overrun else "inputs stale; phase kept",
            )
        stop_out = ModuleOutput("stop_point")
        if stop_point is not None:
            stop_out.cap_mps = stop_point.cap_mps
            stop_out.stop_distance_m = 0.0 if stop_point.stop else None
            stop_out.tags.extend(stop_point.tags)
            if "SIGNAL_UNKNOWN_TIMEOUT" in stop_point.tags:
                self.report_signal_unknown_timeout(stop_point.event)
        blocked = stop_point is not None and "SIGNAL_PHASE_TRACKER_BLOCK" in stop_point.tags
        if blocked and not self.phase_block_reported:
            self.report_signal_phase_block(stop_point.event)
        self.phase_block_reported = blocked
        outputs.append(stop_out)

        cruise, course_stage = self.course_speed(route, inputs_ok)
        follow = self.route_follow(route, inputs_ok, now, cruise)
        if course_stage:
            follow.tags.append(f"STAGE_COURSE_SPEED:{course_stage}:{cruise:g}")
        outputs.append(follow)
        cap, tags = curve_speed_cap(route, self.curve_policy, cruise, route_is_fresh=inputs_ok)
        outputs.append(ModuleOutput("curve_speed", cap_mps=cap, tags=tags))
        cap, tags = speed_zone_cap(
            route, self.speed_zones, cruise, self.front_overhang_m, route_is_fresh=inputs_ok)
        outputs.append(ModuleOutput("speed_zone", cap_mps=cap, tags=tags))
        outputs.append(self.obstacle_output(route, vehicle, inputs_ok, now))
        outputs.append(self.aeb_warning(now, vehicle, route))

        decision = arbitrate(outputs)
        self.last_need_stop = decision.need_stop
        behavior, fsm_state, reason = self.label(
            estop, loss, stop_point, decision.binding_cap_owner, cruise if course_stage else None)
        tags = list(decision.tags)
        if self.standstill_brake_release(now_sec, speed, stop_point, decision, estop or loss.active):
            tags.append(STANDSTILL_BRAKE_RELEASE)
        if stop_point is not None and stop_point.phase not in {FREE, CLEARED}:
            tags.append(f"EVENT:{stop_point.event.event_id}")
            if stop_point.event.brake_wp is not None:
                tags.append(f"BRAKE_TRIGGER_WP:{stop_point.event.brake_wp}")
        if self.route_ready_seen and not self.route_ready:
            tags.append("ROUTE_READINESS_WARNING")
        latency_ms = (time.perf_counter() - start) * 1000.0
        if latency_ms > self.planning_deadline_ms:
            tags.append("PLANNING_DEADLINE_EXCEEDED")
        self.publish(now, route, decision, behavior, fsm_state, reason, tags, latency_ms)

    def course_speed(self, route: RouteContext | None, route_ok: bool) -> tuple[float, str]:
        """Cruise speed of this tick and the stage that sets it: the stage's course_speed_mps
        (final F_STAGE_6, the signal-free run through the obstacle zone to the finish) in place of
        base_speed_mps, else base_speed_mps and an empty stage."""
        runtime = getattr(self, "policy_runtime", None)
        if route_ok and runtime is not None:
            stage = runtime.stage_for(route.current_zone)
            if stage is not None and stage.course_speed_mps is not None:
                return float(stage.course_speed_mps), stage.stage_id
        return self.base_speed_mps, ""

    def standstill_brake_release(self, now_sec, speed, stop_point, decision, override: bool) -> bool:
        """Lets motion control release the brake servo at a planned stop (signal, timed stop,
        finish HELD) once the car has stood still for standstill_brake_release_sec; any motion
        above the stopped speed restarts the count and the brake comes back. Only when the stop
        point is the one stop request: never under E-stop or the input-loss rule."""
        delay = getattr(self, "standstill_brake_release_sec", 0.0)
        held = (
            delay > 0.0
            and not override
            and stop_point is not None
            and stop_point.phase == HELD
            and decision.stop_owners == ("stop_point",)
            and speed is not None
            and speed <= self.permission.stopped_speed_mps
        )
        if not held:
            self.standstill_since_sec = None
            return False
        if self.standstill_since_sec is None:
            self.standstill_since_sec = now_sec
        return now_sec - self.standstill_since_sec >= delay

    def route_follow(
        self, route: RouteContext | None, route_ok: bool, now, cruise_mps: float | None = None,
    ) -> ModuleOutput:
        output = ModuleOutput("route_follow", cap_mps=self.base_speed_mps if cruise_mps is None else cruise_mps)
        if route_ok:
            path = Centerline()
            path.detection_id = 1
            path.points = list(route.local_path_points)
            path.confidence = float(route.projection_confidence)
            path.source = "global_route_local_path"
            output.path, output.path_request = path, "ROUTE_LOCAL_PATH"
        elif self.previous_path is not None:
            # The path is vehicle-relative: a fresh stamp on unchanged points drags it along with
            # the car. Rebase it from its own stamp's pose to now; without both poses it keeps its
            # old header so the target-age checks downstream still see how old it is.
            # previous_path stays the anchor, so repeated holds never accumulate the transform.
            target = Header(stamp=now.to_msg(), frame_id=self.previous_path.header.frame_id)
            path = self.odometry.align_path(self.previous_path, target) or deepcopy(self.previous_path)
            path.confidence = min(0.5, float(path.confidence))
            path.source = "previous_safe_path"
            output.path, output.path_request = path, "HOLD_PREVIOUS_PATH"
        return output

    def obstacle_output(self, route, vehicle, inputs_ok: bool, now) -> ModuleOutput:
        if not inputs_ok or not self.obstacle_mode_enabled:
            return ModuleOutput("obstacle")
        progress = float(route.progress_s)
        stage = self.policy_runtime.stage_for(route.current_zone) if self.policy_runtime else None
        stage_enabled = self.policy_runtime is None or (stage is not None and stage.obstacle_enabled)
        if stage_enabled:
            # Only in the obstacle stage: the K-City final start drives the zone road's other lane.
            self.observe_objects(route)
        self.world.forget_passed(progress)
        pose = route_pose(self.route_geometry, route)
        perception_ok = pose is not None and self.fresh(self.latest_objects, self.objects_stale_sec, now)
        lane_lines = (
            self.obstacle_zone.lane_lines(progress, ahead_m=60.0) if self.obstacle_zone is not None else None
        )
        steering = float(vehicle.steering_rad)
        return self.obstacle.update(
            route,
            mode_enabled=True,
            stage_enabled=stage_enabled,
            objects=self.world.local_objects(pose, route.header) if pose is not None else None,
            perception_ok=perception_ok,
            lane_lines=lane_lines,
            steering_rad=steering if math.isfinite(steering) else None,
            now_sec=stamp_seconds(route.header.stamp),
        )

    def obstacle_active(self, binding_cap_owner: str = "") -> bool:
        """Inside the zone, bypassing, or slowing down for the zone right now."""
        state = self.obstacle.state
        if state == PRE_SLOW:
            return binding_cap_owner == "obstacle"
        return state != OBSTACLE_IDLE

    def aeb_warning(self, now, vehicle, route) -> ModuleOutput:
        """Decision's TTC check only warns; it never brakes or stops."""
        output = ModuleOutput("aeb")
        if self.obstacle.bypass_active or self.obstacle.state not in {OBSTACLE_IDLE, PRE_SLOW}:
            self.aeb.suppress("SUPPRESSED_OBSTACLE_ZONE")
        elif self.aeb.brake_required(now, self.latest_objects, vehicle, bypass_active=False):
            output.tags.append("AEB_BRAKE_WARNING")
        if self.aeb.state in {"WARNING", "BRAKE", "HOLD"}:
            output.tags.append("AEB_WARNING")
        return output

    def label(
        self, estop: bool, loss, stop_point, binding_cap_owner: str = "", course_speed_mps: float | None = None,
    ) -> tuple[str, str, str]:
        if estop:
            return "EMERGENCY", "ESTOP", "estop active"
        if loss.active:
            # Brake after the delay until stopped / the brake engaged at the loss / speed 0 only.
            state = ("INPUT_LOSS_BRAKE" if loss.latched
                     else "INPUT_LOSS_HOLD_BRAKE" if loss.brake else "INPUT_LOSS_COAST")
            return "RECOVERY", state, "input stale"
        if stop_point is not None and stop_point.phase not in {FREE, CLEARED}:
            return stop_point.event.behavior, stop_point.state, stop_point.reason
        if self.obstacle_active(binding_cap_owner):
            return "OBSTACLE", self.obstacle.state, "obstacle behavior"
        if course_speed_mps is not None:
            return "GLOBAL_ROUTE_FOLLOW", "FINAL_SPRINT", f"signal-free stage at {course_speed_mps:g} m/s"
        return "GLOBAL_ROUTE_FOLLOW", "ROUTE_FOLLOW", "following global route local path"

    def publish(self, now, route, decision, behavior, fsm_state, reason, tags, latency_ms) -> None:
        frame = route.header.frame_id if route is not None and route.header.frame_id else "base_footprint"
        stamp = now.to_msg()
        path = decision.path if decision.path is not None else Centerline()
        if decision.path is None or decision.path_request != "HOLD_PREVIOUS_PATH":
            path.header.stamp = stamp
            path.header.frame_id = frame
            if decision.path is not None:
                self.previous_path = deepcopy(path)

        target = TargetSpeed()
        target.header.stamp = stamp
        target.header.frame_id = frame
        target.speed_limit_mps = float(decision.speed_limit_mps)
        target.target_speed_mps = float(decision.target_speed_mps)
        target.stop_target_distance = INF_DISTANCE
        if decision.need_stop:
            stop_command(target, decision.stop_distance_m)
        target.source_behavior = behavior
        target.constraints = list(dict.fromkeys(tags))

        message = BehaviorDecision()
        message.header.stamp = stamp
        message.header.frame_id = frame
        message.safety_state = {"EMERGENCY": "EMERGENCY", "RECOVERY": "DEGRADED"}.get(behavior, "NORMAL")
        message.active_behavior = behavior
        message.fsm_state = fsm_state
        message.selected_reason = reason
        message.need_stop = bool(target.need_stop)
        message.target_speed_limit_mps = float(target.speed_limit_mps)
        message.path_request = decision.path_request or "NO_PATH"
        message.stop_target_distance = float(target.stop_target_distance)
        message.constraints = list(target.constraints)
        message.risk_level = "HIGH" if target.need_stop or behavior in {"EMERGENCY", "RECOVERY"} else "LOW"

        self.behavior_decision_pub.publish(message)
        self.target_path_pub.publish(path)
        self.target_speed_pub.publish(target)
        self.debug_active_pub.publish(String(data=f"{behavior}: {reason}"))
        self.debug_fsm_pub.publish(String(data=fsm_state))
        self.debug_latency_pub.publish(Float32(data=float(latency_ms)))
        route_age = self.latest_route_context
        self.debug_route_age_pub.publish(Float32(
            data=-1.0 if route_age is None else float(stamp_age_sec(now, route_age.header.stamp))))
        vehicle = self.latest_vehicle_state
        self.debug_vehicle_age_pub.publish(Float32(
            data=-1.0 if vehicle is None else float(stamp_age_sec(now, vehicle.header.stamp))))

    def publish_manual(self, now, vehicle: VehicleState) -> None:
        if not self.manual_override_active:
            self.reset_transient_state()
        self.manual_override_active = True
        route = self.latest_route_context
        route_ok = route is not None and route.route_projection_valid and len(route.local_path_points) >= 2
        outputs = [
            self.route_follow(route, route_ok, now),
            ModuleOutput("input_state", cap_mps=abs(float(vehicle.speed_mps)),
                         tags=["MANUAL_MODE", "AUTONOMY_SUSPENDED"]),
        ]
        decision = arbitrate(outputs)
        self.publish(now, route, decision, "MANUAL_OVERRIDE", "MANUAL",
                     "physical manual mode selected", list(decision.tags), 0.0)

    def reset_transient_state(self) -> None:
        """MANUAL entry and exit: the current event restarts from FREE; cleared events stay cleared."""
        self.tracker.reset_current()
        self.permission.reset()
        self.signal_selector.reset()
        self.input_latch.reset()
        self.obstacle.reset()
        self.world.reset()
        self.last_need_stop = False
        self.phase_block_reported = False
        self.standstill_since_sec = None

    def report_signal_unknown_timeout(self, event) -> None:
        waited_sec = self.permission.unknown_waited_sec or 0.0
        message = (
            f"signal UNKNOWN wait limit reached: event={event.event_id or '-'} "
            f"stop_line={event.stop_line_id or '-'} waited={waited_sec:.1f}s; departing"
        )
        self.get_logger().warn(message)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = "kaiev26_decision/signal_unknown_wait_limit"
        status.hardware_id = "decision"
        status.message = message
        status.values = [
            KeyValue(key="event_id", value=event.event_id),
            KeyValue(key="stop_line_id", value=event.stop_line_id),
            KeyValue(key="waited_sec", value=f"{waited_sec:.1f}"),
        ]
        diagnostics.status = [status]
        self.signal_diagnostics_pub.publish(diagnostics)

    def report_signal_phase_block(self, event) -> None:
        """Once when the block starts: the own light reads go, the schedule forbids the launch."""
        message = (
            f"signal launch withheld: event={event.event_id or '-'} "
            f"stop_line={event.stop_line_id or '-'} own light reads go, but the schedule does not "
            "permit near a change or its permission ends too soon"
        )
        self.get_logger().warn(message)
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.WARN
        status.name = "kaiev26_decision/signal_phase_tracker_block"
        status.hardware_id = "decision"
        status.message = message
        status.values = [
            KeyValue(key="event_id", value=event.event_id),
            KeyValue(key="stop_line_id", value=event.stop_line_id),
        ]
        diagnostics.status = [status]
        self.signal_diagnostics_pub.publish(diagnostics)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MainPlanningEngineNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
