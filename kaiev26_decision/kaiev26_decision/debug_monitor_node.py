#!/usr/bin/env python3
from __future__ import annotations

import math

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import ActuatorCommand, BehaviorDecision, Centerline, RouteContext, SceneSummary, TargetSpeed, VehicleState
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from kaiev26_decision.common import (
    INF_DISTANCE,
    require_finite_parameter,
    spin_until_shutdown,
    traffic_state_name,
)


def set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


def kinematic_trajectory_points(
    speed_mps: float,
    steering_rad: float,
    wheelbase_m: float,
    horizon_sec: float,
    step_sec: float,
) -> list[Point]:
    curvature = math.tan(steering_rad) / wheelbase_m
    points: list[Point] = []
    sample_count = int(math.floor(horizon_sec / step_sec)) + 1
    for index in range(sample_count):
        time_sec = index * step_sec
        distance_m = speed_mps * time_sec
        if abs(curvature) <= 1.0e-9:
            x = distance_m
            y = 0.0
        else:
            heading = distance_m * curvature
            x = math.sin(heading) / curvature
            y = (1.0 - math.cos(heading)) / curvature
        points.append(Point(x=float(x), y=float(y), z=0.10))
    return points


class DebugMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_debug_monitor")
        self.declare_parameter("target_path_topic", "/planning/target_path")
        self.declare_parameter("target_speed_topic", "/planning/target_speed")
        self.declare_parameter("behavior_decision_topic", "/planning/behavior_decision")
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("scene_summary_topic", "/planning/scene_summary")
        self.declare_parameter("command_topic", "/planning/command")
        self.declare_parameter("vehicle_state_topic", "/vehicle/state")
        self.declare_parameter("marker_topic", "/debug/autonomous_markers")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("wheelbase_m", 1.2991017929)
        self.declare_parameter("trajectory_horizon_sec", 3.0)
        self.declare_parameter("trajectory_step_sec", 0.1)
        self.declare_parameter("trajectory_stationary_speed_mps", 0.05)

        self.target_path: Centerline | None = None
        self.target_speed: TargetSpeed | None = None
        self.behavior: BehaviorDecision | None = None
        self.route: RouteContext | None = None
        self.scene: SceneSummary | None = None
        self.command: ActuatorCommand | None = None
        self.vehicle: VehicleState | None = None
        self.wheelbase_m = require_finite_parameter(
            "wheelbase_m", self.get_parameter("wheelbase_m").value, minimum=0.1
        )
        self.trajectory_horizon_sec = require_finite_parameter(
            "trajectory_horizon_sec",
            self.get_parameter("trajectory_horizon_sec").value,
            minimum=0.1,
        )
        self.trajectory_step_sec = require_finite_parameter(
            "trajectory_step_sec",
            self.get_parameter("trajectory_step_sec").value,
            minimum=0.01,
        )
        self.trajectory_stationary_speed_mps = require_finite_parameter(
            "trajectory_stationary_speed_mps",
            self.get_parameter("trajectory_stationary_speed_mps").value,
            minimum=0.0,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("target_path_topic").value),
            self.on_target_path,
            10,
        )
        self.create_subscription(
            TargetSpeed,
            str(self.get_parameter("target_speed_topic").value),
            self.on_target_speed,
            10,
        )
        self.create_subscription(
            BehaviorDecision,
            str(self.get_parameter("behavior_decision_topic").value),
            self.on_behavior,
            10,
        )
        self.create_subscription(
            RouteContext,
            str(self.get_parameter("route_context_topic").value),
            self.on_route,
            10,
        )
        self.create_subscription(
            SceneSummary,
            str(self.get_parameter("scene_summary_topic").value),
            self.on_scene,
            10,
        )
        self.create_subscription(
            ActuatorCommand,
            str(self.get_parameter("command_topic").value),
            self.on_command,
            10,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle,
            10,
        )
        publish_rate_hz = require_finite_parameter(
            "publish_rate_hz",
            self.get_parameter("publish_rate_hz").value,
            minimum=1.0,
        )
        self.create_timer(1.0 / publish_rate_hz, self.publish_markers)

    def on_target_path(self, msg: Centerline) -> None:
        self.target_path = msg

    def on_target_speed(self, msg: TargetSpeed) -> None:
        self.target_speed = msg

    def on_behavior(self, msg: BehaviorDecision) -> None:
        self.behavior = msg

    def on_route(self, msg: RouteContext) -> None:
        self.route = msg

    def on_scene(self, msg: SceneSummary) -> None:
        self.scene = msg

    def on_command(self, msg: ActuatorCommand) -> None:
        self.command = msg

    def on_vehicle(self, msg: VehicleState) -> None:
        self.vehicle = msg

    def publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        status = self.make_status_text(now)
        markers.markers.append(status)

        target_path = self.target_path
        if target_path is not None and len(target_path.points) >= 2:
            markers.markers.append(self.make_path_marker(now, target_path))

        route = self.route
        if route is not None and len(route.local_path_points) >= 2:
            markers.markers.append(self.make_route_local_path_marker(now, route))

        scene = self.scene
        if scene is not None:
            if scene.stopline_distance < INF_DISTANCE:
                markers.markers.append(self.make_stopline_marker(now, scene))
            if scene.obstacle_on_path:
                markers.markers.append(self.make_obstacle_marker(now, scene))

        command = self.command
        if command is not None:
            target_speed = 0.0 if command.brake_engage else float(command.speed_target_mps)
            if abs(target_speed) >= self.trajectory_stationary_speed_mps:
                markers.markers.append(
                    self.make_trajectory_marker(
                        now,
                        "target_command_trajectory",
                        6,
                        target_speed,
                        float(command.steering_target_rad),
                        (1.0, 0.15, 0.88, 0.96),
                        0.12,
                    )
                )

        vehicle = self.vehicle
        if vehicle is not None and abs(float(vehicle.speed_mps)) >= self.trajectory_stationary_speed_mps:
            markers.markers.append(
                self.make_trajectory_marker(
                    now,
                    "actual_vehicle_trajectory",
                    7,
                    float(vehicle.speed_mps),
                    float(vehicle.steering_rad),
                    (0.25, 1.0, 0.25, 0.94),
                    0.09,
                )
            )
        if command is not None or vehicle is not None:
            markers.markers.append(self.make_steering_text(now))

        self.marker_pub.publish(markers)

    def make_trajectory_marker(
        self,
        now,
        namespace: str,
        marker_id: int,
        speed_mps: float,
        steering_rad: float,
        rgba: tuple[float, float, float, float],
        width_m: float,
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = "base_footprint"
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width_m
        marker.points = kinematic_trajectory_points(
            speed_mps,
            steering_rad,
            self.wheelbase_m,
            self.trajectory_horizon_sec,
            self.trajectory_step_sec,
        )
        set_color(marker, rgba)
        return marker

    def make_steering_text(self, now) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = "base_footprint"
        marker.ns = "steering_comparison"
        marker.id = 8
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 1.2
        marker.pose.position.y = -1.4
        marker.pose.position.z = 1.3
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.28
        target_speed = 0.0
        target_steer = 0.0
        if self.command is not None:
            target_speed = 0.0 if self.command.brake_engage else float(self.command.speed_target_mps)
            target_steer = float(self.command.steering_target_rad)
        actual_speed = float(self.vehicle.speed_mps) if self.vehicle is not None else 0.0
        actual_steer = float(self.vehicle.steering_rad) if self.vehicle is not None else 0.0
        marker.text = (
            f"TARGET  {target_speed:.2f} m/s  {math.degrees(target_steer):+.1f} deg\n"
            f"ACTUAL  {actual_speed:.2f} m/s  {math.degrees(actual_steer):+.1f} deg"
        )
        set_color(marker, (0.88, 0.94, 1.0, 0.98))
        return marker

    def make_status_text(self, now) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = "base_footprint"
        marker.ns = "autonomous_status"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 3.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.2
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35

        behavior = self.behavior.active_behavior if self.behavior is not None else "WAITING"
        fsm = self.behavior.fsm_state if self.behavior is not None else "-"
        reason = self.behavior.selected_reason if self.behavior is not None else "no decision yet"
        target_speed = self.target_speed.target_speed_mps if self.target_speed is not None else 0.0
        actual_speed = self.vehicle.speed_mps if self.vehicle is not None else 0.0
        scene_health = self.scene.perception_health if self.scene is not None else "WAITING"
        light = traffic_state_name(self.scene.traffic_light_state) if self.scene is not None else "UNKNOWN"
        brake_active = self.command.brake_engage if self.command is not None else False
        path_source = self.target_path.source if self.target_path is not None else "-"
        route_text = "Route: WAITING"
        if self.route is not None:
            route_text = (
                f"Route: s={self.route.progress_s:.1f}m "
                f"cte={self.route.cross_track_error:+.2f}m "
                f"zone={self.route.current_zone}\n"
                f"Maneuver: {self.route.next_maneuver} "
                f"stop={self.route.next_stop_line_id or '-'}"
            )

        marker.text = (
            f"Behavior: {behavior}\n"
            f"FSM: {fsm}\n"
            f"Reason: {reason}\n"
            f"Speed: {actual_speed:.2f} / {target_speed:.2f} m/s\n"
            f"Path: {path_source}\n"
            f"{route_text}\n"
            f"Scene: {scene_health}  Light: {light}\n"
            f"Brake: {'ENGAGED' if brake_active else 'RELEASED'}"
        )
        set_color(marker, self.status_color(behavior, brake_active, scene_health))
        return marker

    def make_path_marker(self, now, target_path: Centerline) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = target_path.header.frame_id or "base_footprint"
        marker.ns = "target_path"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.points = list(target_path.points)
        set_color(marker, (0.0, 0.85, 1.0, 0.95))
        return marker

    def make_route_local_path_marker(self, now, route: RouteContext) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = route.header.frame_id or "base_footprint"
        marker.ns = "route_local_path"
        marker.id = 5
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.07
        marker.points = list(route.local_path_points)
        set_color(marker, (0.65, 0.35, 1.0, 0.85))
        return marker

    def make_stopline_marker(self, now, scene: SceneSummary) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = scene.header.frame_id or "base_footprint"
        marker.ns = "stopline"
        marker.id = 3
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(scene.stopline_distance)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.15
        marker.scale.y = 4.0
        marker.scale.z = 0.1
        color = (1.0, 0.1, 0.1, 0.75) if scene.stopline_distance <= 10.0 else (1.0, 0.85, 0.1, 0.55)
        set_color(marker, color)
        return marker

    def make_obstacle_marker(self, now, scene: SceneSummary) -> Marker:
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = scene.header.frame_id or "base_footprint"
        marker.ns = "front_obstacle"
        marker.id = 4
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(scene.front_obstacle_x)
        marker.pose.position.y = float(scene.front_obstacle_y)
        marker.pose.position.z = 0.7
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.6, float(scene.front_obstacle_length))
        marker.scale.y = max(0.6, float(scene.front_obstacle_width))
        marker.scale.z = 0.8
        color = (1.0, 0.0, 0.0, 0.8) if scene.obstacle_risk_level == "HIGH" else (1.0, 0.45, 0.0, 0.75)
        set_color(marker, color)
        return marker

    def status_color(self, behavior: str, brake_active: bool, scene_health: str) -> tuple[float, float, float, float]:
        if brake_active or behavior == "EMERGENCY":
            return (1.0, 0.0, 0.0, 1.0)
        if behavior in {"OBSTACLE", "TRAFFIC_LIGHT", "RECOVERY", "ROUTE_REJOIN"} or scene_health in {"STALE", "LOST"}:
            return (1.0, 0.75, 0.0, 1.0)
        return (0.2, 1.0, 0.35, 1.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DebugMonitorNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
