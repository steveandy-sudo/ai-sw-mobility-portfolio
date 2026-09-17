#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Point, PoseStamped
from kaiev26_msgs.msg import RouteContext
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

from kaiev26_decision_fox.trail import TrailBuffer, TrailPoint


@dataclass(frozen=True)
class VehiclePose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    frame_id: str


def set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


class FoxNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_decision_fox")
        self.declare_parameter("odometry_topic", "/localization/kinematic_state")
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("trajectory_topic", "/debug/decision_fox/trajectory")
        self.declare_parameter("marker_topic", "/debug/decision_fox/markers")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("trail_max_points", 4000)
        self.declare_parameter("trail_min_spacing_m", 0.10)
        self.declare_parameter("trail_max_jump_m", 8.0)
        self.declare_parameter("vehicle_length_m", 2.60)
        self.declare_parameter("vehicle_width_m", 1.18)
        self.declare_parameter("vehicle_height_m", 0.75)

        self.trail = TrailBuffer(
            max_points=int(self.get_parameter("trail_max_points").value),
            min_spacing_m=float(self.get_parameter("trail_min_spacing_m").value),
            max_jump_m=float(self.get_parameter("trail_max_jump_m").value),
        )
        self.vehicle_pose: VehiclePose | None = None
        self.current_zone = "UNKNOWN_ZONE"
        self.vehicle_length_m = float(self.get_parameter("vehicle_length_m").value)
        self.vehicle_width_m = float(self.get_parameter("vehicle_width_m").value)
        self.vehicle_height_m = float(self.get_parameter("vehicle_height_m").value)

        self.path_publisher = self.create_publisher(
            Path,
            str(self.get_parameter("trajectory_topic").value),
            10,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            RouteContext,
            str(self.get_parameter("route_context_topic").value),
            self.on_route_context,
            10,
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self.publish_visualization)
        self.get_logger().info("decision Foxglove visualization outputs are ready")

    def on_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        frame_id = message.header.frame_id or "map"
        self.vehicle_pose = VehiclePose(
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            qx=float(pose.orientation.x),
            qy=float(pose.orientation.y),
            qz=float(pose.orientation.z),
            qw=float(pose.orientation.w),
            frame_id=frame_id,
        )
        self.trail.add(
            TrailPoint(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
                stamp_ns=stamp_ns,
            )
        )

    def on_route_context(self, message: RouteContext) -> None:
        self.current_zone = message.current_zone or "UNKNOWN_ZONE"

    def publish_visualization(self) -> None:
        pose = self.vehicle_pose
        if pose is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.make_path(stamp, pose.frame_id))
        self.marker_publisher.publish(self.make_markers(stamp, pose))

    def make_path(self, stamp, frame_id: str) -> Path:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id
        for point in self.trail.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = point.z + 0.08
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def make_markers(self, stamp, pose: VehiclePose) -> MarkerArray:
        result = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        vehicle = Marker()
        vehicle.header.stamp = stamp
        vehicle.header.frame_id = pose.frame_id
        vehicle.ns = "decision_fox_vehicle"
        vehicle.id = 0
        vehicle.type = Marker.CUBE
        vehicle.action = Marker.ADD
        vehicle.pose.position.x = pose.x
        vehicle.pose.position.y = pose.y
        vehicle.pose.position.z = pose.z + self.vehicle_height_m * 0.5
        vehicle.pose.orientation.x = pose.qx
        vehicle.pose.orientation.y = pose.qy
        vehicle.pose.orientation.z = pose.qz
        vehicle.pose.orientation.w = pose.qw
        vehicle.scale.x = self.vehicle_length_m
        vehicle.scale.y = self.vehicle_width_m
        vehicle.scale.z = self.vehicle_height_m
        set_color(vehicle, (0.05, 0.88, 1.0, 0.58))
        result.markers.append(vehicle)

        zone = Marker()
        zone.header = vehicle.header
        zone.ns = "decision_fox_zone"
        zone.id = 0
        zone.type = Marker.TEXT_VIEW_FACING
        zone.action = Marker.ADD
        zone.pose.position.x = pose.x
        zone.pose.position.y = pose.y
        zone.pose.position.z = pose.z + self.vehicle_height_m + 1.0
        zone.pose.orientation.w = 1.0
        zone.scale.z = 0.55
        zone.text = self.current_zone
        set_color(zone, (0.92, 0.96, 1.0, 0.96))
        result.markers.append(zone)

        trail = Marker()
        trail.header = vehicle.header
        trail.ns = "decision_fox_trajectory"
        trail.id = 0
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        trail.pose.orientation.w = 1.0
        trail.scale.x = 0.12
        trail.points = [
            Point(x=point.x, y=point.y, z=point.z + 0.10)
            for point in self.trail.points
        ]
        set_color(trail, (0.96, 0.98, 1.0, 0.92))
        result.markers.append(trail)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
