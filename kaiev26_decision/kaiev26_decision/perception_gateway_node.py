#!/usr/bin/env python3
from __future__ import annotations

import math

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import (
    Centerline,
    PerceptionObject,
    PerceptionObjectArray,
    RoadSegment,
    RoadSegmentArray,
    RouteContext,
    SceneSummary,
    TrafficLightObservation,
    TrafficLightObservationArray,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from kaiev26_decision.common import INF_DISTANCE, polyline_length, spin_until_shutdown, stamp_age_sec, traffic_state_name


OBSERVATION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
TRAFFIC_LIGHT_LAMP_ON_THRESHOLD = 0.5


class ObservationFrameJoiner:
    """Join the four split perception topics without mixing timestamps."""

    REQUIRED_KINDS = ("road_segments", "centerline", "objects", "traffic_lights")

    def __init__(self, max_pending_frames: int = 8) -> None:
        self.max_pending_frames = max_pending_frames
        self.pending: dict[int, dict[str, object]] = {}
        self.latest_complete_stamp_ns = -1

    def add(self, kind: str, message):
        stamp = message.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= self.latest_complete_stamp_ns:
            return None

        frame = self.pending.setdefault(stamp_ns, {})
        frame[kind] = message
        if not all(required in frame for required in self.REQUIRED_KINDS):
            self._drop_old_pending_frames()
            return None

        complete = tuple(frame[required] for required in self.REQUIRED_KINDS)
        self.latest_complete_stamp_ns = stamp_ns
        self.pending = {
            pending_stamp: pending_frame
            for pending_stamp, pending_frame in self.pending.items()
            if pending_stamp > stamp_ns
        }
        return complete

    def _drop_old_pending_frames(self) -> None:
        while len(self.pending) > self.max_pending_frames:
            del self.pending[min(self.pending)]


class PerceptionGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_perception_gateway")
        self.declare_parameter("road_segments_topic", "/perception/road_segments")
        self.declare_parameter("centerline_topic", "/perception/centerline")
        self.declare_parameter("objects_topic", "/perception/objects")
        self.declare_parameter("traffic_lights_topic", "/perception/traffic_lights")
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("scene_summary_topic", "/planning/scene_summary")
        self.declare_parameter("events_topic", "/planning/events")
        self.declare_parameter("debug_status_topic", "/debug/perception_status")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("stale_data_sec", 0.35)
        self.declare_parameter("lost_data_sec", 1.0)
        self.declare_parameter("obstacle_path_half_width_m", 1.4)
        self.declare_parameter("obstacle_detect_distance_m", 20.0)
        self.declare_parameter("traffic_light_map_match_distance_m", 2.0)

        self.stale_data_sec = float(self.get_parameter("stale_data_sec").value)
        self.lost_data_sec = float(self.get_parameter("lost_data_sec").value)
        self.obstacle_path_half_width_m = float(self.get_parameter("obstacle_path_half_width_m").value)
        self.obstacle_detect_distance_m = float(self.get_parameter("obstacle_detect_distance_m").value)
        self.traffic_light_map_match_distance_m = float(
            self.get_parameter("traffic_light_map_match_distance_m").value
        )

        self.latest_road_segments: RoadSegmentArray | None = None
        self.latest_centerline: Centerline | None = None
        self.latest_objects: PerceptionObjectArray | None = None
        self.latest_traffic_lights: TrafficLightObservationArray | None = None
        self.latest_route_context: RouteContext | None = None
        self.observation_frames = ObservationFrameJoiner()

        self.scene_pub = self.create_publisher(
            SceneSummary,
            str(self.get_parameter("scene_summary_topic").value),
            10,
        )
        self.events_pub = self.create_publisher(
            String,
            str(self.get_parameter("events_topic").value),
            10,
        )
        self.debug_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_status_topic").value),
            10,
        )

        self.create_subscription(
            RoadSegmentArray,
            str(self.get_parameter("road_segments_topic").value),
            self.on_road_segments,
            OBSERVATION_QOS,
        )
        self.create_subscription(
            Centerline,
            str(self.get_parameter("centerline_topic").value),
            self.on_centerline,
            OBSERVATION_QOS,
        )
        self.create_subscription(
            PerceptionObjectArray,
            str(self.get_parameter("objects_topic").value),
            self.on_objects,
            OBSERVATION_QOS,
        )
        self.create_subscription(
            TrafficLightObservationArray,
            str(self.get_parameter("traffic_lights_topic").value),
            self.on_traffic_lights,
            OBSERVATION_QOS,
        )
        self.create_subscription(
            RouteContext,
            str(self.get_parameter("route_context_topic").value),
            self.on_route_context,
            10,
        )

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self.publish_scene_summary)

    def on_road_segments(self, msg: RoadSegmentArray) -> None:
        self.accept_observation("road_segments", msg)

    def on_centerline(self, msg: Centerline) -> None:
        self.accept_observation("centerline", msg)

    def on_objects(self, msg: PerceptionObjectArray) -> None:
        self.accept_observation("objects", msg)

    def on_traffic_lights(self, msg: TrafficLightObservationArray) -> None:
        self.accept_observation("traffic_lights", msg)

    def on_route_context(self, msg: RouteContext) -> None:
        self.latest_route_context = msg

    def accept_observation(self, kind: str, msg) -> None:
        complete = self.observation_frames.add(kind, msg)
        if complete is None:
            return
        (
            self.latest_road_segments,
            self.latest_centerline,
            self.latest_objects,
            self.latest_traffic_lights,
        ) = complete

    def publish_scene_summary(self) -> None:
        centerline = self.latest_centerline
        objects = self.latest_objects
        road_segments = self.latest_road_segments
        traffic_lights = self.latest_traffic_lights

        if centerline is None:
            self.get_logger().warn(
                "perception gateway waiting for a complete same-stamp perception frame",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()
        ages = {
            "road": self.message_age(now, road_segments),
            "centerline": self.message_age(now, centerline),
            "objects": self.message_age(now, objects),
            "traffic_lights": self.message_age(now, traffic_lights),
        }
        oldest_age = max(age for age in ages.values() if age < INF_DISTANCE) if any(
            age < INF_DISTANCE for age in ages.values()
        ) else INF_DISTANCE

        summary = SceneSummary()
        summary.header.stamp = now.to_msg()
        summary.header.frame_id = self.pick_frame_id(centerline, objects, road_segments, traffic_lights)

        summary.centerline_valid = (
            centerline is not None
            and ages["centerline"] <= self.stale_data_sec
            and len(centerline.points) >= 2
            and centerline.confidence > 0.0
        )
        summary.lane_valid = summary.centerline_valid
        summary.centerline_quality = float(centerline.confidence if centerline is not None else 0.0)
        summary.centerline_points = list(centerline.points) if centerline is not None else []
        summary.centerline_visible_length = float(polyline_length(summary.centerline_points))
        (
            summary.left_lane_center_valid,
            summary.left_lane_center_points,
            summary.right_lane_center_valid,
            summary.right_lane_center_points,
            summary.lane_width_estimate_m,
        ) = self.estimate_adjacent_lane_centers(road_segments, summary.centerline_points)

        summary.stopline_distance, traffic_state, traffic_conf = self.summarize_traffic_control(
            traffic_lights if ages["traffic_lights"] <= self.stale_data_sec else None,
            self.latest_route_context,
        )
        summary.traffic_light_state = traffic_state
        summary.traffic_light_confidence = traffic_conf

        (
            obstacle_on_path,
            front_obstacle_distance,
            obstacle_x,
            obstacle_y,
            obstacle_length,
            obstacle_width,
            risk_level,
        ) = self.summarize_obstacles(objects)
        summary.obstacle_on_path = obstacle_on_path
        summary.front_obstacle_distance = float(front_obstacle_distance)
        summary.front_obstacle_x = float(obstacle_x)
        summary.front_obstacle_y = float(obstacle_y)
        summary.front_obstacle_length = float(obstacle_length)
        summary.front_obstacle_width = float(obstacle_width)
        summary.obstacle_risk_level = risk_level

        summary.oldest_data_age_ms = float(oldest_age * 1000.0 if oldest_age < INF_DISTANCE else INF_DISTANCE)
        summary.events = self.events_for(summary, ages)
        summary.perception_health = self.health_for(summary, ages)

        self.scene_pub.publish(summary)
        events = String()
        events.data = ",".join(summary.events)
        self.events_pub.publish(events)

        debug = String()
        debug.data = (
            f"health={summary.perception_health} centerline={summary.centerline_valid} "
            f"oldest_ms={summary.oldest_data_age_ms:.1f} light={traffic_state_name(summary.traffic_light_state)} "
            f"stopline={summary.stopline_distance:.2f} obstacle={summary.obstacle_on_path} "
            f"front={summary.front_obstacle_distance:.2f} "
            f"obs_xy=({summary.front_obstacle_x:.2f},{summary.front_obstacle_y:.2f}) "
            f"lanes=L:{summary.left_lane_center_valid} R:{summary.right_lane_center_valid}"
        )
        self.debug_pub.publish(debug)

    def message_age(self, now, msg) -> float:
        if msg is None:
            return INF_DISTANCE
        return stamp_age_sec(now, msg.header.stamp)

    def pick_frame_id(self, *msgs) -> str:
        for msg in msgs:
            if msg is not None and msg.header.frame_id:
                return msg.header.frame_id
        return "base_footprint"

    def summarize_traffic_control(
        self,
        traffic_lights: TrafficLightObservationArray | None,
        route: RouteContext | None,
    ) -> tuple[float, int, float]:
        if (
            route is None
            or not route.route_projection_valid
            or not route.next_stop_line_id
        ):
            return INF_DISTANCE, SceneSummary.TRAFFIC_UNKNOWN, 0.0
        stopline_distance = float(route.distance_to_stopline)
        if not math.isfinite(stopline_distance) or stopline_distance >= INF_DISTANCE:
            return INF_DISTANCE, SceneSummary.TRAFFIC_UNKNOWN, 0.0
        if traffic_lights is None or not route.next_traffic_light_positions:
            return stopline_distance, SceneSummary.TRAFFIC_UNKNOWN, 0.0

        matches = [
            (
                math.hypot(
                    float(light.position.x) - float(expected.x),
                    float(light.position.y) - float(expected.y),
                ),
                light,
            )
            for light in traffic_lights.lights
            for expected in route.next_traffic_light_positions
        ]
        if not matches:
            return stopline_distance, SceneSummary.TRAFFIC_UNKNOWN, 0.0
        match_distance, selected = min(matches, key=lambda item: item[0])
        if match_distance > self.traffic_light_map_match_distance_m:
            return stopline_distance, SceneSummary.TRAFFIC_UNKNOWN, 0.0
        return (
            stopline_distance,
            self.map_light_observation(selected),
            float(selected.confidence),
        )

    def estimate_adjacent_lane_centers(
        self,
        road_segments: RoadSegmentArray | None,
        centerline_points: list[Point],
    ) -> tuple[bool, list[Point], bool, list[Point], float]:
        lane_segments = self.lane_segments(road_segments)
        if len(lane_segments) < 3 or len(centerline_points) < 2:
            return False, [], False, [], 0.0

        left_points: list[Point] = []
        right_points: list[Point] = []
        lane_width_samples: list[float] = []
        for center_point in centerline_points:
            if center_point.x < 0.0:
                continue
            current_y = float(center_point.y)
            boundary_y_values = sorted(
                y
                for segment in lane_segments
                for y in [self.lane_y_at_x(segment, float(center_point.x))]
                if y is not None
            )
            if len(boundary_y_values) < 3:
                continue

            right_boundaries = sorted((y for y in boundary_y_values if y < current_y - 0.15), reverse=True)
            left_boundaries = sorted(y for y in boundary_y_values if y > current_y + 0.15)
            if right_boundaries and left_boundaries:
                lane_width_samples.append(abs(left_boundaries[0] - right_boundaries[0]))
            if len(left_boundaries) >= 2:
                left_points.append(self.make_point(center_point.x, (left_boundaries[0] + left_boundaries[1]) * 0.5))
            if len(right_boundaries) >= 2:
                right_points.append(self.make_point(center_point.x, (right_boundaries[0] + right_boundaries[1]) * 0.5))

        lane_width = sum(lane_width_samples) / len(lane_width_samples) if lane_width_samples else 0.0
        return len(left_points) >= 2, left_points, len(right_points) >= 2, right_points, float(lane_width)

    def lane_segments(self, road_segments: RoadSegmentArray | None) -> list[RoadSegment]:
        if road_segments is None:
            return []
        lane_types = {
            RoadSegment.TYPE_WHSOL,
            RoadSegment.TYPE_WHDOT,
            RoadSegment.TYPE_YESOL,
            RoadSegment.TYPE_YEDOT,
        }
        return [segment for segment in road_segments.segments if segment.type in lane_types and len(segment.points) >= 2]

    def lane_y_at_x(self, segment: RoadSegment, target_x: float) -> float | None:
        points = segment.points
        closest = min(points, key=lambda point: abs(float(point.x) - target_x))
        for start, end in zip(points, points[1:]):
            min_x = min(float(start.x), float(end.x))
            max_x = max(float(start.x), float(end.x))
            if target_x < min_x or target_x > max_x:
                continue
            dx = float(end.x) - float(start.x)
            if abs(dx) < 1.0e-6:
                continue
            ratio = (target_x - float(start.x)) / dx
            return float(start.y) + ratio * (float(end.y) - float(start.y))
        return float(closest.y)

    def make_point(self, x: float, y: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        return point

    @staticmethod
    def map_light_state(state: int) -> int:
        mapping = {
            TrafficLightObservation.STATE_RED: SceneSummary.TRAFFIC_RED,
            TrafficLightObservation.STATE_YELLOW: SceneSummary.TRAFFIC_YELLOW,
            TrafficLightObservation.STATE_GREEN: SceneSummary.TRAFFIC_GREEN,
            TrafficLightObservation.STATE_ARROW: SceneSummary.TRAFFIC_ARROW,
        }
        return mapping.get(int(state), SceneSummary.TRAFFIC_UNKNOWN)

    @staticmethod
    def map_light_observation(light: TrafficLightObservation) -> int:
        scores = {
            "red": float(light.red_score),
            "yellow": float(light.yellow_score),
            "green": float(light.green_score),
            "arrow": float(light.arrow_score),
        }
        active = {
            name
            for name, score in scores.items()
            if math.isfinite(score) and score >= TRAFFIC_LIGHT_LAMP_ON_THRESHOLD
        }
        if "yellow" in active:
            return SceneSummary.TRAFFIC_YELLOW
        if "red" in active and "green" in active:
            return SceneSummary.TRAFFIC_RED
        if "red" in active and "arrow" in active:
            return SceneSummary.TRAFFIC_ARROW
        if "red" in active:
            return SceneSummary.TRAFFIC_RED
        if "green" in active:
            return SceneSummary.TRAFFIC_GREEN
        if "arrow" in active:
            return SceneSummary.TRAFFIC_ARROW
        return PerceptionGatewayNode.map_light_state(light.state)

    def summarize_obstacles(
        self,
        objects: PerceptionObjectArray | None,
    ) -> tuple[bool, float, float, float, float, float, str]:
        if objects is None:
            return False, INF_DISTANCE, INF_DISTANCE, 0.0, 0.0, 0.0, "UNKNOWN"

        front_distance = INF_DISTANCE
        selected_x = INF_DISTANCE
        selected_y = 0.0
        selected_length = 0.0
        selected_width = 0.0
        obstacle_classes = {
            PerceptionObject.CLASS_VEHICLE,
            PerceptionObject.CLASS_BIKE,
            PerceptionObject.CLASS_PEDESTRIAN,
            PerceptionObject.CLASS_TRAFFIC_CONE,
            PerceptionObject.CLASS_OBSTACLE,
        }
        for obj in objects.objects:
            if obj.class_id not in obstacle_classes:
                continue
            x = float(obj.pose.position.x)
            if x < 0.0:
                continue
            half_width = max(0.1, float(obj.size.y) * 0.5)
            lateral_limit = self.obstacle_path_half_width_m + half_width
            if abs(float(obj.pose.position.y)) > lateral_limit:
                continue
            distance = max(0.0, x - max(0.0, float(obj.size.x) * 0.5))
            if distance < front_distance:
                front_distance = distance
                selected_x = x
                selected_y = float(obj.pose.position.y)
                selected_length = max(0.0, float(obj.size.x))
                selected_width = max(0.0, float(obj.size.y))

        obstacle_on_path = front_distance <= self.obstacle_detect_distance_m
        if not obstacle_on_path:
            return False, front_distance, selected_x, selected_y, selected_length, selected_width, "LOW"
        if front_distance <= 2.0:
            return True, front_distance, selected_x, selected_y, selected_length, selected_width, "HIGH"
        if front_distance <= 6.0:
            return True, front_distance, selected_x, selected_y, selected_length, selected_width, "MEDIUM"
        return True, front_distance, selected_x, selected_y, selected_length, selected_width, "LOW"

    def events_for(self, summary: SceneSummary, ages: dict[str, float]) -> list[str]:
        events = []
        if not summary.centerline_valid:
            events.append("CENTERLINE_INVALID")
        if summary.stopline_distance <= 10.0:
            events.append("STOPLINE_NEAR")
        if summary.traffic_light_state == SceneSummary.TRAFFIC_RED and summary.traffic_light_confidence >= 0.5:
            events.append("RED_CONFIRMED")
        if summary.obstacle_on_path:
            events.append("OBSTACLE_ON_PATH")
        for name, age in ages.items():
            if age > self.stale_data_sec:
                events.append(f"{name.upper()}_STALE")
        return events

    def health_for(self, summary: SceneSummary, ages: dict[str, float]) -> str:
        essential = [ages["centerline"], ages["objects"]]
        if any(age >= self.lost_data_sec for age in essential):
            return "LOST"
        if any(age > self.stale_data_sec for age in essential):
            return "STALE"
        if not summary.centerline_valid or summary.centerline_quality < 0.55:
            return "DEGRADED"
        return "GOOD"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerceptionGatewayNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
