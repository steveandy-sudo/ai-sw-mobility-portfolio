#!/usr/bin/env python3
from __future__ import annotations

import math
import re

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import (
    Centerline,
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


def light_numeric_id(light_id: str) -> int | None:
    """Trailing digits of a map light id: the track_id pv26 gives that light (traffic_light_manual_305 -> 305)."""
    digits = re.search(r"(\d+)$", str(light_id).strip())
    return int(digits.group(1)) if digits else None


def lights_by_map_id(lights, light_ids) -> dict[str, TrafficLightObservation]:
    """Observation of each map light id, the most confident one when several carry its track_id.

    Perception decides which map light an observation is (track_id) and its state; this only
    looks the ids up. An unassociated observation has track_id 0 and matches nothing.
    """
    wanted = {light_numeric_id(light_id): str(light_id) for light_id in light_ids}
    wanted.pop(None, None)
    wanted.pop(0, None)
    found: dict[str, TrafficLightObservation] = {}
    for light in lights:
        light_id = wanted.get(int(light.track_id))
        if light_id is None:
            continue
        if light_id not in found or float(light.confidence) > float(found[light_id].confidence):
            found[light_id] = light
    return found


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

        self.stale_data_sec = float(self.get_parameter("stale_data_sec").value)
        self.lost_data_sec = float(self.get_parameter("lost_data_sec").value)

        self.latest_road_segments: RoadSegmentArray | None = None
        self.latest_centerline: Centerline | None = None
        self.latest_objects: PerceptionObjectArray | None = None
        self.latest_obstacle_objects: PerceptionObjectArray | None = None
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
        previous = getattr(self, "latest_obstacle_objects", None)
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if previous is None or stamp >= (previous.header.stamp.sec, previous.header.stamp.nanosec):
            self.latest_obstacle_objects = msg
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
        objects = getattr(self, "latest_obstacle_objects", None) or self.latest_objects
        road_segments = self.latest_road_segments
        traffic_lights = self.latest_traffic_lights

        if centerline is None and objects is None:
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

        (
            summary.stopline_distance,
            traffic_state,
            traffic_conf,
            traffic_arrow_active,
        ) = self.summarize_traffic_control(
            traffic_lights if ages["traffic_lights"] <= self.stale_data_sec else None,
            self.latest_route_context,
        )
        summary.traffic_light_state = traffic_state
        summary.traffic_light_arrow_active = traffic_arrow_active
        summary.traffic_light_confidence = traffic_conf

        # Obstacles are judged by the planning engine's obstacle world (all objects in the zone),
        # so the summary keeps its obstacle fields at their defaults.
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
            f"arrow={summary.traffic_light_arrow_active} "
            f"stopline={summary.stopline_distance:.2f} "
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
    ) -> tuple[float, int, float, bool]:
        if (
            route is None
            or not route.route_projection_valid
            or not route.next_stop_line_id
        ):
            return INF_DISTANCE, SceneSummary.TRAFFIC_UNKNOWN, 0.0, False
        stopline_distance = float(route.distance_to_stopline)
        if not math.isfinite(stopline_distance) or stopline_distance >= INF_DISTANCE:
            return INF_DISTANCE, SceneSummary.TRAFFIC_UNKNOWN, 0.0, False
        if traffic_lights is None or not route.next_traffic_light_ids:
            return stopline_distance, SceneSummary.TRAFFIC_UNKNOWN, 0.0, False

        seen = lights_by_map_id(traffic_lights.lights, route.next_traffic_light_ids)
        if not seen:
            return stopline_distance, SceneSummary.TRAFFIC_UNKNOWN, 0.0, False
        selected = max(seen.values(), key=lambda light: float(light.confidence))
        return (
            stopline_distance,
            self.map_light_observation(selected),
            float(selected.confidence),
            self.map_light_arrow_active(selected),
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
        """Perception's state for the light, as it is."""
        return PerceptionGatewayNode.map_light_state(light.state)

    @staticmethod
    def map_light_arrow_active(light: TrafficLightObservation) -> bool:
        return bool(int(light.lamps) & TrafficLightObservation.LAMP_ARROW)

    def events_for(self, summary: SceneSummary, ages: dict[str, float]) -> list[str]:
        events = []
        if not summary.centerline_valid:
            events.append("CENTERLINE_INVALID")
        if summary.stopline_distance <= 10.0:
            events.append("STOPLINE_NEAR")
        if summary.traffic_light_state == SceneSummary.TRAFFIC_RED and summary.traffic_light_confidence >= 0.5:
            events.append("RED_CONFIRMED")
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
