#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from geometry_msgs.msg import Point
from kaiev26_msgs.msg import RouteContext
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time as RosTime
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from kaiev26_decision.common import spin_until_shutdown
from kaiev26_decision.zone_policy import ZonePolicy, load_zone_policy


INF_DISTANCE = 1.0e6


@dataclass(frozen=True)
class RouteWaypoint:
    x: float
    y: float
    z: float = 0.0
    speed_mps: float = 2.0
    zone: str = "NORMAL_ZONE"


@dataclass(frozen=True)
class Projection:
    progress_s: float
    x: float
    y: float
    yaw: float
    cross_track_error: float
    distance: float


@dataclass(frozen=True)
class RouteZoneSpan:
    zone: str
    start_s: float
    end_s: float


@dataclass(frozen=True)
class RouteMapPoint:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class RouteStopline:
    landmark_id: str
    progress_s: float
    width_m: float = 4.0
    rule_id: str = ""
    traffic_light_ids: tuple[str, ...] = ()
    traffic_light_positions: tuple[RouteMapPoint, ...] = ()


def merge_route_stoplines(stoplines: list[RouteStopline]) -> list[RouteStopline]:
    unique: dict[str, RouteStopline] = {}
    for stopline in stoplines:
        existing = unique.get(stopline.landmark_id)
        if existing is not None and abs(existing.progress_s - stopline.progress_s) > 0.5:
            raise ValueError(
                f"stop line [{stopline.landmark_id}] has conflicting route positions"
            )
        if existing is None:
            unique[stopline.landmark_id] = stopline
        elif stopline.traffic_light_ids:
            inherited_positions = (
                existing.traffic_light_positions
                if stopline.traffic_light_ids == existing.traffic_light_ids
                else ()
            )
            unique[stopline.landmark_id] = RouteStopline(
                landmark_id=stopline.landmark_id,
                progress_s=stopline.progress_s,
                width_m=stopline.width_m,
                rule_id=stopline.rule_id,
                traffic_light_ids=stopline.traffic_light_ids,
                traffic_light_positions=(
                    stopline.traffic_light_positions or inherited_positions
                ),
            )
    return sorted(unique.values(), key=lambda item: item.progress_s)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_point(x: float, y: float, z: float = 0.0) -> Point:
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def odometry_position_covariance_usable(
    odometry: Odometry,
    max_projection_distance_m: float,
) -> bool:
    variance_limit = max_projection_distance_m * max_projection_distance_m
    x_variance = float(odometry.pose.covariance[0])
    y_variance = float(odometry.pose.covariance[7])
    return (
        math.isfinite(x_variance)
        and math.isfinite(y_variance)
        and 0.0 <= x_variance <= variance_limit
        and 0.0 <= y_variance <= variance_limit
    )


def set_color(marker: Marker, rgba: tuple[float, float, float, float]) -> None:
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]


class PolylineRoute:
    def __init__(
        self,
        waypoints: list[RouteWaypoint],
        zone_ranges: list[dict] | None = None,
        zone_spans: list[RouteZoneSpan] | None = None,
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError("global route requires at least two waypoints")
        self.waypoints = waypoints
        self.cumulative_s = [0.0]
        for start, end in zip(waypoints, waypoints[1:]):
            self.cumulative_s.append(
                self.cumulative_s[-1] + math.hypot(end.x - start.x, end.y - start.y)
            )
        self.total_length = self.cumulative_s[-1]
        self.zone_spans = (
            list(zone_spans)
            if zone_spans is not None
            else self.build_zone_spans(zone_ranges or [])
        )

    def build_zone_spans(self, zone_ranges: list[dict]) -> list[RouteZoneSpan]:
        spans = []
        for index, item in enumerate(zone_ranges):
            zone = str(item.get("zone", "")).strip()
            if not zone:
                raise ValueError(f"zone_ranges[{index}] requires a zone")
            if "start_s" in item or "end_s" in item:
                if "start_s" not in item or "end_s" not in item:
                    raise ValueError(
                        f"zone_ranges[{index}] requires both start_s and end_s"
                    )
                start_s = float(item["start_s"])
                end_s = float(item["end_s"])
                if not 0.0 <= start_s < end_s <= self.total_length + 1.0e-6:
                    raise ValueError(
                        f"zone_ranges[{index}] progress range "
                        f"{start_s:.2f}..{end_s:.2f} is outside the route"
                    )
            else:
                start_waypoint = int(item["start_waypoint"])
                end_waypoint = int(item["end_waypoint"])
                if not 1 <= start_waypoint <= end_waypoint <= len(self.waypoints):
                    raise ValueError(
                        f"zone_ranges[{index}] waypoint range "
                        f"{start_waypoint}..{end_waypoint} is outside "
                        f"1..{len(self.waypoints)}"
                    )
                start_s = self.cumulative_s[start_waypoint - 1]
                end_s = self.cumulative_s[end_waypoint - 1]
            spans.append(
                RouteZoneSpan(
                    zone=zone,
                    start_s=start_s,
                    end_s=end_s,
                )
            )

        spans.sort(key=lambda span: (span.start_s, span.end_s))
        for previous, current in zip(spans, spans[1:]):
            if current.start_s < previous.end_s - 1e-6:
                raise ValueError("zone_ranges must not overlap")
        return spans

    def set_zone_ranges(self, zone_ranges: list[dict]) -> None:
        self.zone_spans = self.build_zone_spans(zone_ranges)

    def project(self, x: float, y: float) -> Projection:
        return self.project_segments(x, y, range(len(self.waypoints) - 1))

    def project_with_heading(
        self,
        x: float,
        y: float,
        yaw: float,
        max_heading_error_rad: float,
        heading_weight_m: float,
    ) -> Projection:
        """Select the nearest route segment that also points with the vehicle."""
        candidates = self.projection_candidates(
            x,
            y,
            range(len(self.waypoints) - 1),
        )
        aligned = [
            projection
            for projection in candidates
            if abs(normalize_angle(projection.yaw - yaw))
            <= max_heading_error_rad
        ]
        pool = aligned or candidates
        if not pool:
            return self.project_segments(x, y, ())
        return min(
            pool,
            key=lambda projection: (
                projection.distance
                + heading_weight_m
                * abs(normalize_angle(projection.yaw - yaw)),
                projection.distance,
            ),
        )

    def project_prefix(self, x: float, y: float, max_progress_s: float) -> Projection:
        indices = [
            index
            for index, start_s in enumerate(self.cumulative_s[:-1])
            if start_s <= max_progress_s
        ]
        return self.project_segments(x, y, indices)

    def project_near_progress(self, x: float, y: float, center_s: float, window_m: float) -> Projection:
        start_s = max(0.0, center_s - window_m)
        end_s = min(self.total_length, center_s + window_m)
        indices = [
            index
            for index, (segment_start_s, segment_end_s) in enumerate(zip(self.cumulative_s, self.cumulative_s[1:]))
            if segment_end_s >= start_s and segment_start_s <= end_s
        ]
        return self.project_segments(x, y, indices)

    def project_segments(self, x: float, y: float, segment_indices) -> Projection:
        candidates = self.projection_candidates(x, y, segment_indices)
        if candidates:
            return min(candidates, key=lambda projection: projection.distance)

        first = self.waypoints[0]
        second = self.waypoints[1]
        yaw = math.atan2(second.y - first.y, second.x - first.x)
        return Projection(0.0, first.x, first.y, yaw, 0.0, INF_DISTANCE)

    def projection_candidates(self, x: float, y: float, segment_indices) -> list[Projection]:
        candidates: list[Projection] = []
        for index in segment_indices:
            start = self.waypoints[index]
            end = self.waypoints[index + 1]
            dx = end.x - start.x
            dy = end.y - start.y
            segment_len2 = dx * dx + dy * dy
            if segment_len2 <= 1e-9:
                continue

            t = ((x - start.x) * dx + (y - start.y) * dy) / segment_len2
            t = max(0.0, min(1.0, t))
            px = start.x + t * dx
            py = start.y + t * dy
            distance = math.hypot(x - px, y - py)
            yaw = math.atan2(dy, dx)
            cross_track = -math.sin(yaw) * (x - px) + math.cos(yaw) * (y - py)
            progress = self.cumulative_s[index] + math.sqrt(segment_len2) * t
            candidates.append(
                Projection(progress, px, py, yaw, cross_track, distance)
            )
        return candidates

    def sample_at(self, progress_s: float) -> tuple[float, float, float, str]:
        s = max(0.0, min(self.total_length, progress_s))
        for index, (start_s, end_s) in enumerate(zip(self.cumulative_s, self.cumulative_s[1:])):
            if s > end_s and index + 1 < len(self.cumulative_s) - 1:
                continue
            start = self.waypoints[index]
            end = self.waypoints[index + 1]
            segment_len = max(1e-9, end_s - start_s)
            ratio = (s - start_s) / segment_len
            x = start.x + (end.x - start.x) * ratio
            y = start.y + (end.y - start.y) * ratio
            z = start.z + (end.z - start.z) * ratio
            zone = end.zone if ratio > 0.5 and end.zone != "NORMAL_ZONE" else start.zone
            return x, y, z, zone
        last = self.waypoints[-1]
        return last.x, last.y, last.z, last.zone

    def heading_at(self, progress_s: float) -> float:
        s = max(0.0, min(self.total_length, progress_s))
        for index, end_s in enumerate(self.cumulative_s[1:]):
            if s <= end_s + 1e-9:
                start = self.waypoints[index]
                end = self.waypoints[index + 1]
                return math.atan2(end.y - start.y, end.x - start.x)
        start = self.waypoints[-2]
        end = self.waypoints[-1]
        return math.atan2(end.y - start.y, end.x - start.x)

    def zone_progress(self, zone_name: str) -> list[float]:
        waypoint_progress = [
            s
            for s, waypoint in zip(self.cumulative_s, self.waypoints)
            if waypoint.zone == zone_name
        ]
        span_progress = [
            span.start_s for span in self.zone_spans if span.zone == zone_name
        ]
        return sorted(set(waypoint_progress + span_progress))

    def zone_at(self, progress_s: float) -> str:
        for span in self.zone_spans:
            if span.start_s - 1e-6 <= progress_s <= span.end_s + 1e-6:
                return span.zone
        return "NORMAL_ZONE"

    def distance_to_zone(self, progress_s: float, zone_name: str) -> float:
        for span in self.zone_spans:
            if span.zone != zone_name:
                continue
            if span.start_s - 1e-6 <= progress_s <= span.end_s + 1e-6:
                return 0.0

        ahead = [
            zone_s - progress_s
            for zone_s in self.zone_progress(zone_name)
            if zone_s >= progress_s
        ]
        return min(ahead) if ahead else INF_DISTANCE

    def next_zone_after(self, progress_s: float) -> str | None:
        current_index = None
        for index, span in enumerate(self.zone_spans):
            if span.start_s - 1.0e-6 <= progress_s <= span.end_s + 1.0e-6:
                current_index = index
                break
        if current_index is not None:
            for span in self.zone_spans[current_index + 1 :]:
                if span.zone != self.zone_spans[current_index].zone:
                    return span.zone
            return None
        for span in self.zone_spans:
            if span.start_s > progress_s:
                return span.zone
        return None


def parse_route_stoplines(
    route: PolylineRoute,
    data: dict,
    max_snap_distance_m: float = 5.0,
) -> list[RouteStopline]:
    landmarks = data.get("landmarks", [])
    if not isinstance(landmarks, list):
        raise ValueError("landmark config landmarks must be a list")

    stoplines = []
    for index, item in enumerate(landmarks):
        if not isinstance(item, dict):
            raise ValueError(f"landmarks[{index}] must be a mapping")
        if not bool(item.get("enabled", True)):
            continue
        if str(item.get("type", "")).upper() != "STOP_LINE":
            continue

        landmark_id = str(item.get("id", f"stopline_{index + 1:02d}"))
        if "route_s" in item:
            progress_s = float(item["route_s"])
        elif "waypoint" in item:
            waypoint_number = int(item["waypoint"])
            if not 1 <= waypoint_number <= len(route.waypoints):
                raise ValueError(
                    f"landmarks[{index}] waypoint {waypoint_number} is outside "
                    f"1..{len(route.waypoints)}"
                )
            progress_s = route.cumulative_s[waypoint_number - 1]
        elif "x" in item and "y" in item:
            projection = route.project(float(item["x"]), float(item["y"]))
            if projection.distance > max_snap_distance_m:
                raise ValueError(
                    f"landmarks[{index}] is {projection.distance:.2f} m from the route"
                )
            progress_s = projection.progress_s
        else:
            raise ValueError(
                f"landmarks[{index}] requires route_s, waypoint, or x/y"
            )

        if not 0.0 <= progress_s <= route.total_length:
            raise ValueError(
                f"landmarks[{index}] route_s {progress_s:.2f} is outside the route"
            )

        traffic_light_ids_value = item.get("traffic_light_ids", [])
        if not isinstance(traffic_light_ids_value, list):
            raise ValueError(
                f"landmarks[{index}].traffic_light_ids must be a list"
            )
        traffic_light_positions_value = item.get("traffic_light_positions", [])
        if not isinstance(traffic_light_positions_value, list):
            raise ValueError(
                f"landmarks[{index}].traffic_light_positions must be a list"
            )
        traffic_light_positions = []
        for position_index, position in enumerate(traffic_light_positions_value):
            if (
                not isinstance(position, dict)
                or "x" not in position
                or "y" not in position
            ):
                raise ValueError(
                    f"landmarks[{index}].traffic_light_positions[{position_index}] "
                    "requires x/y"
                )
            traffic_light_positions.append(
                RouteMapPoint(
                    x=float(position["x"]),
                    y=float(position["y"]),
                    z=float(position.get("z", 0.0)),
                )
            )
        stoplines.append(
            RouteStopline(
                landmark_id=landmark_id,
                progress_s=progress_s,
                width_m=max(0.5, float(item.get("width_m", 4.0))),
                rule_id=str(item.get("rule_id", "")).strip(),
                traffic_light_ids=tuple(
                    str(value).strip()
                    for value in traffic_light_ids_value
                    if str(value).strip()
                ),
                traffic_light_positions=tuple(traffic_light_positions),
            )
        )

    stoplines.sort(key=lambda landmark: landmark.progress_s)
    return stoplines


def distance_to_next_stopline(
    progress_s: float,
    stoplines: list[RouteStopline],
    passed_tolerance_m: float = 0.25,
) -> float:
    distances = [
        stopline.progress_s - progress_s
        for stopline in stoplines
        if stopline.progress_s >= progress_s - passed_tolerance_m
    ]
    if not distances:
        return INF_DISTANCE
    return max(0.0, min(distances))


class RouteZoneManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_route_zone_manager")
        self.declare_parameter("odometry_topic", "/localization/odometry")
        self.declare_parameter("route_context_topic", "/planning/route_context")
        self.declare_parameter("global_route_marker_topic", "/debug/global_route_markers")
        self.declare_parameter("route_state_marker_topic", "/debug/route_state_markers")
        self.declare_parameter("route_config", "")
        self.declare_parameter("landmark_config", "")
        self.declare_parameter("mission_policy_config", "")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("lookahead_distance_m", 18.0)
        self.declare_parameter("local_path_spacing_m", 1.0)
        self.declare_parameter("max_projection_distance_m", 5.0)
        self.declare_parameter("progress_search_window_m", 35.0)
        self.declare_parameter("start_snap_distance_m", 8.0)
        self.declare_parameter("start_snap_heading_error_rad", 0.8)
        self.declare_parameter("start_snap_progress_m", 35.0)
        self.declare_parameter("initial_projection_max_heading_error_rad", 0.7)
        self.declare_parameter("initial_projection_heading_weight_m", 4.0)
        self.declare_parameter("finish_zone_distance_m", 4.0)
        self.declare_parameter("intersection_zone_radius_m", 8.0)
        self.declare_parameter("landmark_snap_distance_m", 5.0)
        self.declare_parameter("stopline_passed_tolerance_m", 0.25)

        self.map_frame_id = str(self.get_parameter("map_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.lookahead_distance_m = float(self.get_parameter("lookahead_distance_m").value)
        self.local_path_spacing_m = max(0.2, float(self.get_parameter("local_path_spacing_m").value))
        self.max_projection_distance_m = max(1.0, float(self.get_parameter("max_projection_distance_m").value))
        self.progress_search_window_m = max(5.0, float(self.get_parameter("progress_search_window_m").value))
        self.start_snap_distance_m = max(0.0, float(self.get_parameter("start_snap_distance_m").value))
        self.start_snap_heading_error_rad = max(0.0, float(self.get_parameter("start_snap_heading_error_rad").value))
        self.start_snap_progress_m = max(2.0, float(self.get_parameter("start_snap_progress_m").value))
        self.initial_projection_max_heading_error_rad = max(
            0.0,
            float(
                self.get_parameter(
                    "initial_projection_max_heading_error_rad"
                ).value
            ),
        )
        self.initial_projection_heading_weight_m = max(
            0.0,
            float(
                self.get_parameter("initial_projection_heading_weight_m").value
            ),
        )
        self.finish_zone_distance_m = float(self.get_parameter("finish_zone_distance_m").value)
        self.intersection_zone_radius_m = float(self.get_parameter("intersection_zone_radius_m").value)
        policy_path = str(self.get_parameter("mission_policy_config").value).strip()
        self.zone_policy: ZonePolicy | None = (
            load_zone_policy(policy_path) if policy_path else None
        )
        self.route = self.load_route(str(self.get_parameter("route_config").value))
        self.stopline_passed_tolerance_m = max(
            0.0,
            float(self.get_parameter("stopline_passed_tolerance_m").value),
        )
        self.stoplines = self.load_stoplines(
            str(self.get_parameter("landmark_config").value),
            float(self.get_parameter("landmark_snap_distance_m").value),
        )
        if self.zone_policy is not None:
            self.validate_policy_controls()
            stopline_progress = {
                stopline.landmark_id: stopline.progress_s
                for stopline in self.stoplines
            }
            self.route.set_zone_ranges(
                self.zone_policy.zone_ranges(
                    stopline_progress,
                    self.route.total_length,
                )
            )
            self.get_logger().info(
                f"loaded zone policy [{self.zone_policy.course_id}] with "
                f"{len(self.zone_policy.stages)} stage(s)"
            )

        self.latest_odometry: Odometry | None = None
        self.last_progress_s: float | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            RouteContext,
            str(self.get_parameter("route_context_topic").value),
            10,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.global_route_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("global_route_marker_topic").value),
            marker_qos,
        )
        self.route_state_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("route_state_marker_topic").value),
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self.on_odometry,
            10,
        )

        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self.publish_route_context)
        self.publish_global_route_markers(self.get_clock().now().to_msg())

    def load_route(self, route_config_value: str) -> PolylineRoute:
        if not route_config_value.strip():
            raise ValueError(
                "route_config is required; select a course profile or provide a route YAML"
            )
        route_config = Path(route_config_value)
        with route_config.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError(f"route config must be a mapping: {route_config}")
        if "waypoints" not in data:
            raise ValueError(
                f"route config requires a waypoints list: {route_config}"
            )
        default_speed = float(data.get("default_speed_mps", 2.0))
        waypoints = [
            RouteWaypoint(
                x=float(item["x"]),
                y=float(item["y"]),
                z=0.0,
                speed_mps=float(item.get("speed_mps", default_speed)),
                zone=str(item.get("zone", "NORMAL_ZONE")),
            )
            for item in data["waypoints"]
        ]
        zone_ranges = data.get("zone_ranges", [])
        if not isinstance(zone_ranges, list):
            raise ValueError("route zone_ranges must be a list")
        route = PolylineRoute(waypoints, zone_ranges=zone_ranges)
        self.get_logger().info(
            f"loaded global route [{data.get('route_name', route_config.name)}] "
            f"with {len(waypoints)} waypoints"
        )
        return route

    def load_stoplines(
        self,
        landmark_config_value: str,
        max_snap_distance_m: float,
    ) -> list[RouteStopline]:
        stoplines = []
        if landmark_config_value:
            landmark_config = Path(landmark_config_value).expanduser()
            with landmark_config.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            stoplines.extend(
                parse_route_stoplines(
                    self.route,
                    data,
                    max_snap_distance_m=max(0.1, max_snap_distance_m),
                )
            )
            self.get_logger().info(
                f"loaded route landmarks from {landmark_config}"
            )

        return merge_route_stoplines(stoplines)

    def on_odometry(self, msg: Odometry) -> None:
        self.latest_odometry = msg

    def next_route_stopline(self, progress_s: float) -> RouteStopline | None:
        for stopline in self.stoplines:
            if stopline.progress_s >= progress_s - self.stopline_passed_tolerance_m:
                return stopline
        return None

    def policy_stage_at(self, progress_s: float):
        if self.zone_policy is None:
            return None
        zone = self.route.zone_at(progress_s)
        stage_index = self.zone_policy.stage_index(zone)
        if stage_index is None:
            return None
        return self.zone_policy.stages[stage_index]

    def next_policy_stopline(self, progress_s: float) -> RouteStopline | None:
        stage = self.policy_stage_at(progress_s)
        if stage is None or stage.event is None:
            return None
        stop_line_id = stage.event.stop_line_id
        if not stop_line_id:
            return None
        return next(
            (
                stopline
                for stopline in self.stoplines
                if stopline.landmark_id == stop_line_id
            ),
            None,
        )

    def validate_policy_controls(self) -> None:
        controls = {item.landmark_id: item for item in self.stoplines}
        for stage in self.zone_policy.stages:
            event = stage.event
            if event is None or not event.stop_line_id:
                continue
            control = controls.get(event.stop_line_id)
            if control is None:
                raise ValueError(
                    f"event [{event.event_id}] stop line is not available: "
                    f"{event.stop_line_id}"
                )
            if event.brake_trigger_waypoint is not None:
                waypoint = event.brake_trigger_waypoint
                if waypoint >= len(self.route.waypoints):
                    raise ValueError(
                        f"event [{event.event_id}] brake trigger WP {waypoint} is outside "
                        f"0..{len(self.route.waypoints) - 1}"
                    )
                expected_s = self.route.cumulative_s[waypoint]
                configured_s = float(event.brake_trigger_route_s)
                if abs(configured_s - expected_s) > 0.10:
                    raise ValueError(
                        f"event [{event.event_id}] brake trigger WP {waypoint} is "
                        f"{expected_s:.3f} m, not configured {configured_s:.3f} m"
                    )
                if configured_s > control.progress_s + 0.10:
                    raise ValueError(
                        f"event [{event.event_id}] brake trigger is after its stop line"
                    )
            if event.event_type != "SIGNAL_OBEY":
                continue
            expected = set(event.traffic_light_ids)
            available = set(control.traffic_light_ids)
            if not expected.issubset(available):
                raise ValueError(
                    f"event [{event.event_id}] traffic-light mapping is invalid: "
                    f"expected={sorted(expected)}, available={sorted(available)}"
                )

    def map_point_to_base(
        self,
        point_value: RouteMapPoint,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
    ) -> Point:
        dx = point_value.x - ego_x
        dy = point_value.y - ego_y
        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)
        return make_point(
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            0.0,
        )

    def next_maneuver(self, progress_s: float) -> str:
        if self.zone_policy is not None:
            stage = self.policy_stage_at(progress_s)
            if stage is not None and stage.event is not None:
                policy_maneuver = stage.event.arrow_maneuver.upper()
                if policy_maneuver in {"LEFT", "RIGHT"}:
                    return policy_maneuver
                if stage.event.event_type == "SIGNAL_OBEY":
                    return "STRAIGHT"
        return "UNKNOWN"

    def publish_route_context(self) -> None:
        msg = RouteContext()
        now = self.get_clock().now()
        if self.latest_odometry is None:
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.base_frame_id
            msg.current_zone = "UNKNOWN_ZONE"
            msg.next_zone = "NORMAL_ZONE"
            msg.next_maneuver = "UNKNOWN"
            msg.distance_to_stopline = INF_DISTANCE
            msg.distance_to_intersection = INF_DISTANCE
            msg.distance_to_finish = self.route.total_length
            msg.route_projection_valid = False
            msg.projection_confidence = 0.0
            self.publisher.publish(msg)
            self.publish_route_state_markers(now.to_msg(), None)
            return

        if not odometry_position_covariance_usable(
            self.latest_odometry,
            self.max_projection_distance_m,
        ):
            self.publish_invalid_route(now.to_msg(), "localization covariance unavailable")
            return

        pose = self.lookup_vehicle_pose()
        if pose is None:
            self.publish_invalid_route(now.to_msg(), "TF lookup failed")
            return

        x, y, yaw = pose
        projection = self.project_route(x, y, yaw)
        distance_to_finish = max(0.0, self.route.total_length - projection.progress_s)
        distance_to_intersection = self.distance_to_next_zone(
            projection.progress_s, "INTERSECTION_ZONE"
        )
        next_stopline = (
            self.next_policy_stopline(projection.progress_s)
            if self.zone_policy is not None
            else self.next_route_stopline(projection.progress_s)
        )
        stopline_distance = (
            max(0.0, next_stopline.progress_s - projection.progress_s)
            if next_stopline is not None
            else INF_DISTANCE
        )
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.progress_s = float(projection.progress_s)
        msg.current_zone = self.current_zone(
            projection.progress_s,
            distance_to_finish,
            distance_to_intersection,
        )
        msg.next_zone = self.next_zone(
            projection.progress_s,
            msg.current_zone,
            distance_to_finish,
            distance_to_intersection,
        )
        msg.next_maneuver = self.next_maneuver(projection.progress_s)
        if next_stopline is not None:
            msg.next_stop_line_id = next_stopline.landmark_id
            msg.next_traffic_light_ids = list(next_stopline.traffic_light_ids)
            msg.next_traffic_light_positions = [
                self.map_point_to_base(point_value, x, y, yaw)
                for point_value in next_stopline.traffic_light_positions
            ]
        msg.distance_to_stopline = float(stopline_distance)
        msg.distance_to_intersection = float(distance_to_intersection)
        msg.distance_to_finish = float(distance_to_finish)
        heading_error = normalize_angle(projection.yaw - yaw)
        initial_heading_valid = (
            self.last_progress_s is not None
            or abs(heading_error) <= self.initial_projection_max_heading_error_rad
        )
        msg.route_projection_valid = (
            projection.distance <= self.max_projection_distance_m
            and initial_heading_valid
        )
        msg.projection_confidence = float(
            max(0.0, min(1.0, 1.0 - projection.distance / self.max_projection_distance_m))
        )
        msg.cross_track_error = float(projection.cross_track_error)
        msg.heading_error = float(heading_error)
        msg.local_path_points = self.local_path_points(projection.progress_s, x, y, yaw)
        self.publisher.publish(msg)
        self.publish_route_state_markers(now.to_msg(), projection)
        if msg.route_projection_valid:
            self.last_progress_s = float(projection.progress_s)

    def lookup_vehicle_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.base_frame_id,
                RosTime(),
                timeout=Duration(seconds=0.03),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return self.pose_from_odometry_if_map()

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return float(translation.x), float(translation.y), yaw

    def pose_from_odometry_if_map(self) -> tuple[float, float, float] | None:
        if self.latest_odometry is None or self.latest_odometry.header.frame_id != self.map_frame_id:
            return None
        pose = self.latest_odometry.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        return float(pose.position.x), float(pose.position.y), yaw

    def publish_invalid_route(self, stamp, reason: str) -> None:
        msg = RouteContext()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame_id
        msg.current_zone = "UNKNOWN_ZONE"
        msg.next_zone = "NORMAL_ZONE"
        msg.next_maneuver = "UNKNOWN"
        msg.distance_to_stopline = INF_DISTANCE
        msg.distance_to_intersection = INF_DISTANCE
        msg.distance_to_finish = self.route.total_length
        msg.route_projection_valid = False
        msg.projection_confidence = 0.0
        self.get_logger().warn(f"route context invalid: {reason}", throttle_duration_sec=2.0)
        self.publisher.publish(msg)
        self.publish_route_state_markers(stamp, None)

    def project_route(self, x: float, y: float, yaw: float) -> Projection:
        if self.last_progress_s is None and self.should_snap_to_route_start(x, y, yaw):
            return self.route.project_prefix(x, y, self.start_snap_progress_m)

        if self.last_progress_s is not None:
            local_projection = self.route.project_near_progress(
                x,
                y,
                self.last_progress_s,
                self.progress_search_window_m,
            )
            if local_projection.distance <= self.max_projection_distance_m:
                return local_projection

        return self.route.project_with_heading(
            x,
            y,
            yaw,
            self.initial_projection_max_heading_error_rad,
            self.initial_projection_heading_weight_m,
        )

    def should_snap_to_route_start(self, x: float, y: float, yaw: float) -> bool:
        first = self.route.waypoints[0]
        second = self.route.waypoints[1]
        distance_to_start = math.hypot(x - first.x, y - first.y)
        route_start_yaw = math.atan2(second.y - first.y, second.x - first.x)
        heading_error = abs(normalize_angle(route_start_yaw - yaw))
        return (
            distance_to_start <= self.start_snap_distance_m
            and heading_error <= self.start_snap_heading_error_rad
        )

    def route_line_marker(
        self,
        stamp,
        namespace: str,
        marker_id: int,
        points: list[Point],
        width_m: float,
        rgba: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width_m
        marker.points = points
        set_color(marker, rgba)
        return marker

    def route_text_marker(
        self,
        stamp,
        namespace: str,
        marker_id: int,
        text: str,
        x: float,
        y: float,
        z: float,
        rgba: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = make_point(x, y, z)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.42
        marker.text = text
        set_color(marker, rgba)
        return marker

    def route_slice_points(self, start_s: float, end_s: float) -> list[Point]:
        start_s = max(0.0, min(self.route.total_length, start_s))
        end_s = max(start_s, min(self.route.total_length, end_s))
        start_x, start_y, start_z, _ = self.route.sample_at(start_s)
        points = [make_point(start_x, start_y, start_z + 0.19)]
        for waypoint, progress_s in zip(self.route.waypoints, self.route.cumulative_s):
            if start_s < progress_s < end_s:
                points.append(make_point(waypoint.x, waypoint.y, waypoint.z + 0.19))
        end_x, end_y, end_z, _ = self.route.sample_at(end_s)
        points.append(make_point(end_x, end_y, end_z + 0.19))
        return points

    @staticmethod
    def route_zone_color(zone: str) -> tuple[float, float, float, float]:
        stage_colors = {
            "Q_STAGE_1": (0.10, 0.78, 1.0, 0.88),
            "Q_STAGE_2": (1.0, 0.72, 0.10, 0.90),
            "Q_STAGE_3": (0.70, 0.32, 1.0, 0.88),
            "Q_STAGE_4": (0.18, 1.0, 0.35, 0.90),
            "Q_STAGE_5": (0.60, 0.47, 1.0, 0.90),
            "F_STAGE_1": (0.10, 0.78, 1.0, 0.88),
            "F_STAGE_2": (1.0, 0.72, 0.10, 0.90),
            "F_STAGE_3": (0.70, 0.32, 1.0, 0.88),
            "F_STAGE_4": (1.0, 0.34, 0.08, 0.90),
            "F_STAGE_5": (1.0, 0.12, 0.58, 0.90),
            "F_STAGE_6": (0.18, 1.0, 0.35, 0.90),
        }
        if zone in stage_colors:
            return stage_colors[zone]
        return {
            "STRAIGHT_ZONE": (0.10, 0.78, 1.0, 0.86),
            "CURVE_ZONE": (0.24, 0.52, 1.0, 0.86),
            "INTERSECTION_APPROACH_ZONE": (1.0, 0.72, 0.10, 0.88),
            "INTERSECTION_ZONE": (1.0, 0.34, 0.08, 0.90),
            "RIGHT_TURN_ZONE": (0.70, 0.32, 1.0, 0.88),
            "OBSTACLE_ZONE": (1.0, 0.12, 0.58, 0.90),
            "STOPLINE_ZONE": (1.0, 0.10, 0.10, 0.92),
            "FINISH_ZONE": (0.18, 1.0, 0.35, 0.92),
        }.get(zone, (0.72, 0.78, 0.86, 0.82))

    def publish_global_route_markers(self, stamp) -> None:
        markers = MarkerArray()
        route_points = [
            make_point(waypoint.x, waypoint.y, waypoint.z + 0.18)
            for waypoint in self.route.waypoints
        ]
        markers.markers.append(
            self.route_line_marker(
                stamp,
                "global_route_underlay",
                0,
                route_points,
                0.44,
                (0.02, 0.04, 0.06, 0.86),
            )
        )
        markers.markers.append(
            self.route_line_marker(
                stamp,
                "global_route_line",
                0,
                route_points,
                0.20,
                (0.05, 0.86, 1.0, 0.98),
            )
        )

        for index, span in enumerate(self.route.zone_spans):
            zone_points = self.route_slice_points(span.start_s, span.end_s)
            rgba = self.route_zone_color(span.zone)
            markers.markers.append(
                self.route_line_marker(
                    stamp,
                    f"route_zone_{span.zone.lower()}",
                    index,
                    zone_points,
                    0.58,
                    rgba,
                )
            )
            middle_s = (span.start_s + span.end_s) * 0.5
            x, y, z, _ = self.route.sample_at(middle_s)
            markers.markers.append(
                self.route_text_marker(
                    stamp,
                    "route_zone_labels",
                    index,
                    span.zone,
                    x,
                    y,
                    z + 0.85,
                    rgba,
                )
            )

        waypoints = Marker()
        waypoints.header.stamp = stamp
        waypoints.header.frame_id = self.map_frame_id
        waypoints.ns = "global_route_waypoints"
        waypoints.id = 0
        waypoints.type = Marker.SPHERE_LIST
        waypoints.action = Marker.ADD
        waypoints.pose.orientation.w = 1.0
        waypoints.scale.x = 0.26
        waypoints.scale.y = 0.26
        waypoints.scale.z = 0.26
        waypoints.points = [
            make_point(waypoint.x, waypoint.y, waypoint.z + 0.26)
            for waypoint in self.route.waypoints
        ]
        set_color(waypoints, (0.86, 0.94, 1.0, 0.76))
        markers.markers.append(waypoints)

        brake_waypoints = Marker()
        brake_waypoints.header.stamp = stamp
        brake_waypoints.header.frame_id = self.map_frame_id
        brake_waypoints.ns = "global_route_brake_waypoints"
        brake_waypoints.id = 0
        brake_waypoints.type = Marker.SPHERE_LIST
        brake_waypoints.action = Marker.ADD
        brake_waypoints.pose.orientation.w = 1.0
        brake_waypoints.scale.x = 0.72
        brake_waypoints.scale.y = 0.72
        brake_waypoints.scale.z = 0.72
        if self.zone_policy is not None:
            brake_waypoints.points = [
                make_point(
                    self.route.waypoints[event.brake_trigger_waypoint].x,
                    self.route.waypoints[event.brake_trigger_waypoint].y,
                    self.route.waypoints[event.brake_trigger_waypoint].z + 0.42,
                )
                for stage in self.zone_policy.stages
                for event in (stage.event,)
                if event is not None and event.brake_trigger_waypoint is not None
            ]
        set_color(brake_waypoints, (1.0, 0.10, 0.12, 0.98))
        markers.markers.append(brake_waypoints)

        tick_lines = Marker()
        tick_lines.header.stamp = stamp
        tick_lines.header.frame_id = self.map_frame_id
        tick_lines.ns = "global_route_distance_ticks"
        tick_lines.id = 0
        tick_lines.type = Marker.LINE_LIST
        tick_lines.action = Marker.ADD
        tick_lines.pose.orientation.w = 1.0
        tick_lines.scale.x = 0.10
        set_color(tick_lines, (0.75, 0.92, 1.0, 0.92))
        tick_s = 25.0
        tick_index = 0
        while tick_s < self.route.total_length:
            x, y, z, _ = self.route.sample_at(tick_s)
            yaw = self.route.heading_at(tick_s)
            normal_x = -math.sin(yaw)
            normal_y = math.cos(yaw)
            tick_lines.points.extend(
                (
                    make_point(x - normal_x * 0.65, y - normal_y * 0.65, z + 0.24),
                    make_point(x + normal_x * 0.65, y + normal_y * 0.65, z + 0.24),
                )
            )
            markers.markers.append(
                self.route_text_marker(
                    stamp,
                    "global_route_distance_labels",
                    tick_index,
                    f"{int(tick_s)} m",
                    x + normal_x * 1.0,
                    y + normal_y * 1.0,
                    z + 0.62,
                    (0.72, 0.90, 1.0, 0.94),
                )
            )
            tick_index += 1
            tick_s += 25.0
        markers.markers.append(tick_lines)

        for marker_id, (label, progress_s, rgba) in enumerate(
            (
                ("START", 0.0, (0.12, 1.0, 0.32, 0.98)),
                ("FINISH", self.route.total_length, (1.0, 0.18, 0.16, 0.98)),
            )
        ):
            x, y, z, _ = self.route.sample_at(progress_s)
            endpoint = Marker()
            endpoint.header.stamp = stamp
            endpoint.header.frame_id = self.map_frame_id
            endpoint.ns = "global_route_endpoints"
            endpoint.id = marker_id
            endpoint.type = Marker.SPHERE
            endpoint.action = Marker.ADD
            endpoint.pose.position = make_point(x, y, z + 0.48)
            endpoint.pose.orientation.w = 1.0
            endpoint.scale.x = 0.82
            endpoint.scale.y = 0.82
            endpoint.scale.z = 0.82
            set_color(endpoint, rgba)
            markers.markers.append(endpoint)
            markers.markers.append(
                self.route_text_marker(
                    stamp,
                    "global_route_endpoint_labels",
                    marker_id,
                    label,
                    x,
                    y,
                    z + 1.2,
                    rgba,
                )
            )

        for index, stopline in enumerate(self.stoplines):
            markers.markers.extend(self.make_stopline_markers(stamp, index, stopline))
        self.global_route_marker_pub.publish(markers)

    def publish_route_state_markers(
        self, stamp, projection: Projection | None
    ) -> None:
        markers = MarkerArray()
        if projection is None:
            for namespace, marker_id in (("global_route_projection", 0), ("global_route_progress", 0)):
                marker = Marker()
                marker.header.stamp = stamp
                marker.header.frame_id = self.map_frame_id
                marker.ns = namespace
                marker.id = marker_id
                marker.action = Marker.DELETE
                markers.markers.append(marker)
            self.route_state_marker_pub.publish(markers)
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame_id
        marker.ns = "global_route_projection"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = make_point(projection.x, projection.y, 0.52)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.74
        marker.scale.y = 0.74
        marker.scale.z = 0.74
        set_color(marker, (0.0, 0.95, 1.0, 0.98))
        markers.markers.append(marker)
        markers.markers.append(
            self.route_text_marker(
                stamp,
                "global_route_progress",
                0,
                f"s={projection.progress_s:.1f} m",
                projection.x,
                projection.y,
                1.15,
                (0.30, 1.0, 1.0, 0.98),
            )
        )
        self.route_state_marker_pub.publish(markers)

    def make_stopline_markers(
        self,
        stamp,
        index: int,
        stopline: RouteStopline,
    ) -> list[Marker]:
        x, y, z, _ = self.route.sample_at(stopline.progress_s)
        yaw = self.route.heading_at(stopline.progress_s)
        half_width = stopline.width_m * 0.5
        normal_x = -math.sin(yaw)
        normal_y = math.cos(yaw)
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.map_frame_id
        line.ns = "global_route_stoplines"
        line.id = index
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.34
        line.points = [
            make_point(x - normal_x * half_width, y - normal_y * half_width, z + 0.23),
            make_point(x + normal_x * half_width, y + normal_y * half_width, z + 0.23),
        ]
        set_color(line, (1.0, 0.12, 0.10, 0.98))
        label = self.route_text_marker(
            stamp,
            "global_route_stopline_labels",
            index,
            stopline.landmark_id,
            x,
            y,
            z + 0.82,
            (1.0, 0.34, 0.28, 0.98),
        )
        return [line, label]

    def local_path_points(self, progress_s: float, ego_x: float, ego_y: float, ego_yaw: float) -> list[Point]:
        points: list[Point] = []
        end_s = min(self.route.total_length, progress_s + self.lookahead_distance_m)
        sample_s = progress_s
        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)
        while sample_s <= end_s + 1e-6:
            x, y, z, _ = self.route.sample_at(sample_s)
            dx = x - ego_x
            dy = y - ego_y
            local_x = cos_yaw * dx + sin_yaw * dy
            local_y = -sin_yaw * dx + cos_yaw * dy
            if local_x >= -0.2:
                points.append(make_point(local_x, local_y, z))
            sample_s += self.local_path_spacing_m

        if len(points) < 2 and progress_s < self.route.total_length:
            x, y, z, _ = self.route.sample_at(min(self.route.total_length, progress_s + 2.0))
            dx = x - ego_x
            dy = y - ego_y
            points.append(make_point(cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy, z))
        return points

    def distance_to_next_zone(self, progress_s: float, zone_name: str) -> float:
        return self.route.distance_to_zone(progress_s, zone_name)

    def current_zone(
        self,
        progress_s: float,
        distance_to_finish: float,
        distance_to_intersection: float,
    ) -> str:
        if distance_to_finish <= self.finish_zone_distance_m:
            return "FINISH_ZONE"
        explicit_zone = self.route.zone_at(progress_s)
        if explicit_zone != "NORMAL_ZONE":
            return explicit_zone
        if self.route.zone_spans:
            return "NORMAL_ZONE"
        if distance_to_intersection <= self.intersection_zone_radius_m:
            return "INTERSECTION_ZONE"
        return "NORMAL_ZONE"

    def next_zone(
        self,
        progress_s: float,
        current_zone: str,
        distance_to_finish: float,
        distance_to_intersection: float,
    ) -> str:
        explicit_next = self.route.next_zone_after(progress_s)
        if explicit_next is not None:
            return explicit_next
        if current_zone != "NORMAL_ZONE":
            return current_zone
        if distance_to_intersection < distance_to_finish:
            return "INTERSECTION_ZONE"
        if distance_to_finish < INF_DISTANCE:
            return "FINISH_ZONE"
        return "NORMAL_ZONE"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RouteZoneManagerNode()
    spin_until_shutdown(node)


if __name__ == "__main__":
    main()
