from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time as RosTime


INF_DISTANCE = 1.0e6


def spin_until_shutdown(node: Node) -> None:
    """Spin a node and close it once even when launch already shut ROS down."""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


@dataclass(frozen=True)
class DecisionCommand:
    safety_state: str = "NORMAL"
    active_behavior: str = "LANE_FOLLOW"
    fsm_state: str = "LANE_GOOD"
    selected_reason: str = "default"
    need_stop: bool = False
    target_speed_limit_mps: float = 0.0
    path_request: str = "CENTERLINE"
    stop_target_distance: float = INF_DISTANCE
    constraints: list[str] = field(default_factory=list)
    risk_level: str = "LOW"


@dataclass(frozen=True)
class ScenarioChoice:
    behavior: str
    reason: str
    stage_id: str = ""
    event_id: str = ""
    stop_line_id: str = ""
    hold_sec: float = 0.0
    arrow_maneuver: str = ""
    brake_trigger_waypoint: int | None = None
    brake_trigger_route_s: float | None = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def distance_2d(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def polyline_length(points: Iterable[Point]) -> float:
    items = list(points)
    if len(items) < 2:
        return 0.0
    return sum(distance_2d(start, end) for start, end in zip(items, items[1:]))


def min_forward_x(points: Iterable[Point], max_distance: float = INF_DISTANCE) -> float:
    values = [point.x for point in points if 0.0 <= point.x <= max_distance]
    if not values:
        return INF_DISTANCE
    return min(values)


def stamp_age_sec(now: RosTime, stamp: Time) -> float:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return INF_DISTANCE
    age = now - RosTime.from_msg(stamp)
    return max(0.0, age.nanoseconds * 1.0e-9)


def normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> tuple[float, float, float, float] | None:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-6:
        return None
    return tuple(value / norm for value in values)


def quaternion_yaw(quaternion) -> float | None:
    normalized = normalize_quaternion(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    if normalized is None:
        return None
    x, y, z, w = normalized
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def require_finite_parameter(
    name: str,
    value: float,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    minimum_exclusive: bool = False,
) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"parameter [{name}] must be finite")
    if minimum is not None:
        below_minimum = converted <= minimum if minimum_exclusive else converted < minimum
        if below_minimum:
            relation = ">" if minimum_exclusive else ">="
            raise ValueError(f"parameter [{name}] must be {relation} {minimum}")
    if maximum is not None and converted > maximum:
        raise ValueError(f"parameter [{name}] must be <= {maximum}")
    return converted


def traffic_state_name(state: int) -> str:
    names = {
        0: "UNKNOWN",
        1: "RED",
        2: "YELLOW",
        3: "GREEN",
        4: "ARROW",
    }
    return names.get(int(state), "UNKNOWN")
