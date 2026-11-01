from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from kaiev26_msgs.msg import TargetSpeed
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
class StoppingModel:
    """Service-brake stopping distance used by the stop-point machine.

    d(v) = v * t_d + v**2 / (2 * a_b). Defaults are the 2026-10-11 vehicle brake fit;
    the node passes the configured values. Distances are measured at the front bumper.
    """

    brake_delay_sec: float = 0.70
    brake_decel_mps2: float = 3.9

    def distance_m(self, speed_mps: float) -> float:
        speed = abs(float(speed_mps))
        if not math.isfinite(speed):
            return INF_DISTANCE
        return speed * self.brake_delay_sec + speed * speed / (2.0 * self.brake_decel_mps2)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def distance_2d(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def polyline_length(points: Iterable[Point]) -> float:
    items = list(points)
    if len(items) < 2:
        return 0.0
    return sum(distance_2d(start, end) for start, end in zip(items, items[1:]))


def stamp_age_sec(now: RosTime, stamp: Time) -> float:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return INF_DISTANCE
    age = now - RosTime.from_msg(stamp)
    return max(0.0, age.nanoseconds * 1.0e-9)


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


def stop_command(target: TargetSpeed, distance_m: float = 0.0) -> TargetSpeed:
    """The only producer of a stop in TargetSpeed: need_stop, target 0, distance (0 = brake now)."""
    target.need_stop = True
    target.target_speed_mps = 0.0
    distance = float(distance_m)
    target.stop_target_distance = max(0.0, distance) if math.isfinite(distance) else 0.0
    return target
