#!/usr/bin/env python3
"""전역경로의 로컬 구간을 상수속도 경로추종 입력으로 바꾼다."""
from __future__ import annotations

import math

from kaiev26_decision.common import require_finite_parameter, spin_until_shutdown
from kaiev26_msgs.msg import Centerline, RouteContext, TargetSpeed
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ConstantRoutePlanNode(Node):
    """Publish a valid local route with one fixed target speed."""

    def __init__(self) -> None:
        super().__init__('kaiev26_constant_route_plan')
        self.declare_parameter('route_context_topic', '/planning/route_context')
        self.declare_parameter('target_path_topic', '/planning/target_path')
        self.declare_parameter('target_speed_topic', '/planning/target_speed')
        self.declare_parameter('constant_speed_mps', 1.0)
        self.declare_parameter('require_readiness', False)
        self.declare_parameter('readiness_topic', '/planning/route_ready')

        self.constant_speed_mps = require_finite_parameter(
            'constant_speed_mps',
            self.get_parameter('constant_speed_mps').value,
            minimum=0.0,
        )
        self.require_readiness = bool(
            self.get_parameter('require_readiness').value
        )
        self.route_ready = not self.require_readiness

        self.target_path_pub = self.create_publisher(
            Centerline,
            str(self.get_parameter('target_path_topic').value),
            10,
        )
        self.target_speed_pub = self.create_publisher(
            TargetSpeed,
            str(self.get_parameter('target_speed_topic').value),
            10,
        )
        self.create_subscription(
            RouteContext,
            str(self.get_parameter('route_context_topic').value),
            self.on_route_context,
            10,
        )
        if self.require_readiness:
            self.create_subscription(
                Bool,
                str(self.get_parameter('readiness_topic').value),
                self.on_readiness,
                10,
            )

    def on_readiness(self, message: Bool) -> None:
        self.route_ready = bool(message.data)

    def on_route_context(self, route: RouteContext) -> None:
        usable_route = (
            self.route_ready
            and bool(route.route_projection_valid)
            and len(route.local_path_points) >= 2
            and math.isfinite(route.distance_to_finish)
            and route.distance_to_finish >= 0.0
        )

        path = Centerline()
        path.header = route.header
        path.points = list(route.local_path_points) if usable_route else []
        path.confidence = float(route.projection_confidence) if usable_route else 0.0
        path.source = 'global_route_constant_speed'

        speed = TargetSpeed()
        speed.header = route.header
        speed.target_speed_mps = self.constant_speed_mps if usable_route else 0.0
        speed.speed_limit_mps = self.constant_speed_mps
        speed.need_stop = True
        speed.stop_target_distance = float(route.distance_to_finish) if usable_route else 0.0
        speed.source_behavior = 'GLOBAL_ROUTE'

        self.target_path_pub.publish(path)
        self.target_speed_pub.publish(speed)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ConstantRoutePlanNode()
    spin_until_shutdown(node)


if __name__ == '__main__':
    main()
