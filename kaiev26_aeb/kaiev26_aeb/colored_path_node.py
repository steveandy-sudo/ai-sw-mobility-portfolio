"""Adapt cone_perception Detection3DArray to the shared AEB path contract."""

from dataclasses import fields
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray

from .colored_corridor import COLOR_CLASSES, plan_colored_corridor
from .core import Config
from .messages import empty_result, output_messages, stamp_ns


class ColoredConePathNode(Node):
    def __init__(self):
        super().__init__('cone_colored_path_node')
        defaults = dict(
            detection_topic='/perception/cone/detections', output_frame='base_link',
            stale_timeout_s=0.5, min_confidence=0.5, processing_hz=10.0,
            **vars(Config()),
        )
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        params = {name: self.get_parameter(name).value for name in defaults}
        if min(params['stale_timeout_s'], params['processing_hz']) <= 0:
            raise ValueError('Processing rate and stale timeout must be positive')
        if not 0 <= params['min_confidence'] <= 1:
            raise ValueError('min_confidence must be in [0, 1]')
        self.cfg = Config(**{item.name: params[item.name] for item in fields(Config)})
        self.output_frame = params['output_frame']
        self.stale_timeout_s = params['stale_timeout_s']
        self.min_confidence = params['min_confidence']
        self.latest = None
        self.last_stamp = None
        self.last_reason = None
        self.last_arrival = 0.0
        self.create_subscription(Detection3DArray, params['detection_topic'],
                                 self.on_detections, qos_profile_sensor_data)
        self.publishers_out = {
            topic: self.create_publisher(message_type, topic, 2)
            for topic, message_type in {
                '/aeb/center_path': Path,
                '/aeb/cones': MarkerArray,
                '/aeb/boundaries': MarkerArray,
                '/aeb/status': String,
                '/aeb/red_gate': PoseArray,
            }.items()
        }
        self.create_timer(1.0 / params['processing_hz'], self.tick)

    def on_detections(self, message):
        self.latest = message
        self.last_arrival = time.monotonic()

    def emit(self, result, stamp, extra=None):
        for topic, message in output_messages(result, stamp, self.output_frame, extra).items():
            self.publishers_out[topic].publish(message)
        reason = result['plan']['reason']
        if reason != self.last_reason:
            self.get_logger().info(f'Colored cone path: {reason}')
            self.last_reason = reason

    def tick(self):
        now = self.get_clock().now().nanoseconds
        source = self.latest
        if source is None:
            self.emit(empty_result('waiting_for_cone_detections'), now)
            return
        stamp = stamp_ns(source.header)
        if (source.header.frame_id != self.output_frame
                or abs(now - stamp) > self.stale_timeout_s * 1e9
                or time.monotonic() - self.last_arrival > self.stale_timeout_s):
            self.emit(empty_result('stale_or_wrong_frame_detections'), now)
            return
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        cones = []
        for detection in source.detections:
            if not detection.results:
                continue
            hypothesis = max(detection.results, key=lambda item: item.hypothesis.score).hypothesis
            color = COLOR_CLASSES.get(hypothesis.class_id.strip().lower())
            xyz = np.array([detection.bbox.center.position.x,
                            detection.bbox.center.position.y,
                            detection.bbox.center.position.z], dtype=float)
            size = np.array([detection.bbox.size.x, detection.bbox.size.y], dtype=float)
            if (color is None or not math.isfinite(hypothesis.score)
                    or hypothesis.score < self.min_confidence
                    or not np.isfinite(xyz).all() or not np.isfinite(size).all()
                    or np.any(size < 0) or not self.cfg.roi_x_min < xyz[0] < self.cfg.roi_x_max
                    or abs(xyz[1]) >= self.cfg.roi_y_abs):
                continue
            cones.append(dict(xyz=xyz, color=color, score=float(hypothesis.score),
                              size_xy=size, indices=np.empty(0, dtype=int),
                              matches=[], side='unassigned'))
        plan = plan_colored_corridor(cones, self.cfg)
        self.emit(dict(cones=cones, plane=None, plan=plan), stamp, {
            'perception_mode': 'cone_perception',
            'input_detections': len(source.detections),
        })


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ColoredConePathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
