"""Shared ROS adapters for fusion and camera-only cone perception."""
from collections import deque
from dataclasses import fields
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, CompressedImage, CameraInfo
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from .core import Config, transform_points, fuse_and_plan
from .detector import Detector, overlay
from .messages import (stamp_ns, camera_info, cloud_xyz, transform_matrix,
                       output_messages, empty_result, image_message, cloud_message)
from .yolo_core import overlay_yolo_only, yolo_only_plan


class ConePathNode(Node):
    def __init__(self):
        super().__init__('cone_path_node')
        defaults = {
            'model_path': '', 'device': 'auto', 'image_size': 800, 'confidence': .5,
            'output_frame': 'base_link', 'points_topic': '/ouster/points',
            'left_image_topic': '/perception/camera/left/source/image_raw/compressed',
            'right_image_topic': '/perception/camera/right/source/image_raw/compressed',
            'left_info_topic': '/perception/camera/left/source/camera_info',
            'right_info_topic': '/perception/camera/right/source/camera_info',
            'processing_hz': 10.0, 'max_camera_dt_s': .05,
            'cloud_time_offset_s': .025, 'stale_timeout_s': .5, 'publish_debug': True,
            **vars(Config()),
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if (self.p['processing_hz'] <= 0 or self.p['max_camera_dt_s'] <= 0
                or self.p['stale_timeout_s'] <= 0):
            raise ValueError('Processing frequency and time limits must be positive')
        self.cfg = Config(**{f.name: self.p[f.name] for f in fields(Config)})
        self.detector = Detector(self.p['model_path'], self.p['device'], self.p['image_size'], self.p['confidence'])
        self.lock = threading.Lock()
        self.images = {s: deque(maxlen=60) for s in ('left', 'right')}
        self.infos = {}
        self.cloud = None
        self.last_processed = None
        self.last_reason = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        group = ReentrantCallbackGroup()
        self.subscriptions_kept = []
        for side in self.images:
            self.subscriptions_kept.append(self.create_subscription(
                CompressedImage, self.p[side + '_image_topic'],
                lambda msg, s=side: self._image(s, msg), qos_profile_sensor_data, callback_group=group))
            self.subscriptions_kept.append(self.create_subscription(
                CameraInfo, self.p[side + '_info_topic'],
                lambda msg, s=side: self._info(s, msg), qos_profile_sensor_data, callback_group=group))
        self.subscriptions_kept.append(self.create_subscription(
            PointCloud2, self.p['points_topic'], self._cloud, qos_profile_sensor_data, callback_group=group))
        self.publishers_out = {topic: self.create_publisher(cls, topic, 2) for topic, cls in {
            '/aeb/center_path': Path, '/aeb/cones': MarkerArray, '/aeb/boundaries': MarkerArray,
            '/aeb/status': String, '/aeb/points': PointCloud2,
            '/aeb/red_gate': PoseArray,
            '/aeb/left/image/compressed': CompressedImage, '/aeb/right/image/compressed': CompressedImage,
        }.items()}
        self.timer = self.create_timer(1 / self.p['processing_hz'], self._tick,
                                      callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'Cone path node ready; device={self.detector.device}; outputs=/aeb/*')

    def _image(self, side, msg):
        with self.lock:
            self.images[side].append(msg)

    def _info(self, side, msg):
        with self.lock:
            self.infos[side] = msg

    def _cloud(self, msg):
        with self.lock:
            if self.cloud is not None and stamp_ns(msg.header) < stamp_ns(self.cloud.header) - 500_000_000:
                # Bag loop / simulated clock reset: do not match frames from the previous run.
                for queue in self.images.values():
                    queue.clear()
                self.last_processed = None
            self.cloud = msg

    def _emit(self, result, stamp, extra=None):
        for topic, msg in output_messages(result, stamp, self.p['output_frame'], extra).items():
            self.publishers_out[topic].publish(msg)
        reason = result['plan']['reason']
        if reason != self.last_reason:
            self.get_logger().info(f'Path: {reason}')
            self.last_reason = reason

    def _tick(self):
        begin = time.monotonic()
        with self.lock:
            cloud = self.cloud
            infos = self.infos.copy()
            images = {s: list(q) for s, q in self.images.items()}
        now = self.get_clock().now().nanoseconds
        if cloud is None:
            self._emit(empty_result('waiting_for_points'), now)
            return
        stamp = stamp_ns(cloud.header) + round(self.p['cloud_time_offset_s'] * 1e9)
        if now - stamp > self.p['stale_timeout_s'] * 1e9 or stamp > now + self.p['stale_timeout_s'] * 1e9:
            self._emit(empty_result('stale_points_or_clock_mismatch'), now)
            return
        if stamp == self.last_processed:
            return
        selected = {}
        for side in images:
            if side not in infos or not images[side]:
                self._emit(empty_result(f'waiting_for_{side}_camera'), stamp)
                return
            selected[side] = min(images[side], key=lambda m: abs(stamp_ns(m.header) - stamp))
            if abs(stamp_ns(selected[side].header) - stamp) > self.p['max_camera_dt_s'] * 1e9:
                self._emit(empty_result(f'{side}_camera_unsynchronized'), stamp)
                return
        try:
            transforms = {}
            for target, source in [(self.p['output_frame'], cloud.header.frame_id)] + [
                    (infos[s].header.frame_id, self.p['output_frame']) for s in images]:
                tf = self.buffer.lookup_transform(target, source, Time(nanoseconds=stamp))
                transforms[target, source] = transform_matrix(tf.transform)
            camera_data, decoded = {}, {}
            for side, msg in selected.items():
                info = camera_info(infos[side])
                if msg.header.frame_id != info['frame']:
                    raise ValueError(f'{side} image and CameraInfo frames differ')
                im = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if im is None or (im.shape[1], im.shape[0]) != (info['width'], info['height']):
                    raise ValueError(f'{side} image dimensions do not match CameraInfo')
                decoded[side] = im
                camera_data[side] = {'info': info, 'transform': transforms[info['frame'], self.p['output_frame']]}
            detections, inference_ms = self.detector.predict(decoded)
            for side in camera_data:
                camera_data[side]['detections'] = detections[side]
            xyz = transform_points(cloud_xyz(cloud), transforms[self.p['output_frame'], cloud.header.frame_id])
            result = fuse_and_plan(xyz, camera_data, self.cfg)
            if self.get_clock().now().nanoseconds - stamp > self.p['stale_timeout_s'] * 1e9:
                self._emit(empty_result('processing_too_old'), self.get_clock().now().nanoseconds)
                self.last_processed = stamp
                return
            extra = {'inference_ms': inference_ms, 'processing_ms': (time.monotonic() - begin) * 1000,
                     'camera_dt_ms': {s: (stamp_ns(m.header) - stamp) / 1e6 for s, m in selected.items()}}
            self._emit(result, stamp, extra)
            if self.p['publish_debug']:
                self.publishers_out['/aeb/points'].publish(cloud_message(result['xyz'], stamp, self.p['output_frame']))
                for side, im in decoded.items():
                    ok, encoded = cv2.imencode('.jpg', overlay(im, side, detections[side], result))
                    if ok:
                        self.publishers_out[f'/aeb/{side}/image/compressed'].publish(
                            image_message(encoded, stamp, infos[side].header.frame_id))
            self.last_processed = stamp
        except (TransformException, ValueError, cv2.error) as exc:
            self._emit(empty_result(f'input_error: {exc}'), stamp)


class YoloConePathNode(Node):
    def __init__(self):
        super().__init__('cone_yolo_path_node')
        defaults = {
            'model_path': '', 'device': 'auto', 'image_size': 800, 'confidence': .5,
            'output_frame': 'base_link',
            'left_image_topic': '/perception/camera/left/source/image_raw/compressed',
            'right_image_topic': '/perception/camera/right/source/image_raw/compressed',
            'left_info_topic': '/perception/camera/left/source/camera_info',
            'right_info_topic': '/perception/camera/right/source/camera_info',
            'processing_hz': 10.0, 'max_camera_dt_s': .05,
            'stale_timeout_s': .5, 'publish_debug': True,
            'min_bbox_height_px': 8.0, 'min_bbox_width_px': 3.0,
            'bbox_bottom_margin_px': 2.0, 'bbox_bottom_offset_px': 0.0,
            'duplicate_merge_radius_m': .75,
            **vars(Config()),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if min(self.p['processing_hz'], self.p['max_camera_dt_s'],
               self.p['stale_timeout_s']) <= 0:
            raise ValueError('Processing frequency and time limits must be positive')
        self.cfg = Config(**{item.name: self.p[item.name] for item in fields(Config)})
        self.detector = Detector(
            self.p['model_path'], self.p['device'],
            self.p['image_size'], self.p['confidence'])
        self.lock = threading.Lock()
        self.images = {side: deque(maxlen=60) for side in ('left', 'right')}
        self.infos = {}
        self.last_processed = None
        self.last_reason = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        group = ReentrantCallbackGroup()
        self.subscriptions_kept = []
        for side in self.images:
            self.subscriptions_kept.append(self.create_subscription(
                CompressedImage, self.p[side + '_image_topic'],
                lambda msg, name=side: self._image(name, msg),
                qos_profile_sensor_data, callback_group=group))
            self.subscriptions_kept.append(self.create_subscription(
                CameraInfo, self.p[side + '_info_topic'],
                lambda msg, name=side: self._info(name, msg),
                qos_profile_sensor_data, callback_group=group))
        self.publishers_out = {
            topic: self.create_publisher(message_type, topic, 2)
            for topic, message_type in {
                '/aeb/center_path': Path,
                '/aeb/cones': MarkerArray,
                '/aeb/boundaries': MarkerArray,
                '/aeb/status': String,
                '/aeb/red_gate': PoseArray,
                '/aeb/left/image/compressed': CompressedImage,
                '/aeb/right/image/compressed': CompressedImage,
            }.items()
        }
        self.timer = self.create_timer(
            1 / self.p['processing_hz'], self._tick,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f'YOLO-only cone path ready; device={self.detector.device}; no LiDAR input')

    def _image(self, side, msg):
        with self.lock:
            queue = self.images[side]
            if (queue and stamp_ns(msg.header) <
                    stamp_ns(queue[-1].header) - 500_000_000):
                for rows in self.images.values():
                    rows.clear()
                self.last_processed = None
            queue.append(msg)

    def _info(self, side, msg):
        with self.lock:
            self.infos[side] = msg

    def _emit(self, result, stamp, extra=None):
        for topic, message in output_messages(
                result, stamp, self.p['output_frame'], extra).items():
            self.publishers_out[topic].publish(message)
        reason = result['plan']['reason']
        if reason != self.last_reason:
            self.get_logger().info(f'YOLO-only path: {reason}')
            self.last_reason = reason

    def _tick(self):
        begin = time.monotonic()
        with self.lock:
            infos = self.infos.copy()
            images = {side: list(queue) for side, queue in self.images.items()}
        if any(side not in infos or not images[side] for side in images):
            self._emit(
                empty_result('waiting_for_cameras'),
                self.get_clock().now().nanoseconds,
                {'perception_mode': 'yolo_only'})
            return
        reference = min(stamp_ns(rows[-1].header) for rows in images.values())
        selected = {
            side: min(rows, key=lambda msg: abs(stamp_ns(msg.header) - reference))
            for side, rows in images.items()
        }
        stamps = {side: stamp_ns(msg.header) for side, msg in selected.items()}
        stamp = round(sum(stamps.values()) / len(stamps))
        now = self.get_clock().now().nanoseconds
        if (max(stamps.values()) - min(stamps.values()) >
                self.p['max_camera_dt_s'] * 1e9):
            self._emit(
                empty_result('camera_unsynchronized'), stamp,
                {'perception_mode': 'yolo_only'})
            return
        if abs(now - stamp) > self.p['stale_timeout_s'] * 1e9:
            self._emit(
                empty_result('stale_cameras_or_clock_mismatch'), now,
                {'perception_mode': 'yolo_only'})
            return
        if stamp == self.last_processed:
            return
        try:
            camera_data, decoded = {}, {}
            for side, msg in selected.items():
                info = camera_info(infos[side])
                if msg.header.frame_id != info['frame']:
                    raise ValueError(f'{side} image and CameraInfo frames differ')
                image = cv2.imdecode(
                    np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if image is None or (image.shape[1], image.shape[0]) != (
                        info['width'], info['height']):
                    raise ValueError(
                        f'{side} image dimensions do not match CameraInfo')
                transform = self.buffer.lookup_transform(
                    self.p['output_frame'], info['frame'],
                    Time(nanoseconds=stamp))
                decoded[side] = image
                camera_data[side] = {
                    'info': info,
                    'transform': transform_matrix(transform.transform),
                }
            detections, inference_ms = self.detector.predict(decoded)
            for side in camera_data:
                camera_data[side]['detections'] = detections[side]
            result = yolo_only_plan(
                camera_data, self.cfg, self.p['min_bbox_height_px'],
                self.p['min_bbox_width_px'], self.p['bbox_bottom_margin_px'],
                self.p['bbox_bottom_offset_px'],
                self.p['duplicate_merge_radius_m'])
            if (self.get_clock().now().nanoseconds - stamp >
                    self.p['stale_timeout_s'] * 1e9):
                self._emit(
                    empty_result('processing_too_old'),
                    self.get_clock().now().nanoseconds,
                    {'perception_mode': 'yolo_only'})
                self.last_processed = stamp
                return
            extra = {
                'perception_mode': 'yolo_only',
                'distance_method': 'bbox_bottom_ground_intersection',
                'inference_ms': inference_ms,
                'processing_ms': (time.monotonic() - begin) * 1000,
                'camera_dt_ms': {
                    side: (value - stamp) / 1e6
                    for side, value in stamps.items()
                },
            }
            self._emit(result, stamp, extra)
            if self.p['publish_debug']:
                for side, image in decoded.items():
                    debug = overlay_yolo_only(
                        image, side, detections[side], result)
                    ok, encoded = cv2.imencode('.jpg', debug)
                    if ok:
                        self.publishers_out[
                            f'/aeb/{side}/image/compressed'].publish(
                                image_message(
                                    encoded, stamp,
                                    infos[side].header.frame_id))
            self.last_processed = stamp
        except (TransformException, ValueError, cv2.error) as error:
            self._emit(
                empty_result(f'input_error: {error}'), stamp,
                {'perception_mode': 'yolo_only'})


def _run(node_class, args=None):
    rclpy.init(args=args)
    node = node_class()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def main_fusion(args=None):
    _run(ConePathNode, args)


def main_yolo(args=None):
    _run(YoloConePathNode, args)
