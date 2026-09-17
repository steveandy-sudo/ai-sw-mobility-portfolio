"""Shared ROS adapter for Pure Pursuit and Stanley AEB tracking."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
from kaiev26_msgs.msg import ActuatorCommand, VehicleState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker

from .messages import predicted_path_marker, stamp_ns
from .pursuit import Pursuit, PursuitConfig
from .stanley import Stanley, StanleyConfig


class PursuitNode(Node):
    def __init__(self, node_name='cone_pursuit_node', controller_class=Pursuit,
                 config_class=PursuitConfig):
        super().__init__(node_name)
        defaults = dict(vars(config_class()), path_topic='/aeb/center_path',
                        state_topic='/vehicle/state', command_topic='', enabled=False,
                        control_hz=50.0, input_wall_timeout_s=0.75)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.control = controller_class(config_class(**{k: self.p[k] for k in vars(config_class())}))
        if self.p['control_hz'] <= 0 or self.p['input_wall_timeout_s'] <= 0:
            raise ValueError('Control rate and wall timeout must be positive')
        if self.p['command_topic'] in ('/vehicle/command', '/aeb/control_command'):
            raise ValueError('Use /planning/command through command_governor, or leave command_topic empty')
        self.enabled = self.p['enabled']
        self.path = self.state = None
        self.red_gate = None
        self.red_gate_wall = 0.0
        self.path_wall = self.state_wall = 0.0
        self.last_ns = None
        self.command_pub = self.create_publisher(ActuatorCommand, '/aeb/control_command', 10)
        self.drive_pub = (self.create_publisher(ActuatorCommand, self.p['command_topic'], 10)
                          if self.p['command_topic'] else None)
        self.status_pub = self.create_publisher(String, '/aeb/tracking_status', 10)
        self.target_pub = self.create_publisher(Marker, '/aeb/pursuit_target', 2)
        self.predicted_path_pub = self.create_publisher(Marker, '/aeb/predicted_path', 2)
        self.create_subscription(Path, self.p['path_topic'], self.on_path, 2)
        self.create_subscription(PoseArray, '/aeb/red_gate', self.on_red_gate, 2)
        self.create_subscription(VehicleState, self.p['state_topic'], self.on_state, qos_profile_sensor_data)
        self.create_service(SetBool, '/aeb/set_tracking_enabled', self.set_enabled)
        self.create_service(Trigger, '/aeb/reset_red_stop', self.reset_red_stop)
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(
            successful=False, reason='Restart to change configuration; use set_tracking_enabled service to start/stop'))
        self.create_timer(1 / self.p['control_hz'], self.tick)
        self.get_logger().info(f"{self.control.controller_name}: {self.p['target_speed_kph']} km/h; enabled={self.enabled}; "
                               f"vehicle output={self.p['command_topic'] or 'none (observation only)'}")

    def on_path(self, msg):
        self.path, self.path_wall = msg, time.monotonic()

    def on_state(self, msg):
        self.state, self.state_wall = msg, time.monotonic()

    def on_red_gate(self, msg):
        self.red_gate, self.red_gate_wall = msg, time.monotonic()

    def reset_red_stop(self, req, res):
        age = ((self.get_clock().now().nanoseconds - stamp_ns(self.state.header)) / 1e9
               if self.state else 1e6)
        if (self.enabled or not self.state or not -0.05 <= age <= self.p['state_timeout_s']
                or time.monotonic() - self.state_wall > self.p['input_wall_timeout_s']
                or not abs(self.state.speed_mps) < .05):
            res.success, res.message = False, 'Disable tracking and stop with fresh vehicle state before reset'
        else:
            self.control.red.reset()
            self.red_gate = None
            res.success, res.message = True, 'Red stop reset; tracking remains disabled'
        return res

    def set_enabled(self, req, res):
        if req.data and self.control.red.latched:
            res.success, res.message = False, 'Red stop is latched; stop, disable tracking, then reset_red_stop'
            return res
        if req.data and not self.enabled:
            self.control.red.reset()
            self.red_gate = None
        self.enabled = req.data
        self.control.speed_command = 0.0
        res.success, res.message = True, ('enabled' if req.data else 'disabled: braking')
        if not req.data:
            self.tick()
        return res

    def tick(self):
        now = self.get_clock().now()
        ns = now.nanoseconds
        if self.enabled and self.last_ns == ns:
            return  # A service and timer may run at the same simulated instant.
        dt = (ns - self.last_ns) / 1e9 if self.last_ns is not None else 1 / self.p['control_hz']
        if self.last_ns is not None and ns < self.last_ns:
            self.path = self.state = None
            self.red_gate = None
            self.control = type(self.control)(self.control.cfg)
            self.enabled = False
        self.last_ns = ns
        path_age = (ns - stamp_ns(self.path.header)) / 1e9 if self.path else 1e6
        state_age = (ns - stamp_ns(self.state.header)) / 1e9 if self.state else 1e6
        wall = time.monotonic()
        if wall - self.path_wall > self.p['input_wall_timeout_s']:
            path_age = 1e6
        if wall - self.state_wall > self.p['input_wall_timeout_s']:
            state_age = 1e6
        speed = float(self.state.speed_mps) if self.state else 0.0
        gate = self.red_gate
        gate_age = (ns - stamp_ns(gate.header)) / 1e9 if gate else 1e6
        if wall - self.red_gate_wall > self.p['input_wall_timeout_s']:
            gate_age = 1e6
        result = self.control.step(
            [(p.pose.position.x, p.pose.position.y) for p in self.path.poses] if self.path else [],
            speed, float(self.state.steering_rad) if self.state else 0.0,
            path_age, state_age, dt, self.enabled,
            self.path.header.frame_id if self.path else 'base_link',
            bool(self.state.estop_active) if self.state else False,
            self.state.gear != VehicleState.GEAR_FORWARD if self.state else False,
            red_gate=([(p.position.x, p.position.y) for p in gate.poses]
                      if gate and gate.header.frame_id == 'base_link' else []),
            red_gate_stamp=stamp_ns(gate.header) if gate else None, red_gate_age=gate_age)
        msg = ActuatorCommand()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), 'base_link'
        for key in ('speed_target_mps', 'steering_target_rad', 'brake_engage'):
            setattr(msg, key, result[key])
        self.command_pub.publish(msg)
        if self.drive_pub:
            self.drive_pub.publish(msg)
        status = dict(result, stamp_ns=ns, speed_mps=speed, target_speed_kph=self.p['target_speed_kph'],
                      path_age_s=path_age, state_age_s=state_age, enabled=self.enabled,
                      command_topic=self.p['command_topic'])
        self.status_pub.publish(String(data=json.dumps(status)))
        marker = Marker()
        marker.header = msg.header
        marker.ns, marker.id, marker.type = self.control.controller_name, 0, Marker.SPHERE
        marker.action = Marker.ADD if result['target_xy'] else Marker.DELETE
        marker.pose.orientation.w = 1.0
        if result['target_xy']:
            marker.pose.position.x, marker.pose.position.y = result['target_xy']
        marker.scale.x = marker.scale.y = marker.scale.z = 0.4
        marker.color.r = marker.color.g = marker.color.a = 1.0
        self.target_pub.publish(marker)
        self.predicted_path_pub.publish(predicted_path_marker(
            result, self.control.cfg.wheelbase_m, msg.header))


class StanleyNode(PursuitNode):
    def __init__(self):
        super().__init__('cone_stanley_node', Stanley, StanleyConfig)


def _run(args, node_class):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.enabled = False
            for _ in range(3):
                node.tick()
                time.sleep(0.02)
        node.destroy_node()
        rclpy.try_shutdown()


def main_pursuit(args=None):
    _run(args, PursuitNode)


def main_stanley(args=None):
    _run(args, StanleyNode)
