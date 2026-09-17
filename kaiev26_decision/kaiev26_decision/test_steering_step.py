"""Publish configurable open-loop steering steps for field actuator data."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math

from kaiev26_msgs.msg import ActuatorCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


RIGHT = "R"
LEFT = "L"
CENTER = "CENTER"


@dataclass(frozen=True)
class SteeringStepSetpoint:
    steering_deg: float
    phase: str
    pattern_index: int
    pattern_cycle: int
    elapsed_s: float


def normalize_step_pattern(value: str) -> str:
    pattern = "".join(value.upper().split())
    if not pattern or any(direction not in {RIGHT, LEFT} for direction in pattern):
        raise ValueError("steering_pattern must contain only R and L")
    return pattern


def steering_step_setpoint(
    elapsed_s: float,
    amplitude_deg: float,
    frequency_hz: float,
    pattern: str,
    initial_straight_s: float,
) -> SteeringStepSetpoint:
    """Return one repeated direction pulse followed by a center-return pulse."""
    pattern = normalize_step_pattern(pattern)
    elapsed = max(0.0, float(elapsed_s))
    initial = max(0.0, float(initial_straight_s))
    if elapsed < initial:
        return SteeringStepSetpoint(0.0, "STRAIGHT", 0, 0, elapsed)

    frequency = float(frequency_hz)
    if not math.isfinite(frequency) or frequency <= 0.0:
        raise ValueError("steering_frequency_hz must be positive")
    period_s = 1.0 / frequency
    pattern_elapsed = elapsed - initial
    pulse_number = int(pattern_elapsed / period_s)
    pattern_index = pulse_number % len(pattern)
    pattern_cycle = pulse_number // len(pattern)
    within_pulse_s = pattern_elapsed - pulse_number * period_s
    direction = pattern[pattern_index]
    active = within_pulse_s < 0.5 * period_s
    steering_deg = 0.0
    if active:
        steering_deg = abs(float(amplitude_deg)) * (1.0 if direction == LEFT else -1.0)
    return SteeringStepSetpoint(
        steering_deg,
        direction if active else CENTER,
        pattern_index,
        pattern_cycle,
        elapsed,
    )


class SteeringStepTestNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_steering_step_test")
        self.declare_parameter("command_topic", "/planning/command")
        self.declare_parameter("run_enable_topic", "/decision/test_run_active")
        self.declare_parameter("status_topic", "/decision/steering_step_state")
        self.declare_parameter("constant_speed_mps", 1.0)
        self.declare_parameter("steering_amplitude_deg", 3.0)
        self.declare_parameter("steering_frequency_hz", 0.25)
        self.declare_parameter("steering_pattern", "RLR")
        self.declare_parameter("initial_straight_s", 3.0)
        self.declare_parameter("wait_for_run_enable", False)
        self.declare_parameter("command_rate_hz", 50.0)

        self.constant_speed_mps = max(
            0.0, float(self.get_parameter("constant_speed_mps").value)
        )
        self.amplitude_deg = abs(
            float(self.get_parameter("steering_amplitude_deg").value)
        )
        self.frequency_hz = float(
            self.get_parameter("steering_frequency_hz").value
        )
        self.pattern = normalize_step_pattern(
            str(self.get_parameter("steering_pattern").value)
        )
        self.initial_straight_s = max(
            0.0, float(self.get_parameter("initial_straight_s").value)
        )
        self.wait_for_run_enable = bool(
            self.get_parameter("wait_for_run_enable").value
        )
        self.run_enabled = not self.wait_for_run_enable
        self.started_at_ns: int | None = None

        if not 0.0 < self.frequency_hz <= 10.0:
            raise ValueError("steering_frequency_hz must be in (0, 10] Hz")
        if not 0.0 <= self.amplitude_deg <= 25.0:
            raise ValueError("steering_amplitude_deg must be in [0, 25] deg")

        self.command_pub = self.create_publisher(
            ActuatorCommand,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("run_enable_topic").value),
            self.on_run_enable,
            10,
        )
        command_rate_hz = max(
            1.0, float(self.get_parameter("command_rate_hz").value)
        )
        self.create_timer(1.0 / command_rate_hz, self.on_timer)
        self.get_logger().info(
            "steering step ready: "
            f"speed={self.constant_speed_mps:g} m/s, amplitude={self.amplitude_deg:g} deg, "
            f"frequency={self.frequency_hz:g} Hz, pattern={self.pattern}"
        )

    def on_run_enable(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled and not self.run_enabled:
            self.started_at_ns = None
        elif not enabled:
            self.started_at_ns = None
        self.run_enabled = enabled

    def on_timer(self) -> None:
        now = self.get_clock().now()
        command = ActuatorCommand()
        command.header.stamp = now.to_msg()
        if self.run_enabled:
            if self.started_at_ns is None:
                self.started_at_ns = now.nanoseconds
            elapsed_s = max(0.0, (now.nanoseconds - self.started_at_ns) * 1.0e-9)
            setpoint = steering_step_setpoint(
                elapsed_s,
                self.amplitude_deg,
                self.frequency_hz,
                self.pattern,
                self.initial_straight_s,
            )
            command.speed_target_mps = self.constant_speed_mps
            command.steering_target_rad = math.radians(setpoint.steering_deg)
        else:
            setpoint = SteeringStepSetpoint(0.0, "WAIT_RUN", 0, 0, 0.0)
        command.brake_engage = False
        self.command_pub.publish(command)

        status = asdict(setpoint)
        status.update(
            {
                "active": self.run_enabled,
                "constant_speed_mps": self.constant_speed_mps,
                "amplitude_deg": self.amplitude_deg,
                "frequency_hz": self.frequency_hz,
                "pattern": self.pattern,
            }
        )
        status_message = String()
        status_message.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(status_message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SteeringStepTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
