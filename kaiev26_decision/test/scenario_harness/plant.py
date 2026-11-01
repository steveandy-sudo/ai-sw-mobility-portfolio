"""Kinematic bicycle that follows the ActuatorCommand like the car's actuators do.

Reference point is the rear-axle centre (base_footprint). Longitudinal:
- drive: speed_target_mps is a setpoint; the car accelerates toward it at accel_mps2 and, when
  the setpoint is below the speed, has no drive and coasts at coast_decel_mps2 (no regen model);
- brake: brake_engage applies the service brake brake_delay_s later at brake_decel_mps2, drive is
  zero from the moment it is commanded (STM_A), release takes brake_release_s;
- a command older than command_timeout_s cuts drive (STM_A), the brake keeps its last state (STM_B).
Lateral: the steering angle moves to the target at steering_rate_dps; while brake_engage is
commanded the angle is held, because lateral_ctrl disables the steering motor then.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class VehicleParams:
    wheelbase_m: float = 1.2991017929
    front_overhang_m: float = 1.98
    rear_overhang_m: float = 0.495  # kaiev26_description chassis collision box 2.473 m
    width_m: float = 1.18
    track_width_m: float = 0.89
    accel_mps2: float = 1.5
    coast_decel_mps2: float = 0.5
    brake_delay_s: float = 0.7
    brake_decel_mps2: float = 3.9
    brake_release_s: float = 0.7
    steering_rate_dps: float = 60.0
    max_steering_deg: float = 25.0
    command_timeout_s: float = 0.5

    @classmethod
    def from_dict(cls, values: dict | None) -> "VehicleParams":
        params = cls()
        for key, value in (values or {}).items():
            if not hasattr(params, key):
                raise ValueError(f"unknown vehicle parameter [{key}]")
            setattr(params, key, float(value))
        return params


@dataclass
class Command:
    stamp_sec: float
    speed_target_mps: float
    steering_target_rad: float
    brake_engage: bool


class Plant:
    def __init__(self, params: VehicleParams, x: float, y: float, yaw: float, speed_mps: float) -> None:
        self.p = params
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.v = max(0.0, float(speed_mps))
        self.steer = 0.0
        self.command: Command | None = None
        self.brake_commanded = False
        self.brake_commanded_since: float | None = None
        self.brake_released_since: float | None = None
        self.brake_applied = False
        self.drive_active = False

    def set_command(self, command: Command) -> None:
        self.command = command

    def _update_brake(self, t: float) -> None:
        command = self.command
        commanded = self.brake_commanded if command is None else bool(command.brake_engage)
        if commanded and not self.brake_commanded:
            self.brake_commanded_since = t
            self.brake_released_since = None
        elif not commanded and self.brake_commanded:
            self.brake_released_since = t
            self.brake_commanded_since = None
        self.brake_commanded = commanded
        if commanded:
            self.brake_applied = t - self.brake_commanded_since >= self.p.brake_delay_s - 1.0e-9
        elif self.brake_applied and self.brake_released_since is not None:
            if t - self.brake_released_since >= self.p.brake_release_s - 1.0e-9:
                self.brake_applied = False

    def step(self, t: float, dt: float) -> None:
        """Advance from t - dt to t."""
        self._update_brake(t)
        command = self.command
        fresh = command is not None and t - command.stamp_sec <= self.p.command_timeout_s + 1.0e-9
        target = max(0.0, command.speed_target_mps) if fresh else 0.0
        drive_allowed = fresh and not self.brake_commanded
        if self.brake_applied:
            accel = -self.p.brake_decel_mps2
            self.drive_active = False
        elif drive_allowed and target > self.v + 1.0e-6:
            accel = min(self.p.accel_mps2, (target - self.v) / dt)
            self.drive_active = True
        elif drive_allowed and target >= self.v - 1.0e-6 and self.v > 0.0:
            accel = 0.0  # holding the setpoint
            self.drive_active = True
        else:
            accel = -self.p.coast_decel_mps2 if self.v > 0.0 else 0.0
            self.drive_active = False
        v_next = max(0.0, self.v + accel * dt)
        # Steering: held while the brake is commanded, else rate limited toward the target.
        if command is not None and not self.brake_commanded:
            limit = math.radians(self.p.max_steering_deg)
            goal = max(-limit, min(limit, float(command.steering_target_rad)))
            step = math.radians(self.p.steering_rate_dps) * dt
            self.steer += max(-step, min(step, goal - self.steer))
        v_mid = 0.5 * (self.v + v_next)
        distance = v_mid * dt
        if distance > 0.0:
            yaw_rate = v_mid * math.tan(self.steer) / self.p.wheelbase_m
            yaw_mid = self.yaw + 0.5 * yaw_rate * dt
            self.x += distance * math.cos(yaw_mid)
            self.y += distance * math.sin(yaw_mid)
            self.yaw = math.atan2(math.sin(self.yaw + yaw_rate * dt), math.cos(self.yaw + yaw_rate * dt))
        self.v = v_next

    def body_point(self, forward_m: float, left_m: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * forward_m - s * left_m, self.y + s * forward_m + c * left_m

    def front_bumper(self) -> tuple[float, float]:
        return self.body_point(self.p.front_overhang_m, 0.0)

    def body_corners(self) -> list[tuple[float, float]]:
        half = self.p.width_m / 2.0
        front, rear = self.p.front_overhang_m, -self.p.rear_overhang_m
        return [self.body_point(front, half), self.body_point(front, -half),
                self.body_point(rear, -half), self.body_point(rear, half)]

    def wheels(self) -> list[tuple[float, float]]:
        half = self.p.track_width_m / 2.0
        return [self.body_point(0.0, half), self.body_point(0.0, -half),
                self.body_point(self.p.wheelbase_m, half), self.body_point(self.p.wheelbase_m, -half)]
