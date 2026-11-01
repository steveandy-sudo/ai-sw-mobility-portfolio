"""Front-axle Stanley steering with the existing AEB speed/brake state machine.

Path input is rear-axle/base_link, x forward, y left. Positive steering is left.
Reference: https://robots.stanford.edu/papers/thrun.stanley05.pdf, section 9.2.

The steering law is the qualifying/final tracker of kaiev26_motion_control on a straight:
- speed-scheduled straight gains (natural frequency and damping, fixed gains below the blend),
- heading and cross-track terms clamped to the straight correction limits,
- a slow integral of the front-axle cross-track error,
- the encoder_error compensation (neutral offset and friction push) in front of the actuator.
Its values come from motion_control.yaml (motion_control_defaults), so the cone trial and the
missions steer with the same numbers.
"""
from dataclasses import dataclass, field
import math
from pathlib import Path

import numpy as np

from kaiev26_motion_control.motion_control_node import (
    SteeringCompensationConfig,
    SteeringCompensator,
    scheduled_stanley_gains,
    validate_steering_compensation,
)

from .pursuit import Pursuit, PursuitConfig


# StanleyConfig field -> kaiev26_motion_control parameter.
MOTION_CONTROL_PARAMETERS = {
    'wheelbase_m': 'wheelbase_m',
    'max_steering_rad': 'max_steering_rad',
    'heading_gain': 'stanley_heading_gain',
    'cross_track_gain': 'stanley_cross_track_gain',
    'soft_speed_mps': 'stanley_softening_speed_mps',
    'heading_correction_limit_rad': 'stanley_heading_correction_limit_rad',
    'cross_track_correction_limit_rad': 'stanley_cross_track_correction_limit_rad',
    'natural_frequency_radps': 'straight_natural_frequency_radps',
    'damping_ratio': 'straight_damping_ratio',
    'gain_blend_start_mps': 'straight_gain_blend_start_mps',
    'gain_full_speed_mps': 'straight_gain_full_speed_mps',
    'integral_time_s': 'straight_integral_time_s',
    'integral_limit_deg': 'straight_integral_limit_deg',
    'integral_min_speed_mps': 'straight_integral_min_speed_mps',
    'integral_max_error_m': 'straight_integral_max_error_m',
    'straight_encoder_deg': 'straight_encoder_deg',
    'steering_neutral_offset_deg': 'steering_neutral_offset_deg',
    'steering_friction_speed_mps': 'steering_friction_speed_mps',
    'steering_friction_comp_deg': 'steering_friction_comp_deg',
    'steering_error_start_deg': 'steering_error_start_deg',
    'steering_error_full_deg': 'steering_error_full_deg',
    'steering_error_ramp_s': 'steering_error_ramp_s',
}


def motion_control_yaml_path():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('kaiev26_motion_control')) / 'config/motion_control.yaml'


def motion_control_defaults(path=None):
    """StanleyConfig values from the installed (or given) motion_control.yaml."""
    import yaml
    if path is None:
        path = motion_control_yaml_path()
    parameters = yaml.safe_load(Path(path).read_text())['kaiev26_motion_control']['ros__parameters']
    missing = sorted(name for name in MOTION_CONTROL_PARAMETERS.values() if name not in parameters)
    if missing:
        raise ValueError(f'motion_control.yaml lacks {", ".join(missing)}')
    return {key: (list(map(float, parameters[name])) if isinstance(parameters[name], list)
                  else float(parameters[name]))
            for key, name in MOTION_CONTROL_PARAMETERS.items()}


@dataclass
class StanleyConfig:
    target_speed_kph: float = 30.0
    wheelbase_m: float = 1.2991017929
    # Fixed straight gains below gain_blend_start_mps; scheduled from gain_full_speed_mps.
    heading_gain: float = 0.30
    cross_track_gain: float = 0.25
    soft_speed_mps: float = 3.0
    heading_correction_limit_rad: float = 0.10
    cross_track_correction_limit_rad: float = 0.05
    natural_frequency_radps: float = 1.5
    damping_ratio: float = 1.1
    gain_blend_start_mps: float = 3.0
    gain_full_speed_mps: float = 6.0
    integral_time_s: float = 4.0
    integral_limit_deg: float = 1.5
    integral_min_speed_mps: float = 4.0
    integral_max_error_m: float = 0.6
    # Encoder angle of straight running: the yaw rate of the path motion is measured about it.
    straight_encoder_deg: float = -0.20
    steering_neutral_offset_deg: float = -0.52
    steering_friction_speed_mps: list = field(default_factory=lambda: [3.0, 6.5, 12.0])
    steering_friction_comp_deg: list = field(default_factory=lambda: [1.0, 0.85, 0.8])
    steering_error_start_deg: float = 0.2
    steering_error_full_deg: float = 0.5
    steering_error_ramp_s: float = 0.1
    max_steering_rad: float = 0.4363
    steering_rate_radps: float = 1.0471975512
    acceleration_mps2: float = 1.0
    path_timeout_s: float = 0.35
    state_timeout_s: float = 0.25
    red_front_offset_m: float = 1.98
    red_confirm_frames: int = 2
    red_gate_match_m: float = 1.0
    red_brake_lead_m: float = 0.0
    red_tracking_stop_speed_mps: float = 0.5
    vehicle_brake: bool = True
    path_gap_hold_s: float = 0.5

    def compensation(self):
        return SteeringCompensationConfig(
            neutral_offset_rad=math.radians(self.steering_neutral_offset_deg),
            friction_speeds_mps=tuple(float(v) for v in self.steering_friction_speed_mps),
            friction_rad=tuple(math.radians(float(v)) for v in self.steering_friction_comp_deg),
            error_start_rad=math.radians(self.steering_error_start_deg),
            error_full_rad=math.radians(self.steering_error_full_deg),
            error_ramp_s=self.steering_error_ramp_s,
        )

    def validate(self):
        shared = vars(PursuitConfig()).keys() & vars(self).keys()
        PursuitConfig(**{k: getattr(self, k) for k in shared}).validate()
        positive = (self.heading_gain, self.cross_track_gain, self.soft_speed_mps,
                    self.heading_correction_limit_rad, self.cross_track_correction_limit_rad,
                    self.integral_time_s, self.integral_max_error_m)
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError('Stanley gains, limits and integral settings must be finite and positive')
        if not (0.3 <= self.natural_frequency_radps <= 3.0 and 0.5 <= self.damping_ratio <= 1.5
                and 0.0 <= self.gain_blend_start_mps < self.gain_full_speed_mps
                and 0.0 <= self.integral_limit_deg <= 3.0 and self.integral_min_speed_mps >= 0.0
                and abs(self.straight_encoder_deg) <= 3.0):
            raise ValueError('Stanley straight schedule, integral or straight encoder out of range')
        validate_steering_compensation(self.compensation())


class Stanley(Pursuit):
    controller_name = 'stanley'

    def __init__(self, config=None):
        super().__init__(config or StanleyConfig())
        self.compensator = SteeringCompensator(self.cfg.compensation())
        self.reset_lateral()

    def reset_lateral(self):
        self.integral_rad = 0.0
        self.compensator.reset()

    def motion_steering(self, steering):
        return steering - math.radians(self.cfg.straight_encoder_deg)

    def actuator_command(self, raw, steering, speed, dt):
        return self.compensator.apply(raw, steering, speed, dt)

    def straight_gains(self, speed):
        """motion_control straight_gains: scheduled from gain_full_speed_mps, blending back to the
        fixed gains by gain_blend_start_mps."""
        cfg = self.cfg
        speed = abs(float(speed))
        full = cfg.gain_full_speed_mps
        heading, cross_track = scheduled_stanley_gains(
            max(speed, full), cfg.natural_frequency_radps, cfg.damping_ratio,
            cfg.wheelbase_m, cfg.soft_speed_mps, full)
        share = min(1.0, max(0.0, (speed - cfg.gain_blend_start_mps) / (full - cfg.gain_blend_start_mps)))
        return (cfg.heading_gain + share * (heading - cfg.heading_gain),
                cfg.cross_track_gain + share * (cross_track - cfg.cross_track_gain))

    def lateral_solution(self, current, speed):
        cfg = self.cfg
        front = np.array([cfg.wheelbase_m, 0.0])
        starts, vectors = current[:-1], np.diff(current, axis=0)
        length2 = np.einsum('ij,ij->i', vectors, vectors)
        valid = length2 > 1e-12
        starts, vectors, length2 = starts[valid], vectors[valid], length2[valid]
        if not len(starts):
            return dict(reason='no_front_axle_reference')
        t = np.einsum('ij,ij->i', front - starts, vectors) / length2
        closest = starts + np.clip(t, 0, 1)[:, None] * vectors
        index = int(np.argmin(np.sum((closest - front) ** 2, axis=1)))
        # Require support from the supplied polyline, never extrapolate its end.
        if not -1e-6 <= t[index] <= 1 + 1e-6 or closest[index, 0] <= 0:
            return dict(reason='no_front_axle_reference')
        tangent = vectors[index] / math.sqrt(length2[index])
        heading = math.atan2(tangent[1], tangent[0])
        normal = np.array([-tangent[1], tangent[0]])
        cross_track = float((closest[index] - front) @ normal)
        heading_gain, cross_track_gain = self.straight_gains(speed)
        limit = cfg.heading_correction_limit_rad
        heading_correction = min(limit, max(-limit, heading_gain * heading))
        limit = cfg.cross_track_correction_limit_rad
        correction = min(limit, max(-limit, math.atan2(
            cross_track_gain * cross_track, cfg.soft_speed_mps + abs(speed))))
        # A large error is a transient for the proportional terms; latched braking holds it.
        if (not (self.red.latched and cfg.vehicle_brake) and abs(speed) >= cfg.integral_min_speed_mps
                and abs(cross_track) <= cfg.integral_max_error_m):
            rate = cross_track_gain * cross_track / (abs(speed) + cfg.soft_speed_mps) / cfg.integral_time_s
            limit = math.radians(cfg.integral_limit_deg)
            self.integral_rad = min(limit, max(-limit, self.integral_rad + rate * self.tick_dt))
        return dict(reason='tracking', target_xy=closest[index].tolist(),
                    raw_steering_rad=heading_correction + correction + self.integral_rad,
                    heading_error_rad=heading, cross_track_error_m=cross_track,
                    heading_gain=heading_gain, cross_track_gain=cross_track_gain,
                    heading_correction_rad=heading_correction,
                    cross_track_correction_rad=correction,
                    straight_integral_rad=self.integral_rad)
