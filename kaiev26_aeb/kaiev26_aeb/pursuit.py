"""Pure Pursuit for the cone planner's forward, rear-axle/base_link path.

No ROS or simulator dependency. Positive steering is left; all units are SI.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class PursuitConfig:
    target_speed_kph: float = 30.0
    wheelbase_m: float = 1.2991017929
    lookahead_base_m: float = 7.0
    lookahead_gain_s: float = 0.0
    lookahead_max_m: float = 7.0
    max_steering_rad: float = 0.4363
    steering_rate_radps: float = 1.0471975512
    acceleration_mps2: float = 2.0
    path_timeout_s: float = 0.35
    state_timeout_s: float = 0.25
    red_front_offset_m: float = 1.98
    red_confirm_frames: int = 2
    red_gate_match_m: float = 1.0

    def validate(self):
        if any(not math.isfinite(v) or v <= 0 for k, v in vars(self).items() if k != 'lookahead_gain_s'):
            raise ValueError('Pursuit parameters must be finite and positive')
        if not math.isfinite(self.lookahead_gain_s) or self.lookahead_gain_s < 0:
            raise ValueError('lookahead_gain_s must be finite and nonnegative')
        if self.lookahead_max_m < self.lookahead_base_m:
            raise ValueError('lookahead_max_m must be >= lookahead_base_m')
        if self.max_steering_rad >= math.pi / 2:
            raise ValueError('max_steering_rad must be < pi/2')
        if int(self.red_confirm_frames) != self.red_confirm_frames:
            raise ValueError('red_confirm_frames must be an integer')


def advance_path(points, speed, steering, age, wheelbase):
    """Approximate acquisition-to-control motion using measured speed/steering.

    Constant bicycle motion over the short accepted path age, not a stored map.
    """
    angle = speed * math.tan(steering) / wheelbase * age
    if abs(angle) < 1e-7:
        displacement = np.array([speed * age, 0.0])
    else:
        radius = speed * age / angle
        displacement = np.array([radius * math.sin(angle), radius * (1 - math.cos(angle))])
    c, s = math.cos(angle), math.sin(angle)
    return (points - displacement) @ np.array([[c, -s], [s, c]])


def pursuit_target(points, distance):
    """First forward path/circle intersection; never invent a path extension."""
    for a, b in zip(points[:-1], points[1:]):
        d = b - a
        aa = float(d @ d)
        if aa < 1e-12:
            continue
        bb = 2 * float(a @ d)
        cc = float(a @ a) - distance ** 2
        disc = bb * bb - 4 * aa * cc
        if disc < 0:
            continue
        for t in sorted(((-bb - math.sqrt(disc)) / (2 * aa),
                         (-bb + math.sqrt(disc)) / (2 * aa))):
            if -1e-9 <= t <= 1 + 1e-9:
                target = a + np.clip(t, 0, 1) * d
                if target[0] > 0:
                    return target
    return None


class RedStop:
    """Remember one confirmed entry gate, compensate motion, and latch braking.

    The gate is a measured left/right red pair, never a Gazebo coordinate.
    Later rows cannot replace the first confirmed entry. No automatic release.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.gate = None
        self.count = 0
        self.confirmed = self.latched = False
        self.last_stamp = None
        self.distance = None

    def update(self, observed, stamp, age, speed, steering, dt):
        if self.gate is not None:
            self.gate = advance_path(self.gate, speed, steering, dt, self.cfg.wheelbase_m)
        if stamp is not None and stamp != self.last_stamp:
            self.last_stamp = stamp
            gate = np.asarray(observed if observed is not None else [], dtype=float)
            usable = (gate.shape == (2, 2) and np.isfinite(gate).all()
                      and math.isfinite(age) and -0.05 <= age <= self.cfg.path_timeout_s)
            if usable:
                gate = advance_path(gate, speed, steering, max(0.0, age), self.cfg.wheelbase_m)
                span = gate[1] - gate[0]
                usable = np.linalg.norm(span) > 1.0 and -span[1] > 0.5 * np.linalg.norm(span)
            if usable:
                matches = (self.gate is not None
                           and np.linalg.norm(gate - self.gate, axis=1).max() <= self.cfg.red_gate_match_m)
                if matches or not self.confirmed:
                    self.gate = gate
                    self.count = self.count + 1 if matches else 1
                    self.confirmed = self.count >= self.cfg.red_confirm_frames
            elif not self.confirmed:
                self.gate, self.count = None, 0
        if self.confirmed:
            span = self.gate[1] - self.gate[0]
            normal = np.array([-span[1], span[0]]) / np.linalg.norm(span)
            front = np.array([self.cfg.red_front_offset_m, 0.0])
            self.distance = float((self.gate[0] - front) @ normal)
            across = float((front - self.gate[0]) @ span / (span @ span))
            if self.distance <= 0.0 and 0.0 <= across <= 1.0:
                self.latched = True

    def status(self, speed):
        state = 'latched' if self.latched else (
            'approaching' if self.confirmed else 'confirming' if self.count else 'waiting')
        return dict(red_state=state, red_latched=self.latched, red_confirm_count=self.count,
                    red_distance_m=self.distance,
                    red_gate_xy=self.gate.tolist() if self.gate is not None else [])


class Pursuit:
    controller_name = 'pure_pursuit'

    def __init__(self, config=None):
        self.cfg = config or PursuitConfig()
        self.cfg.validate()
        self.speed_command = 0.0
        self.steering_command = 0.0
        self.red = RedStop(self.cfg)

    def lateral_solution(self, current, speed):
        """Only this calculation differs in the Stanley controller."""
        cfg = self.cfg
        lookahead = min(cfg.lookahead_max_m, cfg.lookahead_base_m + cfg.lookahead_gain_s * abs(speed))
        target = pursuit_target(current, lookahead)
        if target is None:
            return dict(reason='short_path_or_no_forward_target', lookahead_m=lookahead)
        raw = math.atan2(2 * cfg.wheelbase_m * target[1], float(target @ target))
        return dict(reason='tracking', lookahead_m=lookahead,
                    target_xy=target.tolist(), raw_steering_rad=raw)

    def step(self, points, speed, steering, path_age, state_age, dt,
             enabled=True, frame='base_link', estop=False, reverse=False,
             red_gate=None, red_gate_stamp=None, red_gate_age=1e6):
        cfg = self.cfg
        reason = 'tracking'
        lateral = dict(lookahead_m=None, target_xy=None)
        if self.red.latched:
            reason = 'red_stop_latched'
        elif not enabled:
            reason = 'disabled'
        elif not all(math.isfinite(v) for v in (speed, steering, path_age, state_age, dt)):
            reason = 'invalid_state_or_time'
        elif dt <= 0 or path_age < -0.05 or state_age < -0.05:
            reason = 'clock_mismatch'
        elif estop:
            reason = 'estop'
        elif reverse or speed < -0.1:
            reason = 'reverse'
        elif state_age > cfg.state_timeout_s:
            reason = 'stale_state'
        else:
            self.red.update(red_gate, red_gate_stamp, red_gate_age, speed, steering, dt)
            points = np.asarray(points, dtype=float)
            if self.red.latched:
                reason = 'red_stop_latched'
            elif path_age > cfg.path_timeout_s:
                reason = 'stale_path'
            elif frame != 'base_link':
                reason = 'wrong_path_frame'
            elif points.ndim != 2 or points.shape[1] != 2 or len(points) < 2:
                reason = 'empty_path'
            elif not np.isfinite(points).all() or np.any(np.diff(points[:, 0]) < 0):
                reason = 'invalid_path'
            else:
                current = advance_path(points, speed, steering, max(0.0, path_age), cfg.wheelbase_m)
                lateral.update(self.lateral_solution(current, speed))
                reason = lateral.pop('reason')
        if reason != 'tracking':
            self.speed_command = 0.0
            # Hold the last limited angle while braking; do not snap the wheels straight.
            return dict(reason=reason, controller=self.controller_name, tracking=False, speed_target_mps=0.0,
                        steering_target_rad=self.steering_command, brake_engage=True,
                        **lateral, **self.red.status(speed))
        raw = lateral['raw_steering_rad']
        desired = float(np.clip(raw, -cfg.max_steering_rad, cfg.max_steering_rad))
        # Bound a scheduling hiccup instead of catching up with a large command jump.
        dt = min(dt, 0.1)
        self.steering_command += float(np.clip(desired - self.steering_command,
                                               -cfg.steering_rate_radps * dt, cfg.steering_rate_radps * dt))
        self.speed_command = min(cfg.target_speed_kph / 3.6,
                                 self.speed_command + cfg.acceleration_mps2 * dt)
        return dict(reason=reason, controller=self.controller_name, tracking=True, speed_target_mps=self.speed_command,
                    steering_target_rad=self.steering_command, brake_engage=False,
                    **lateral,
                    **self.red.status(speed))
