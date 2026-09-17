"""Front-axle Stanley steering with the existing AEB speed/brake state machine.

Path input is rear-axle/base_link, x forward, y left. Positive steering is left.
Reference: https://robots.stanford.edu/papers/thrun.stanley05.pdf, section 9.2.
"""
from dataclasses import dataclass
import math

import numpy as np

from .pursuit import Pursuit, PursuitConfig


@dataclass
class StanleyConfig:
    target_speed_kph: float = 30.0
    wheelbase_m: float = 1.2991017929
    heading_gain: float = 0.4
    cross_track_gain: float = 0.4
    soft_speed_mps: float = 2.0
    max_steering_rad: float = 0.4363
    steering_rate_radps: float = 1.0471975512
    acceleration_mps2: float = 2.0
    path_timeout_s: float = 0.35
    state_timeout_s: float = 0.25
    red_front_offset_m: float = 1.98
    red_confirm_frames: int = 2
    red_gate_match_m: float = 1.0

    def validate(self):
        shared = vars(PursuitConfig()).keys() & vars(self).keys()
        PursuitConfig(**{k: getattr(self, k) for k in shared}).validate()
        for value in (self.heading_gain, self.cross_track_gain, self.soft_speed_mps):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Stanley gains must be finite and positive')


class Stanley(Pursuit):
    controller_name = 'stanley'

    def __init__(self, config=None):
        super().__init__(config or StanleyConfig())

    def lateral_solution(self, current, speed):
        front = np.array([self.cfg.wheelbase_m, 0.0])
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
        correction = math.atan2(self.cfg.cross_track_gain * cross_track,
                                self.cfg.soft_speed_mps + abs(speed))
        return dict(reason='tracking', target_xy=closest[index].tolist(),
                    raw_steering_rad=self.cfg.heading_gain * heading + correction,
                    heading_error_rad=heading, cross_track_error_m=cross_track,
                    cross_track_correction_rad=correction)
