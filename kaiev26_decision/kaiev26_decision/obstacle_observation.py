"""Odometry history: the pose at a stamp and cached local paths moved to the current pose."""
from __future__ import annotations

from bisect import bisect_left
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
import math


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


@dataclass(frozen=True)
class PoseSample:
    time: float
    x: float
    y: float
    z: float
    yaw: float


class ObstacleObservationBuffer:
    def __init__(self, history_sec: float = 2.0, extrapolation_sec: float = 0.1):
        self.history_sec = history_sec
        self.extrapolation_sec = extrapolation_sec
        self.samples: deque[PoseSample] = deque(maxlen=512)
        self.frames: tuple[str, str] | None = None

    def add_odometry(self, message) -> None:
        frame = (message.header.frame_id.lstrip('/'), message.child_frame_id.lstrip('/'))
        position, q = message.pose.pose.position, message.pose.pose.orientation
        t = stamp_seconds(message.header.stamp)
        values = (t, position.x, position.y, position.z, q.x, q.y, q.z, q.w)
        norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
        if not all(math.isfinite(value) for value in values) or norm < 1.0e-12:
            return
        if not all(frame):
            return
        if frame != self.frames or (self.samples and t < self.samples[-1].time - 1.0):
            self.samples.clear()
            self.frames = frame
        if self.samples and t <= self.samples[-1].time:
            return
        yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), norm-2.0*(q.y*q.y+q.z*q.z))
        self.samples.append(PoseSample(t, position.x, position.y, position.z, yaw))
        while len(self.samples) > 2 and self.samples[1].time < t-self.history_sec:
            self.samples.popleft()

    def pose_at(self, time_sec: float) -> PoseSample | None:
        samples = list(self.samples)
        if not samples or time_sec < samples[0].time:
            return None
        index = bisect_left([p.time for p in samples], time_sec)
        if index < len(samples) and abs(samples[index].time-time_sec) < 1.0e-7:
            return samples[index]
        if index == len(samples):
            if len(samples) < 2 or time_sec-samples[-1].time > self.extrapolation_sec:
                return None
            first, last = samples[-2:]
        elif index == 0:
            return None
        else:
            first, last = samples[index-1:index+1]
        dt = last.time-first.time
        if not 0.0 < dt <= 0.25:
            return None
        fraction = (time_sec-first.time)/dt
        angle = math.atan2(math.sin(last.yaw-first.yaw), math.cos(last.yaw-first.yaw))
        return PoseSample(time_sec,
                          first.x+fraction*(last.x-first.x),
                          first.y+fraction*(last.y-first.y),
                          first.z+fraction*(last.z-first.z), first.yaw+fraction*angle)

    def align_path(self, path, target_header):
        """Rebase a cached vehicle-relative path, or return None if poses are unavailable.

        Unlike sensor alignment, failure must not look like a successful re-stamp:
        callers keep the original header so downstream target-age checks still work.
        This transforms coordinates only; it does not revalidate obstacle clearance.
        """
        source_time = stamp_seconds(path.header.stamp)
        target_time = stamp_seconds(target_header.stamp)
        frame = path.header.frame_id.lstrip('/')
        if (not frame or frame != target_header.frame_id.lstrip('/')
                or source_time <= 0.0 or target_time < source_time
                or len(path.points) < 2
                or not all(math.isfinite(v) for p in path.points for v in (p.x, p.y, p.z))):
            return None
        if source_time == target_time:
            aligned = deepcopy(path)
            aligned.header = deepcopy(target_header)
            return aligned
        if (self.frames is None or frame != self.frames[1] or not self.samples
                or source_time > self.samples[-1].time):
            return None
        source, target = self.pose_at(source_time), self.pose_at(target_time)
        if source is None or target is None:
            return None

        # p_target = inverse(T_world_target) * T_world_source * p_source.
        angle = source.yaw-target.yaw
        c, s = math.cos(angle), math.sin(angle)
        ct, st = math.cos(target.yaw), math.sin(target.yaw)
        dx, dy = source.x-target.x, source.y-target.y
        tx, ty = ct*dx+st*dy, -st*dx+ct*dy
        aligned = deepcopy(path)
        aligned.header = deepcopy(target_header)
        for point in aligned.points:
            x, y = point.x, point.y
            point.x, point.y = c*x-s*y+tx, s*x+c*y+ty
            point.z += source.z-target.z
        return aligned
