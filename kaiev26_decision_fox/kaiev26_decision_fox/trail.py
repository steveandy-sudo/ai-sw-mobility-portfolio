from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TrailPoint:
    x: float
    y: float
    z: float
    stamp_ns: int


class TrailBuffer:
    def __init__(
        self,
        max_points: int,
        min_spacing_m: float,
        max_jump_m: float = float("inf"),
    ) -> None:
        if max_points < 2:
            raise ValueError("max_points must be at least 2")
        if min_spacing_m < 0.0 or not math.isfinite(min_spacing_m):
            raise ValueError("min_spacing_m must be finite and non-negative")
        if max_jump_m <= 0.0 or math.isnan(max_jump_m):
            raise ValueError("max_jump_m must be positive")
        self._points: deque[TrailPoint] = deque(maxlen=max_points)
        self._min_spacing_m = min_spacing_m
        self._max_jump_m = max_jump_m

    @property
    def points(self) -> tuple[TrailPoint, ...]:
        return tuple(self._points)

    def clear(self) -> None:
        self._points.clear()

    def add(self, point: TrailPoint) -> bool:
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return False
        if self._points and point.stamp_ns < self._points[-1].stamp_ns:
            self.clear()
        if self._points:
            previous = self._points[-1]
            distance = math.hypot(point.x - previous.x, point.y - previous.y)
            if distance > self._max_jump_m:
                self.clear()
            elif distance < self._min_spacing_m:
                return False
        self._points.append(point)
        return True
