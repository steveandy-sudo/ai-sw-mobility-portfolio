"""One input-loss rule for Decision, motion_control and the test TUI.

The moment an input is stale the command is speed 0 without brake. If it has not recovered
within brake_after_sec, the brake is requested, and that request stays until the vehicle has
actually stopped (|v| <= stopped_speed_mps), even if the input comes back meanwhile. After the
stop, a returned input removes the request. A brake that was already engaged when the input went
stale stays engaged while the input is stale; when the input returns before the delay, the
returned command decides again (main, decision D2). The last command is never kept.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class InputLossState:
    stale: bool = False
    brake: bool = False
    stale_sec: float = 0.0
    # The brake requested after brake_after_sec (held until stopped), not the held engaged brake.
    latched: bool = False

    @property
    def active(self) -> bool:
        return self.stale or self.brake


class InputLossLatch:
    def __init__(self, brake_after_sec: float = 1.0, stopped_speed_mps: float = 0.1) -> None:
        self.brake_after_sec = max(0.0, float(brake_after_sec))
        self.stopped_speed_mps = max(0.0, float(stopped_speed_mps))
        self.reset()

    def reset(self) -> None:
        self.stale_since: float | None = None
        self.brake_latched = False
        self.brake_held = False

    def stopped(self, speed_mps: float | None) -> bool:
        return (
            speed_mps is not None
            and math.isfinite(float(speed_mps))
            and abs(float(speed_mps)) <= self.stopped_speed_mps
        )

    def update(
        self,
        now_sec: float,
        inputs_ok: bool,
        speed_mps: float | None,
        *,
        brake_engaged: bool = False,
    ) -> InputLossState:
        stale_sec = 0.0
        if inputs_ok:
            self.stale_since = None
            self.brake_held = False
        else:
            if self.stale_since is None or now_sec < self.stale_since:
                self.stale_since = now_sec
                # A brake already engaged when the input went stale stays engaged while it is stale.
                self.brake_held = bool(brake_engaged)
            stale_sec = now_sec - self.stale_since
            if stale_sec >= self.brake_after_sec - 1.0e-9:
                self.brake_latched = True
        if self.brake_latched and inputs_ok and self.stopped(speed_mps):
            self.brake_latched = False
        return InputLossState(
            stale=not inputs_ok, brake=self.brake_latched or self.brake_held, stale_sec=stale_sec,
            latched=self.brake_latched,
        )
