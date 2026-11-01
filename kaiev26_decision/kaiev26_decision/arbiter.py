"""Output ownership and the arbiter of the planning engine.

Every module returns a ModuleOutput each tick. The arbiter takes the minimum of all speed caps,
stops when any stop request is present (at the minimum requested distance) and takes the path
of the obstacle behavior while it requests one, otherwise the route follow path.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from kaiev26_msgs.msg import Centerline


PATH = "path"
STOP = "stop"
CAP = "cap"

# Who may write which output. A module that writes outside its column is a programming error.
OUTPUT_OWNERSHIP = {
    "input_state": frozenset({STOP, CAP}),
    "stop_point": frozenset({STOP, CAP}),
    "obstacle": frozenset({PATH, CAP}),
    "route_follow": frozenset({PATH, CAP}),
    "curve_speed": frozenset({CAP}),
    "speed_zone": frozenset({CAP}),
    "aeb": frozenset(),
}
PATH_PRIORITY = ("obstacle", "route_follow")


class OwnershipError(RuntimeError):
    pass


@dataclass
class ModuleOutput:
    module: str
    cap_mps: float | None = None
    stop_distance_m: float | None = None
    path: Centerline | None = None
    path_request: str = ""
    tags: list[str] = field(default_factory=list)

    def writes(self) -> set[str]:
        written = set()
        if self.cap_mps is not None:
            written.add(CAP)
        if self.stop_distance_m is not None:
            written.add(STOP)
        if self.path is not None or self.path_request:
            written.add(PATH)
        return written


@dataclass(frozen=True)
class Arbitration:
    target_speed_mps: float
    speed_limit_mps: float
    need_stop: bool
    stop_distance_m: float
    path: Centerline | None
    path_request: str
    path_owner: str
    stop_owners: tuple[str, ...]
    binding_cap_owner: str
    tags: tuple[str, ...]


def check_ownership(output: ModuleOutput) -> None:
    allowed = OUTPUT_OWNERSHIP.get(output.module)
    if allowed is None:
        raise OwnershipError(f"module [{output.module}] has no ownership entry")
    extra = output.writes() - allowed
    if extra:
        raise OwnershipError(f"module [{output.module}] may not write {sorted(extra)}")


def arbitrate(outputs: list[ModuleOutput]) -> Arbitration:
    caps = []
    stops = []
    tags: list[str] = []
    for output in outputs:
        check_ownership(output)
        if output.cap_mps is not None:
            cap = float(output.cap_mps)
            caps.append((max(0.0, cap) if math.isfinite(cap) else 0.0, output.module))
        if output.stop_distance_m is not None:
            stops.append((max(0.0, float(output.stop_distance_m)), output.module))
        tags.extend(output.tags)
    limit, binding = min(caps) if caps else (0.0, "")
    by_module = {output.module: output for output in outputs}
    path, request, owner = None, "", ""
    for module in PATH_PRIORITY:
        output = by_module.get(module)
        if output is not None and (output.path is not None or output.path_request):
            path, request, owner = output.path, output.path_request, module
            break
    need_stop = bool(stops)
    return Arbitration(
        target_speed_mps=0.0 if need_stop else limit,
        speed_limit_mps=limit,
        need_stop=need_stop,
        stop_distance_m=min(stops)[0] if need_stop else math.inf,
        path=path,
        path_request=request,
        path_owner=owner,
        stop_owners=tuple(dict.fromkeys(module for _, module in stops)),
        binding_cap_owner=binding,
        tags=tuple(dict.fromkeys(tags)),
    )
