"""Arbiter and output ownership: situation -> result tables."""
import math

from kaiev26_msgs.msg import Centerline, TargetSpeed
import pytest

from kaiev26_decision.arbiter import (
    CAP,
    OUTPUT_OWNERSHIP,
    PATH,
    STOP,
    ModuleOutput,
    OwnershipError,
    arbitrate,
    check_ownership,
)
from kaiev26_decision.common import stop_command


def path(source):
    message = Centerline()
    message.source = source
    return message


def out(module, cap=None, stop=None, path_source=None):
    return ModuleOutput(
        module, cap_mps=cap, stop_distance_m=stop,
        path=path(path_source) if path_source else None,
        path_request=f"{module.upper()}_PATH" if path_source else "",
    )


# outputs -> (target, limit, need_stop, stop distance, path source)
ARBITER_CASES = {
    "minimum_cap_wins": (
        [out("route_follow", 8.0, path_source="route"), out("curve_speed", 5.6),
         out("obstacle", 3.0)],
        (3.0, 3.0, False, math.inf, "route"),
    ),
    "any_stop_request_stops_at_the_minimum_distance": (
        [out("route_follow", 8.0, path_source="route"), out("stop_point", stop=0.0),
         out("input_state", stop=0.0)],
        (0.0, 8.0, True, 0.0, "route"),
    ),
    "bypass_path_wins_while_requested_also_while_braking": (
        [out("route_follow", 8.0, path_source="route"), out("obstacle", 3.0, path_source="bypass"),
         out("stop_point", 0.6, stop=0.0)],
        (0.0, 0.6, True, 0.0, "bypass"),
    ),
    "input_stale_cap_is_speed_zero_without_a_stop": (
        [out("route_follow", 8.0, path_source="previous"), out("input_state", 0.0)],
        (0.0, 0.0, False, math.inf, "previous"),
    ),
    "non_finite_cap_counts_as_zero": (
        [out("route_follow", 8.0, path_source="route"), out("curve_speed", math.nan)],
        (0.0, 0.0, False, math.inf, "route"),
    ),
    "no_cap_at_all_is_zero": (
        [out("aeb")],
        (0.0, 0.0, False, math.inf, None),
    ),
}


@pytest.mark.parametrize("name", sorted(ARBITER_CASES))
def test_arbiter(name) -> None:
    outputs, (target, limit, need_stop, distance, source) = ARBITER_CASES[name]
    result = arbitrate(outputs)
    assert (result.target_speed_mps, result.speed_limit_mps, result.need_stop) == (target, limit, need_stop)
    assert result.stop_distance_m == distance
    assert (result.path.source if result.path is not None else None) == source


WRITES = {
    CAP: lambda m: ModuleOutput(m, cap_mps=1.0),
    STOP: lambda m: ModuleOutput(m, stop_distance_m=0.0),
    PATH: lambda m: ModuleOutput(m, path=Centerline(), path_request="X"),
}
OWNERSHIP_CASES = [
    (module, kind, kind in allowed)
    for module, allowed in OUTPUT_OWNERSHIP.items()
    for kind in (CAP, STOP, PATH)
] + [("unknown_module", CAP, False)]


@pytest.mark.parametrize("module, kind, allowed", OWNERSHIP_CASES)
def test_a_module_writing_outside_its_column_fails(module, kind, allowed) -> None:
    output = WRITES[kind](module)
    if allowed:
        check_ownership(output)
        arbitrate([output])
    else:
        with pytest.raises(OwnershipError):
            arbitrate([output])


@pytest.mark.parametrize("distance", [0.0, 1.5])
def test_stop_command_is_the_single_stop_builder(distance) -> None:
    target = TargetSpeed(target_speed_mps=3.0, speed_limit_mps=3.0)
    stop_command(target, distance)
    assert target.need_stop
    assert target.target_speed_mps == 0.0
    assert target.stop_target_distance == pytest.approx(distance)
