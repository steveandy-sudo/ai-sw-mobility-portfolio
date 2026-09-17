import pytest

from kaiev26_decision.test_steering_step import (
    normalize_step_pattern,
    steering_step_setpoint,
)


def test_pattern_accepts_repeated_case_insensitive_directions() -> None:
    assert normalize_step_pattern("r l R") == "RLR"


@pytest.mark.parametrize("pattern", ["", "RC", "R-L"])
def test_pattern_rejects_non_direction_characters(pattern: str) -> None:
    with pytest.raises(ValueError):
        normalize_step_pattern(pattern)


def test_rlr_pulses_return_to_center_between_directions() -> None:
    kwargs = {
        "amplitude_deg": 4.0,
        "frequency_hz": 0.5,
        "pattern": "RLR",
        "initial_straight_s": 3.0,
    }
    assert steering_step_setpoint(2.9, **kwargs).steering_deg == 0.0
    assert steering_step_setpoint(3.1, **kwargs).steering_deg == -4.0
    assert steering_step_setpoint(4.1, **kwargs).steering_deg == 0.0
    assert steering_step_setpoint(5.1, **kwargs).steering_deg == 4.0
    assert steering_step_setpoint(6.1, **kwargs).steering_deg == 0.0
    assert steering_step_setpoint(7.1, **kwargs).steering_deg == -4.0


def test_pattern_repeats_after_last_direction() -> None:
    setpoint = steering_step_setpoint(
        elapsed_s=9.1,
        amplitude_deg=2.0,
        frequency_hz=0.5,
        pattern="RLR",
        initial_straight_s=3.0,
    )
    assert setpoint.steering_deg == -2.0
    assert setpoint.pattern_index == 0
    assert setpoint.pattern_cycle == 1
