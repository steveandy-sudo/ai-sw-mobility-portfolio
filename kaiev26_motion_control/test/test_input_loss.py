"""The shared input-loss rule (D2): situation -> result tables."""
import pytest

from kaiev26_motion_control.input_loss import InputLossLatch


# (time s, inputs ok, measured speed, brake already engaged) -> (stale, brake)
CASES = {
    "short_loss_is_speed_zero_without_brake": [
        (0.0, True, 5.0, False, (False, False)),
        (0.1, False, 5.0, False, (True, False)),
        (0.9, False, 4.0, False, (True, False)),
        (1.0, True, 3.5, False, (False, False)),
    ],
    "loss_of_one_second_brakes_until_actually_stopped": [
        (0.0, False, 5.0, False, (True, False)),
        (1.0, False, 4.0, False, (True, True)),
        (1.2, True, 3.0, False, (False, True)),
        (1.5, True, 0.5, False, (False, True)),
        (1.8, True, 0.05, False, (False, False)),
    ],
    "stopped_car_keeps_the_brake_while_the_input_is_still_missing": [
        (0.0, False, 2.0, False, (True, False)),
        (1.0, False, 0.0, False, (True, True)),
        (5.0, False, 0.0, False, (True, True)),
        (5.1, True, 0.0, False, (False, False)),
    ],
    "an_engaged_brake_is_never_released_by_staleness": [
        (0.0, True, 0.0, True, (False, False)),
        (0.1, False, 0.0, True, (True, True)),
        (0.5, False, 0.0, False, (True, True)),
        (0.6, True, 0.0, False, (False, False)),
    ],
    # main D2: the engaged brake is held only while the input is stale.
    "engaged_brake_is_released_by_the_returned_command_while_rolling": [
        (0.0, True, 4.0, True, (False, False)),
        (0.3, False, 3.5, True, (True, True)),
        (0.5, True, 3.0, False, (False, False)),
        (0.7, True, 2.8, False, (False, False)),
    ],
    "unknown_speed_keeps_the_brake": [
        (0.0, False, None, False, (True, False)),
        (1.0, False, None, False, (True, True)),
        (1.1, True, None, False, (False, True)),
    ],
}


@pytest.mark.parametrize("name", sorted(CASES))
def test_input_loss_latch(name) -> None:
    latch = InputLossLatch(brake_after_sec=1.0, stopped_speed_mps=0.1)
    for now, ok, speed, engaged, expected in CASES[name]:
        state = latch.update(now, ok, speed, brake_engaged=engaged)
        assert (state.stale, state.brake) == expected, (name, now)
        assert state.active == any(expected)
