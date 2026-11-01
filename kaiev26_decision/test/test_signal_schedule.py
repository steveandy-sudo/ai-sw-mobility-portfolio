"""Signal input from the wall-clock schedule with the camera as a veto (signal_schedule.py).

Each test is a short situation and its outcome. Frames come at 30 Hz with distinct stamps.
"""
import math
from pathlib import Path
import sys

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

from kaiev26_decision.signal_phase_tracker import (  # noqa: E402
    LAMP_ARROW as A,
    LAMP_GREEN as G,
    LAMP_RED as R,
    LAMP_YELLOW as Y,
    TRAFFIC_ARROW,
    TRAFFIC_GREEN,
    TRAFFIC_RED,
    TRAFFIC_UNKNOWN,
    TRAFFIC_YELLOW,
)
from kaiev26_decision.signal_schedule import (  # noqa: E402
    SOURCE_CAMERA,
    SOURCE_DISTRUST,
    SOURCE_LIMIT,
    SOURCE_MARGIN,
    SOURCE_REJECT,
    SOURCE_SCHEDULE,
    SOURCE_UNSURE,
    ScheduleChangeClock,
    ScheduleConfig,
    ScheduleState,
    SignalSourceSelector,
)

DT = 1.0 / 30.0


class Approach:
    """One event's ticks: camera frames and the schedule the tracker publishes."""

    def __init__(self, maneuver="LEFT", config=None, event="Z4"):
        self.selector = SignalSourceSelector(config or ScheduleConfig())
        self.maneuver, self.event = maneuver, event
        self.t = 1000.0
        self.last = None

    def run(self, seconds, camera, schedule, arrow=False, stopped=False):
        """Ticks for this long; returns every SignalInput. stopped: standing at the event."""
        out = []
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            self.last = self.selector.update(
                self.t, self.event, self.maneuver, camera, arrow, schedule, self.t,
                stopped_at_line=stopped,
            )
            out.append(self.last)
        return out


PERMIT_375 = ScheduleState(True, R | A, 20.0)
RED_375 = ScheduleState(False, R, 20.0)
PERMIT_305 = ScheduleState(True, G | A, 5.0)
GREEN_STRAIGHT = ScheduleState(True, G, 20.0)
RED_STRAIGHT = ScheduleState(False, R, 20.0)


def test_375_red_read_while_the_schedule_permits_drives_on_the_schedule() -> None:
    # Stopped 36 m from 375, pv26 misses the arrow and reads red; the schedule says red + arrow.
    approach = Approach()
    first = approach.run(0.9, TRAFFIC_RED, PERMIT_375)
    assert {item.source for item in first} == {SOURCE_MARGIN}  # first sight: margin
    later = approach.run(8.0, TRAFFIC_RED, PERMIT_375)
    assert {item.source for item in later[4:]} == {SOURCE_SCHEDULE}
    assert (later[-1].state, later[-1].arrow_active) == (TRAFFIC_ARROW, True)


def test_schedule_red_keeps_a_false_arrow_from_launching() -> None:
    approach = Approach()
    approach.run(2.0, TRAFFIC_RED, RED_375)
    # A false red + arrow for a while (pv26 on 305 and in Gazebo on 375).
    inputs = approach.run(1.0, TRAFFIC_ARROW, RED_375, arrow=True)
    assert inputs[-1].source == SOURCE_SCHEDULE
    assert (inputs[-1].state, inputs[-1].arrow_active) == (TRAFFIC_RED, False)


def test_near_a_scheduled_change_the_camera_decides_and_a_stop_schedule_withholds() -> None:
    approach = Approach(maneuver="NONE", event="F5")
    approach.run(2.0, TRAFFIC_RED, RED_STRAIGHT)
    near = approach.run(0.3, TRAFFIC_GREEN, ScheduleState(False, R, 0.8))
    assert near[-1].source == SOURCE_MARGIN
    assert near[-1].state == TRAFFIC_GREEN and near[-1].schedule_forbids
    # Changed to green: the camera decides for one more second, without the withhold.
    after = approach.run(0.5, TRAFFIC_GREEN, ScheduleState(True, G, 39.0))
    assert after[-1].source == SOURCE_MARGIN and not after[-1].schedule_forbids
    settled = approach.run(1.0, TRAFFIC_GREEN, ScheduleState(True, G, 38.0))
    assert settled[-1].source == SOURCE_SCHEDULE


def test_sure_red_rejects_a_scheduled_green_arrow_until_the_permission_changes() -> None:
    # 305 shows green during its arrow; red there contradicts the schedule.
    approach = Approach(event="F3")
    inputs = approach.run(1.6, TRAFFIC_RED, PERMIT_305)
    assert inputs[-1].source == SOURCE_REJECT
    assert (inputs[-1].state, inputs[-1].arrow_active) == (TRAFFIC_RED, False)
    assert [item.started for item in inputs].count(SOURCE_REJECT) == 1
    # A green arrow reading later does not bring the schedule back within this permission.
    assert approach.run(0.5, TRAFFIC_GREEN, PERMIT_305, arrow=True)[-1].source == SOURCE_REJECT
    # The permission ends and comes back: the schedule is used again after the margin.
    approach.run(1.5, TRAFFIC_YELLOW, ScheduleState(False, Y, 2.5))
    back = approach.run(1.5, TRAFFIC_GREEN, PERMIT_305, arrow=True)
    assert back[-1].source == SOURCE_SCHEDULE


def test_a_reading_already_confirmed_is_taken_back_by_a_sure_opposite() -> None:
    approach = Approach(maneuver="NONE", event="F1")
    assert approach.run(3.0, TRAFFIC_GREEN, GREEN_STRAIGHT)[-1].state == TRAFFIC_GREEN
    inputs = approach.run(0.6, TRAFFIC_RED, GREEN_STRAIGHT)
    assert inputs[-1].source == SOURCE_REJECT and inputs[-1].state == TRAFFIC_RED


def test_short_opposite_runs_and_unknown_do_not_reject() -> None:
    approach = Approach(event="F3")
    approach.run(1.5, TRAFFIC_GREEN, PERMIT_305)
    for _ in range(10):
        approach.run(0.3, TRAFFIC_RED, PERMIT_305)  # 9 frames of red
        approach.run(0.1, TRAFFIC_GREEN, PERMIT_305, arrow=True)
    assert approach.last.source == SOURCE_SCHEDULE
    assert approach.run(5.0, TRAFFIC_UNKNOWN, PERMIT_305)[-1].source == SOURCE_SCHEDULE


def test_375_red_without_the_arrow_never_rejects_or_distrusts() -> None:
    # 103503: red with the arrow off for 5.6 s on end while the arrow was lit.
    approach = Approach()
    inputs = approach.run(10.0, TRAFFIC_RED, PERMIT_375)
    assert {item.source for item in inputs[60:]} == {SOURCE_SCHEDULE}


def test_yellow_while_375_should_show_its_arrow_rejects() -> None:
    approach = Approach()
    approach.run(1.5, TRAFFIC_RED, PERMIT_375)
    assert approach.run(0.6, TRAFFIC_YELLOW, PERMIT_375)[-1].source == SOURCE_REJECT


def test_long_go_against_a_red_schedule_drops_the_schedule_for_the_approach() -> None:
    approach = Approach(maneuver="NONE", event="F5")
    approach.run(1.5, TRAFFIC_RED, RED_STRAIGHT)
    inputs = approach.run(2.2, TRAFFIC_GREEN, RED_STRAIGHT)
    assert inputs[-1].source == SOURCE_DISTRUST and inputs[-1].state == TRAFFIC_GREEN
    assert [item.started for item in inputs].count(SOURCE_DISTRUST) == 1
    # Stays camera for the approach even when the schedule agrees again.
    assert approach.run(3.0, TRAFFIC_RED, GREEN_STRAIGHT)[-1].source == SOURCE_DISTRUST
    # The next event starts fresh.
    approach.event = "Q1"
    approach.run(1.5, TRAFFIC_GREEN, GREEN_STRAIGHT)
    assert approach.last.source == SOURCE_SCHEDULE


def test_long_opposite_against_a_permitting_schedule_drops_it() -> None:
    approach = Approach(maneuver="NONE", event="F1")
    approach.run(1.5, TRAFFIC_GREEN, GREEN_STRAIGHT)
    inputs = approach.run(3.3, TRAFFIC_RED, GREEN_STRAIGHT)
    assert SOURCE_REJECT in {item.source for item in inputs}
    assert inputs[-1].source == SOURCE_DISTRUST


def test_no_aligned_schedule_is_camera() -> None:
    approach = Approach()
    inputs = approach.run(2.0, TRAFFIC_RED, None)
    assert {item.source for item in inputs} == {SOURCE_CAMERA}
    assert approach.run(2.0, TRAFFIC_RED, ScheduleState(True, R | A, math.nan))[-1].source == SOURCE_MARGIN


def test_repeated_observation_stamps_count_once() -> None:
    selector = SignalSourceSelector()
    for index in range(60):
        t = 1000.0 + index * DT
        # One stale observation repeated on every tick: one frame only, no reject.
        result = selector.update(t, "F3", "LEFT", TRAFFIC_RED, False, PERMIT_305, 999.0)
    assert selector.opposite.frames <= 1
    assert result.source == SOURCE_SCHEDULE


def test_invalid_configuration_is_refused() -> None:
    with pytest.raises(ValueError):
        ScheduleConfig(reject_frames=0)
    with pytest.raises(ValueError):
        ScheduleConfig(transition_margin_sec=-1.0)


def test_a_long_false_arrow_over_a_scheduled_red_does_not_drop_the_schedule() -> None:
    # Gazebo 375 and the real 305 read red + arrow on a red-only head for seconds.
    approach = Approach()
    approach.run(2.0, TRAFFIC_RED, RED_375)
    inputs = approach.run(6.0, TRAFFIC_ARROW, RED_375, arrow=True)
    assert {item.source for item in inputs} == {SOURCE_SCHEDULE}
    assert (inputs[-1].state, inputs[-1].arrow_active) == (TRAFFIC_RED, False)


def test_a_false_arrow_over_a_scheduled_green_does_not_drop_the_schedule() -> None:
    # 305 close up reads green + arrow during its green-only phase.
    approach = Approach(event="F3")
    green_only = ScheduleState(False, G, 20.0)
    approach.run(2.0, TRAFFIC_GREEN, green_only)
    inputs = approach.run(4.0, TRAFFIC_GREEN, green_only, arrow=True)
    assert {item.source for item in inputs} == {SOURCE_SCHEDULE}
    assert (inputs[-1].state, inputs[-1].arrow_active) == (TRAFFIC_GREEN, False)


def test_a_green_arrow_over_a_scheduled_red_drops_the_schedule() -> None:
    # The operation changed: 305 shows green and arrow while the schedule says red.
    approach = Approach(event="F3")
    approach.run(2.0, TRAFFIC_RED, ScheduleState(False, R, 30.0))
    inputs = approach.run(2.3, TRAFFIC_GREEN, ScheduleState(False, R, 30.0), arrow=True)
    assert inputs[-1].source == SOURCE_DISTRUST and inputs[-1].arrow_active


def permit_375(probability, permitted_for):
    return ScheduleState(True, R | A, 20.0, probability, permitted_for)


# Stopped at 375 reading red while the schedule permits (real car 2026-10-17, kty 17d49de):
# situation: permission probability, permission left [s], standing at the line -> input.
LAUNCH_CASES = [
    pytest.param(0.95, 20.0, True, SOURCE_SCHEDULE, id="sure-schedule-launches"),
    pytest.param(0.6, 20.0, True, SOURCE_UNSURE, id="unsure-schedule-leaves-it-to-the-camera"),
    pytest.param(0.95, 2.0, True, SOURCE_UNSURE, id="ending-permission-leaves-it-to-the-camera"),
    pytest.param(math.nan, 20.0, True, SOURCE_UNSURE, id="unknown-probability-leaves-it-to-the-camera"),
    pytest.param(0.6, 20.0, False, SOURCE_SCHEDULE, id="moving-car-drives-on-the-schedule"),
]


@pytest.mark.parametrize("probability,permitted_for,stopped,source", LAUNCH_CASES)
def test_a_scheduled_permission_launches_from_a_stop_only_when_sure(
    probability, permitted_for, stopped, source
) -> None:
    approach = Approach()
    approach.run(2.0, TRAFFIC_RED, RED_375, stopped=stopped)
    inputs = approach.run(3.0, TRAFFIC_RED, permit_375(probability, permitted_for), stopped=stopped)
    assert inputs[-1].source == source
    if source == SOURCE_UNSURE:
        assert (inputs[-1].state, inputs[-1].arrow_active) == (TRAFFIC_RED, False)


# Stopped at 305 while the scheduled arrow ends (2026-10-17 14:57 F3: 0.4 s left at the stop, and
# the launch takes 3.2 s to the line): situation: permission left [s], standing at the line ->
# input and whether it withholds a launch.
ENDING_CASES = [
    pytest.param(0.4, True, SOURCE_MARGIN, True, id="near-change-0.4s-left-withholds"),
    pytest.param(3.5, True, SOURCE_UNSURE, True, id="3.5s-left-withholds"),
    pytest.param(4.5, True, SOURCE_SCHEDULE, False, id="4.5s-left-launches"),
    pytest.param(0.4, False, SOURCE_MARGIN, False, id="moving-car-is-not-held"),
]


@pytest.mark.parametrize("left,stopped,source,forbids", ENDING_CASES)
def test_a_stopped_car_does_not_launch_on_a_permission_ending_soon(left, stopped, source, forbids) -> None:
    approach = Approach(event="Z3")
    approach.run(2.0, TRAFFIC_GREEN, ScheduleState(False, G, 20.0), stopped=stopped)
    inputs = approach.run(
        1.5, TRAFFIC_GREEN, ScheduleState(True, G | A, left, 0.95, left), arrow=True, stopped=stopped
    )
    assert (inputs[-1].source, inputs[-1].schedule_forbids) == (source, forbids)


# The first sight of the schedule at an event: situation: the tracker's time since the change
# (NaN: unknown) -> the input of the first second (2026-10-17 14:57 F2 had 24 s of green behind).
@pytest.mark.parametrize("elapsed,source", [
    pytest.param(24.0, SOURCE_SCHEDULE, id="mid-phase-drives-on-the-schedule"),
    pytest.param(0.3, SOURCE_MARGIN, id="just-changed-leaves-it-to-the-camera"),
    pytest.param(math.nan, SOURCE_MARGIN, id="unknown-counts-as-a-change"),
])
def test_first_sight_uses_the_time_since_the_scheduled_change(elapsed, source) -> None:
    approach = Approach(maneuver="NONE", event="Z2")
    inputs = approach.run(0.5, TRAFFIC_UNKNOWN, ScheduleState(True, G, 7.7, 1.0, 7.7, elapsed))
    assert inputs[-1].source == source


def test_change_clock_knows_a_change_only_after_seeing_one() -> None:
    clock = ScheduleChangeClock(0.5)
    green, red = (True, G), (False, R)
    clock.observe(10.0, "143", True, red)
    clock.observe(10.1, "143", True, red)
    assert math.isnan(clock.elapsed("143", red, 10.1))  # seen only since the first estimate
    clock.observe(10.2, "143", True, green)
    clock.observe(10.3, "143", True, green)
    assert clock.elapsed("143", green, 12.2) == pytest.approx(2.0)
    assert math.isnan(clock.elapsed("143", red, 12.2))  # another key
    clock.observe(11.0, "143", True, green)  # a gap over 0.5 s: unknown again
    assert math.isnan(clock.elapsed("143", green, 11.0))
    clock.observe(11.1, "143", False, green)  # not aligned: forgotten
    assert "143" not in clock.lights


def test_the_schedule_holds_back_a_read_arrow_for_one_cycle_per_stop() -> None:
    # Stopped at 375, the camera reads red + arrow while the schedule keeps saying red.
    approach = Approach()
    approach.run(2.0, TRAFFIC_RED, RED_375, stopped=True)
    held = approach.run(99.5, TRAFFIC_ARROW, RED_375, arrow=True, stopped=True)
    assert {item.source for item in held} == {SOURCE_SCHEDULE}
    after = approach.run(1.0, TRAFFIC_ARROW, RED_375, arrow=True, stopped=True)
    assert after[-1].source == SOURCE_LIMIT and after[-1].arrow_active
    # For the rest of the approach the camera decides, also once the car moves.
    assert approach.run(1.0, TRAFFIC_RED, RED_375)[-1].source == SOURCE_LIMIT


def test_the_hold_limit_starts_over_after_the_car_moved() -> None:
    approach = Approach()
    approach.run(60.0, TRAFFIC_ARROW, RED_375, arrow=True, stopped=True)
    approach.run(1.0, TRAFFIC_ARROW, RED_375, arrow=True, stopped=False)
    inputs = approach.run(60.0, TRAFFIC_ARROW, RED_375, arrow=True, stopped=True)
    assert {item.source for item in inputs} == {SOURCE_SCHEDULE}
