"""Stop-point machine, event permission and event tracker: situation -> result tables.

Each scenario is a list of ticks (time s, rear-axle gap to the line m, measured speed m/s, light)
and the phase expected after the tick. Lights: RED, YELLOW, GREEN, UNKNOWN, ARROW (arrow only),
GREEN+A / RED+A (circle with a lit arrow), HINT_GO / HINT_STOP / HINT_FAR (own light UNKNOWN with a
signal phase tracker estimate: aligned permitted, aligned not permitted, not aligned).
DONE means the tracker has no current event any more.
"""
from functools import lru_cache
from pathlib import Path

import pytest

from kaiev26_decision.common import StoppingModel
from kaiev26_decision.event_permission import EventPermission, OwnLight, PhaseHint
from kaiev26_decision.mission_events import EventSettings, build_stop_events
from kaiev26_decision.route_zone_manager_node import load_landmark_stoplines, load_polyline_route
from kaiev26_decision.stop_point import (
    BRAKING,
    CLEARED,
    COMMITTED,
    FREE,
    HELD,
    KIND_FINISH,
    KIND_SIGNAL,
    KIND_TIMED,
    LAUNCH,
    WATCH,
    EventTracker,
    StopEvent,
    StopPointMachine,
    validate_stop_events,
)
from kaiev26_decision.zone_policy import load_zone_policy


PACKAGE = Path(__file__).resolve().parents[1]
FRONT = 1.98
STOPPING = StoppingModel()
OPTIONS = {
    "front_overhang_m": FRONT,
    "stopping_model": STOPPING,
    "stopped_speed_mps": 0.1,
    "pre_decel_window_m": 8.0,
    "pre_decel_cap_mps": 2.0,
}
LINE = 100.0
COURSE_SPEED = {"qualifying": 7.0, "final": 10.0}
ROUTES = {"qualifying": "kcity_quali_route.yaml", "final": "kcity_final_route.yaml"}


def signal(arrow="NONE", **changes):
    values = dict(event_id="SIG", kind=KIND_SIGNAL, line_s=LINE, approach_m=30.0,
                  brake_margin_m=1.0, stop_line_id="line", light_ids=("light",), arrow=arrow,
                  brake_wp_s=LINE - 10.0)
    values.update(changes)
    return StopEvent(**values)


def timed(**changes):
    values = dict(event_id="TIMED", kind=KIND_TIMED, line_s=LINE, approach_m=30.0,
                  brake_margin_m=1.0, hold_sec=3.0, brake_wp_s=LINE - 10.0)
    values.update(changes)
    return StopEvent(**values)


def finish(**changes):
    values = dict(event_id="FINISH", kind=KIND_FINISH, line_s=LINE, approach_m=20.0,
                  brake_margin_m=-5.0, hard_stop_rear_m=1.0, watch_cap_mps=0.6)
    values.update(changes)
    return StopEvent(**values)


def light_inputs(token):
    if token.startswith("HINT_"):
        hint = {"HINT_GO": PhaseHint(True, True), "HINT_STOP": PhaseHint(True, False),
                "HINT_FAR": PhaseHint(False, True)}[token]
        return OwnLight(), hint
    state, _, arrow = token.partition("+")
    return OwnLight(state, arrow == "A"), None


class Drive:
    def __init__(self, *events, **permission):
        self.tracker = EventTracker(tuple(events), dict(OPTIONS))
        self.permission = EventPermission(**permission)

    def tick(self, now, gap, speed, light="UNKNOWN", line_s=LINE):
        own, hint = light_inputs(light)
        s_rear = line_s - gap

        def permission_for(event, phase):
            is_signal = event.kind == KIND_SIGNAL
            return self.permission.update(
                now, event, phase, s_rear=s_rear, speed_mps=speed,
                light=own if is_signal else None, hint=hint if is_signal else None,
            )

        return self.tracker.update(s_rear, speed, permission_for)


def brake_gap(speed, margin=1.0):
    """Rear-axle gap at which the front bumper is d_stop(v) + margin from the line."""
    return STOPPING.distance_m(speed) + margin + FRONT


V8_BRAKE = brake_gap(8.0)        # 16.79 m
V8_DILEMMA = STOPPING.distance_m(8.0) + FRONT  # front gap below d_stop(8)

SIGNAL_SCENARIOS = {
    "red_brakes_holds_and_launches_on_confirmed_green": (signal(), [
        (0.0, 20.0, 1.0, "RED", WATCH),
        (0.1, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.2, 3.0, 0.5, "RED", BRAKING),
        (0.3, 2.9, 0.0, "RED", HELD),
        (1.0, 2.9, 0.0, "GREEN", HELD),
        (1.3, 2.9, 0.0, "GREEN", HELD),
        (1.4, 2.9, 0.0, "GREEN", LAUNCH),
        # D4: committed at launch, a red afterwards never brakes again for this event.
        (1.5, 2.8, 0.2, "RED", COMMITTED),
        (2.0, 1.0, 1.0, "RED", COMMITTED),
        (2.5, -0.1, 1.0, "RED", "DONE"),
    ]),
    "braking_cannot_release_before_the_measured_stop": (signal(), [
        (0.0, brake_gap(6.0) - 0.05, 6.0, "RED", BRAKING),
        (0.5, 8.0, 3.0, "GREEN", BRAKING),
        (1.5, 5.0, 0.5, "GREEN", BRAKING),
        (1.6, 5.0, 0.05, "GREEN", HELD),
        # The green confirmed while braking does not launch; it is confirmed again standing.
        (2.0, 5.0, 0.0, "GREEN", HELD),
        (2.39, 5.0, 0.0, "GREEN", HELD),
        (2.4, 5.0, 0.0, "GREEN", LAUNCH),
    ]),
    "confirmed_green_commits_only_inside_eight_metres": (signal(), [
        (0.0, 20.0, 5.0, "GREEN", WATCH),
        (0.4, 15.0, 5.0, "GREEN", WATCH),
        (0.5, 8.05, 5.0, "GREEN", WATCH),
        (0.6, 7.95, 5.0, "GREEN", COMMITTED),
        (0.7, 6.0, 5.0, "RED", COMMITTED),
    ]),
    "first_green_inside_the_commit_distance_waits_for_confirmation": (signal(), [
        (0.0, 7.5, 2.0, "GREEN", WATCH),
        (0.3, 7.0, 2.0, "GREEN", WATCH),
        (0.4, 6.8, 2.0, "GREEN", COMMITTED),
    ]),
    "unconfirmed_green_keeps_pre_deceleration_and_brakes": (signal(), [
        (0.0, 12.0, 2.0, "RED", WATCH),
        (0.1, 9.0, 2.0, "GREEN", WATCH),
        (0.2, brake_gap(2.0) - 0.05, 2.0, "RED", BRAKING),
    ]),
    "permission_far_from_the_line_can_change_to_red_before_commit": (signal(), [
        (0.0, 25.0, 5.0, "GREEN", WATCH),
        (0.5, 15.0, 5.0, "GREEN", WATCH),
        (0.6, brake_gap(5.0) + 0.05, 5.0, "RED", WATCH),
        (0.7, brake_gap(5.0) - 0.05, 5.0, "RED", BRAKING),
    ]),
    "yellow_after_permission_commits_when_the_stop_is_impossible": (signal(), [
        (0.0, 30.0, 8.0, "GREEN", WATCH),
        (0.5, 25.0, 8.0, "GREEN", WATCH),
        (0.6, V8_DILEMMA - 0.05, 8.0, "YELLOW", COMMITTED),
        (0.7, 9.0, 8.0, "RED", COMMITTED),
    ]),
    "yellow_after_permission_brakes_when_the_stop_is_possible": (signal(), [
        (0.0, 30.0, 8.0, "GREEN", WATCH),
        (0.5, 25.0, 8.0, "GREEN", WATCH),
        (0.6, V8_BRAKE + 0.05, 8.0, "YELLOW", WATCH),
        (0.7, V8_BRAKE - 0.05, 8.0, "YELLOW", BRAKING),
    ]),
    "unknown_after_permission_counts_as_stop_after_the_lapse_hold": (signal(), [
        (0.0, 30.0, 8.0, "GREEN", WATCH),
        (0.5, 25.0, 8.0, "GREEN", WATCH),
        (0.6, V8_BRAKE - 0.05, 8.0, "UNKNOWN", WATCH),
        (0.85, V8_BRAKE - 0.10, 8.0, "UNKNOWN", WATCH),
        (0.95, V8_BRAKE - 0.15, 8.0, "UNKNOWN", BRAKING),
    ]),
    "yellow_without_permission_brakes": (signal(), [
        (0.0, V8_DILEMMA - 0.05, 8.0, "YELLOW", BRAKING),
    ]),
    "red_after_permission_brakes_even_when_the_stop_is_impossible": (signal(), [
        (0.0, 30.0, 8.0, "GREEN", WATCH),
        (0.5, 25.0, 8.0, "GREEN", WATCH),
        (0.6, V8_DILEMMA - 0.05, 8.0, "RED", BRAKING),
    ]),
    "stopped_beyond_the_line_escapes_after_one_second_unknown": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, -0.5, 0.0, "RED", HELD),
        (0.2, -0.5, 0.0, "UNKNOWN", HELD),
        (1.1, -0.5, 0.0, "UNKNOWN", HELD),
        (1.2, -0.5, 0.0, "UNKNOWN", LAUNCH),
        (1.3, -0.6, 0.3, "UNKNOWN", "DONE"),
    ]),
    "stopped_slightly_over_the_line_keeps_holding_a_visible_red": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, -0.3, 0.0, "RED", HELD),
        (5.0, -0.3, 0.0, "RED", HELD),
        (60.0, -0.3, 0.0, "RED", HELD),
    ]),
    "unknown_before_the_line_waits_110_s_then_departs": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, 2.9, 0.0, "UNKNOWN", HELD),
        (60.0, 2.9, 0.0, "UNKNOWN", HELD),
        (110.05, 2.9, 0.0, "UNKNOWN", HELD),
        (110.1, 2.9, 0.0, "UNKNOWN", LAUNCH),
        (110.2, 2.8, 0.3, "RED", COMMITTED),
    ]),
    "a_valid_stop_signal_restarts_the_110_s_wait": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, 2.9, 0.0, "UNKNOWN", HELD),
        (50.0, 2.9, 0.0, "RED", HELD),
        (50.1, 2.9, 0.0, "UNKNOWN", HELD),
        (110.2, 2.9, 0.0, "UNKNOWN", HELD),
        (160.1, 2.9, 0.0, "UNKNOWN", LAUNCH),
    ]),
    "clock_rewind_restarts_the_green_confirmation": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, 2.9, 0.0, "RED", HELD),
        (1.0, 2.9, 0.0, "GREEN", HELD),
        (1.3, 2.9, 0.0, "GREEN", HELD),
        (0.5, 2.9, 0.0, "GREEN", HELD),
        (0.8, 2.9, 0.0, "GREEN", HELD),
        (0.9, 2.9, 0.0, "GREEN", LAUNCH),
    ]),
    "phase_tracker_permission_stands_in_for_an_unknown_own_light": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "HINT_STOP", BRAKING),
        (0.1, -0.5, 0.0, "HINT_STOP", HELD),
        # Aligned and not permitted: no UNKNOWN escape although the car is past the line.
        (2.0, -0.5, 0.0, "HINT_STOP", HELD),
        (3.0, -0.5, 0.0, "HINT_GO", HELD),
        (3.4, -0.5, 0.0, "HINT_GO", LAUNCH),
    ]),
    "unaligned_phase_tracker_falls_back_to_the_unknown_escape": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, -0.5, 0.0, "HINT_FAR", HELD),
        (1.05, -0.5, 0.0, "HINT_FAR", HELD),
        (1.15, -0.5, 0.0, "HINT_FAR", LAUNCH),
    ]),
    # The estimate stands in only stopped past the line; before it an UNKNOWN own light is a stop.
    "phase_tracker_is_not_used_on_a_moving_approach": (signal(), [
        (0.0, 20.0, 5.0, "HINT_GO", WATCH),
        (0.5, 7.9, 5.0, "HINT_GO", BRAKING),
    ]),
    "phase_tracker_is_not_used_stopped_before_the_line": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "HINT_GO", BRAKING),
        (0.1, 2.9, 0.0, "HINT_GO", HELD),
        (2.0, 2.9, 0.0, "HINT_GO", HELD),
    ]),
    "left_turn_event_requires_the_arrow_on_a_general_green": (signal(arrow="LEFT"), [
        (0.0, 20.0, 2.0, "GREEN", WATCH),
        (0.5, brake_gap(2.0) - 0.05, 2.0, "GREEN", BRAKING),
        (0.6, 3.0, 0.0, "GREEN", HELD),
        (1.0, 3.0, 0.0, "RED+A", HELD),
        (1.4, 3.0, 0.0, "RED+A", LAUNCH),
    ]),
    "left_turn_event_goes_on_an_arrow_only_reading": (signal(arrow="LEFT"), [
        (0.0, 20.0, 5.0, "ARROW", WATCH),
        (0.5, 7.9, 5.0, "ARROW", COMMITTED),
    ]),
    "short_arrow_dropouts_do_not_stop_a_released_car": (signal(arrow="LEFT"), [
        (0.0, 25.0, 5.0, "RED+A", WATCH),
        (0.5, 20.0, 5.0, "RED+A", WATCH),
        (0.6, brake_gap(5.0) - 0.05, 5.0, "RED", WATCH),
        (0.8, brake_gap(5.0) - 0.10, 5.0, "RED+A", WATCH),
        (0.9, 7.9, 5.0, "RED+A", COMMITTED),
    ]),
    "arrow_off_and_unknown_share_one_hold_timer": (signal(arrow="LEFT"), [
        (0.0, 25.0, 5.0, "RED+A", WATCH),
        (0.5, 20.0, 5.0, "RED+A", WATCH),
        (0.6, brake_gap(5.0) - 0.05, 5.0, "RED", WATCH),
        (0.75, brake_gap(5.0) - 0.10, 5.0, "UNKNOWN", WATCH),
        (0.95, brake_gap(5.0) - 0.15, 5.0, "UNKNOWN", BRAKING),
    ]),
    "arrow_off_is_not_held_without_a_confirmed_arrow": (signal(arrow="LEFT"), [
        (0.0, 25.0, 5.0, "RED+A", WATCH),
        (0.1, brake_gap(5.0) - 0.05, 5.0, "RED", BRAKING),
    ]),
    "straight_event_goes_on_green_with_the_arrow_also_on": (signal(), [
        (0.0, 20.0, 5.0, "GREEN+A", WATCH),
        (0.5, 7.9, 5.0, "GREEN+A", COMMITTED),
    ]),
    "straight_event_does_not_go_on_an_arrow_only_reading": (signal(), [
        (0.0, 20.0, 5.0, "ARROW", WATCH),
        (0.5, brake_gap(5.0) - 0.05, 5.0, "ARROW", BRAKING),
    ]),
    # 2026-10-17 145703 F2: before the confirmation, an UNKNOWN of up to 0.3 s pauses the 0.4 s
    # green confirmation instead of restarting it; a longer one restarts it.
    "short_unknown_pauses_the_green_confirmation": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, 3.0, 0.0, "RED", HELD),
        (1.0, 3.0, 0.0, "GREEN", HELD),
        (1.2, 3.0, 0.0, "UNKNOWN", HELD),
        (1.4, 3.0, 0.0, "GREEN", LAUNCH),
    ]),
    "long_unknown_restarts_the_green_confirmation": (signal(), [
        (0.0, brake_gap(1.0) - 0.05, 1.0, "RED", BRAKING),
        (0.1, 3.0, 0.0, "RED", HELD),
        (1.0, 3.0, 0.0, "GREEN", HELD),
        (1.2, 3.0, 0.0, "UNKNOWN", HELD),
        (1.55, 3.0, 0.0, "UNKNOWN", HELD),
        (1.6, 3.0, 0.0, "GREEN", HELD),
        (1.9, 3.0, 0.0, "GREEN", HELD),
        (2.0, 3.0, 0.0, "GREEN", LAUNCH),
    ]),
}

TIMED_SCENARIOS = {
    "timed_stop_counts_three_seconds_after_the_measured_stop": (timed(), [
        (0.0, 20.0, 6.0, "UNKNOWN", WATCH),
        (0.1, brake_gap(6.0) - 0.05, 6.0, "UNKNOWN", BRAKING),
        (1.5, 2.0, 0.5, "UNKNOWN", BRAKING),
        (2.0, 1.8, 0.0, "UNKNOWN", HELD),
        (4.9, 1.8, 0.0, "UNKNOWN", HELD),
        (5.0, 1.8, 0.0, "UNKNOWN", LAUNCH),
        (5.1, 1.7, 0.3, "UNKNOWN", COMMITTED),
        (6.0, -0.1, 1.0, "UNKNOWN", "DONE"),
    ]),
    "speed_noise_above_the_stopped_threshold_restarts_the_timer": (timed(), [
        (0.0, brake_gap(2.0) - 0.05, 2.0, "UNKNOWN", BRAKING),
        (0.5, 3.0, 0.0, "UNKNOWN", HELD),
        (2.0, 3.0, 0.15, "UNKNOWN", HELD),
        (2.1, 3.0, 0.0, "UNKNOWN", HELD),
        (5.0, 3.0, 0.0, "UNKNOWN", HELD),
        (5.1, 3.0, 0.0, "UNKNOWN", LAUNCH),
    ]),
    "timed_stop_never_brakes_again_after_release": (timed(), [
        (0.0, brake_gap(2.0) - 0.05, 2.0, "UNKNOWN", BRAKING),
        (0.5, 3.0, 0.0, "UNKNOWN", HELD),
        (3.5, 3.0, 0.0, "UNKNOWN", LAUNCH),
        (3.6, 2.9, 0.4, "UNKNOWN", COMMITTED),
        (3.7, 2.8, 0.0, "UNKNOWN", COMMITTED),
        (9.0, 2.8, 0.0, "UNKNOWN", COMMITTED),
    ]),
}

FINISH_SCENARIOS = {
    "slow_finish_approach_stops_with_the_rear_axle_one_metre_before_the_end": (finish(), [
        (0.0, 25.0, 0.6, "UNKNOWN", FREE),
        (0.1, 19.0, 0.6, "UNKNOWN", WATCH),
        (0.2, 1.05, 0.6, "UNKNOWN", WATCH),
        (0.3, 0.95, 0.6, "UNKNOWN", BRAKING),
        (0.5, 0.8, 0.0, "UNKNOWN", HELD),
        (200.0, 0.8, 0.0, "GREEN", HELD),
    ]),
    "fast_finish_approach_brakes_so_the_front_stops_within_the_overrun": (finish(), [
        (0.0, 19.0, 8.0, "UNKNOWN", WATCH),
        (0.1, brake_gap(8.0, -5.0) + 0.05, 8.0, "UNKNOWN", WATCH),
        (0.2, brake_gap(8.0, -5.0) - 0.05, 8.0, "UNKNOWN", BRAKING),
        (3.0, 0.0, 0.0, "UNKNOWN", HELD),
    ]),
}

ALL_SCENARIOS = {**SIGNAL_SCENARIOS, **TIMED_SCENARIOS, **FINISH_SCENARIOS}


@pytest.mark.parametrize("name", sorted(ALL_SCENARIOS))
def test_stop_point_scenario(name) -> None:
    event, ticks = ALL_SCENARIOS[name]
    drive = Drive(event)
    for now, gap, speed, light, expected in ticks:
        output = drive.tick(now, gap, speed, light)
        phase = "DONE" if output is None else output.phase
        assert phase == expected, (name, now, output)
        if output is not None:
            assert output.stop == (phase in {BRAKING, HELD}), (name, now)


def test_unknown_wait_limit_reports_its_tag_once_released() -> None:
    drive = Drive(signal())
    drive.tick(0.0, brake_gap(1.0) - 0.05, 1.0, "RED")
    drive.tick(0.1, 2.9, 0.0, "UNKNOWN")
    output = drive.tick(110.2, 2.9, 0.0, "UNKNOWN")
    assert output.phase == LAUNCH
    assert "SIGNAL_UNKNOWN_TIMEOUT" in output.tags
    assert drive.permission.unknown_waited_sec == pytest.approx(110.1)


def test_zero_unknown_wait_limit_waits_indefinitely() -> None:
    drive = Drive(signal(), unknown_wait_limit_sec=0.0)
    drive.tick(0.0, brake_gap(1.0) - 0.05, 1.0, "RED")
    for now in (0.1, 200.0, 1000.0):
        assert drive.tick(now, 2.9, 0.0, "UNKNOWN").phase == HELD


# Tags motion_control and the TUI read, by phase.
TAG_CASES = [
    (signal(), (0.0, 9.0, 2.0, "RED"), WATCH, "SIGNAL_BRAKE_TRIGGER_PRE_DECEL"),
    (signal(), (0.0, brake_gap(2.0) - 0.05, 2.0, "RED"), BRAKING, "SIGNAL_HARD_BRAKE"),
    (timed(), (0.0, 9.0, 2.0, "UNKNOWN"), WATCH, "BRAKE_TRIGGER_PRE_DECEL"),
    (finish(), (0.0, 10.0, 0.6, "UNKNOWN"), WATCH, "FINISH_DECEL"),
    (finish(), (0.0, 0.5, 0.6, "UNKNOWN"), BRAKING, "FINISH_STOP"),
    (finish(), (0.0, 0.5, 0.0, "UNKNOWN"), HELD, "FINISH_HELD"),
]


@pytest.mark.parametrize("event, tick, phase, tag", TAG_CASES)
def test_phase_tags(event, tick, phase, tag) -> None:
    output = Drive(event).tick(*tick)
    assert output.phase == phase
    assert tag in output.tags


# fsm_state shown to people by kind, phase and permission (docs/ARCHITECTURE.md).
STATE_SCENARIOS = {
    "signal_red": (signal(), [
        (0.0, 20.0, 1.0, "RED", "SIGNAL_APPROACH_STOP"),
        (0.05, 9.0, 1.0, "RED", "SIGNAL_PRE_BRAKE"),
        (0.1, brake_gap(1.0) - 0.05, 1.0, "RED", "SIGNAL_BRAKE"),
        (0.3, 2.9, 0.0, "RED", "SIGNAL_WAIT"),
        (1.0, 2.9, 0.0, "GREEN", "SIGNAL_WAIT"),
        (1.4, 2.9, 0.0, "GREEN", "SIGNAL_PASS"),
        (1.5, 2.8, 0.2, "RED", "SIGNAL_PASS"),
    ]),
    "signal_green": (signal(), [
        (0.0, 20.0, 5.0, "GREEN", "SIGNAL_APPROACH_STOP"),
        (0.4, 15.0, 5.0, "GREEN", "SIGNAL_APPROACH_GO"),
        (0.6, 7.95, 5.0, "GREEN", "SIGNAL_PASS"),
    ]),
    "timed_stop": (timed(), [
        (0.0, 20.0, 2.0, "UNKNOWN", "TIMED_STOP_APPROACH"),
        (0.1, 9.0, 2.0, "UNKNOWN", "TIMED_STOP_PRE_BRAKE"),
        (0.2, brake_gap(2.0) - 0.05, 2.0, "UNKNOWN", "TIMED_STOP_BRAKE"),
        (0.7, 3.0, 0.0, "UNKNOWN", "TIMED_STOP_WAIT"),
        (3.7, 3.0, 0.0, "UNKNOWN", "TIMED_STOP_PASS"),
        (3.8, 2.9, 0.4, "UNKNOWN", "TIMED_STOP_PASS"),
    ]),
    "finish": (finish(), [
        (0.1, 19.0, 0.6, "UNKNOWN", "FINISH_APPROACH"),
        (0.3, 0.95, 0.6, "UNKNOWN", "FINISH_BRAKE"),
        (0.5, 0.8, 0.0, "UNKNOWN", "FINISH_STOPPED"),
    ]),
}


@pytest.mark.parametrize("name", sorted(STATE_SCENARIOS))
def test_shown_state_names_the_kind_phase_and_permission(name) -> None:
    event, ticks = STATE_SCENARIOS[name]
    drive = Drive(event)
    for now, gap, speed, light, expected in ticks:
        assert drive.tick(now, gap, speed, light).state == expected, (name, now)


# (gap, permission light, expected cap): PRE_DECEL is the 2 m/s cap from 8 m before the brake WP.
PRE_DECEL_CASES = [
    (18.05, "RED", None),
    (17.95, "RED", 2.0),
    (12.0, "RED", 2.0),
    # An unconfirmed green keeps the pre-deceleration.
    (12.0, "GREEN", 2.0),
]


@pytest.mark.parametrize("gap, light, cap", PRE_DECEL_CASES)
def test_pre_deceleration_window_is_unchanged(gap, light, cap) -> None:
    output = Drive(signal()).tick(0.0, gap, 1.0, light)
    assert output.phase == WATCH
    assert output.cap_mps == cap


def test_confirmed_green_lifts_the_pre_deceleration() -> None:
    drive = Drive(signal())
    drive.tick(0.0, 13.0, 1.0, "GREEN")
    output = drive.tick(0.4, 12.6, 1.0, "GREEN")
    assert (output.phase, output.cap_mps) == (WATCH, None)


# Event tracker: the next event starts only after the current one is CLEARED.
def test_tracker_never_advances_while_held_after_an_overshoot() -> None:
    first, second = signal(event_id="A"), timed(event_id="B", line_s=LINE + 40.0)
    drive = Drive(first, second)
    drive.tick(0.0, brake_gap(1.0) - 0.05, 1.0, "RED")
    # Stopped 2 m past the line, inside the next event's approach window.
    for now in (0.1, 30.0, 100.0):
        output = drive.tick(now, -2.0, 0.0, "RED")
        assert (output.event.event_id, output.phase) == ("A", HELD)
    assert drive.tracker.cleared_ids == []
    drive.tick(100.5, -2.0, 0.0, "GREEN")
    output = drive.tick(100.9, -2.0, 0.0, "GREEN")
    assert output.phase == LAUNCH
    output = drive.tick(101.0, -2.1, 0.3, "GREEN")
    assert (output.event.event_id, output.phase) == ("B", FREE)
    assert drive.tracker.cleared_ids == ["A"]


def test_tracker_passes_an_event_driven_through_on_permission() -> None:
    drive = Drive(signal(event_id="A"), finish(line_s=LINE + 60.0))
    drive.tick(0.0, 20.0, 5.0, "GREEN")
    drive.tick(0.5, 7.0, 5.0, "GREEN")
    output = drive.tick(1.0, -0.5, 5.0, "RED")
    assert output.event.kind == KIND_FINISH
    assert drive.tracker.cleared_ids == ["A"]


# hold_output while inputs are stale: (phase reached, stop requested)
HOLD_CASES = [
    ((0.0, 20.0, 5.0, "RED"), WATCH, False),
    ((0.0, brake_gap(5.0) - 0.05, 5.0, "RED"), BRAKING, True),
    ((0.0, 1.0, 0.0, "RED"), HELD, True),
]


@pytest.mark.parametrize("tick, phase, stop", HOLD_CASES)
def test_stale_inputs_freeze_the_phase_and_keep_any_brake(tick, phase, stop) -> None:
    drive = Drive(signal())
    if phase == HELD:
        drive.tick(-0.1, brake_gap(1.0) - 0.05, 1.0, "RED")
    assert drive.tick(*tick).phase == phase
    held = drive.tracker.hold_output()
    assert (held.phase, held.stop) == (phase, stop)


# hold_output with a measured speed: (speed passed while stale, phase after)
HOLD_SPEED_CASES = [
    (None, BRAKING),
    (2.0, BRAKING),
    (0.05, HELD),
]


@pytest.mark.parametrize("speed, phase", HOLD_SPEED_CASES)
def test_stale_route_still_lets_a_braking_machine_reach_held(speed, phase) -> None:
    drive = Drive(signal())
    assert drive.tick(0.0, brake_gap(5.0) - 0.05, 5.0, "RED").phase == BRAKING
    held = drive.tracker.hold_output(speed)
    assert (held.phase, held.stop) == (phase, True)


def test_manual_reset_restarts_only_the_current_event() -> None:
    drive = Drive(signal(event_id="A"), signal(event_id="B", line_s=LINE + 50.0))
    drive.tick(0.0, 20.0, 5.0, "GREEN")
    drive.tick(0.5, 7.0, 5.0, "GREEN")
    drive.tick(1.0, -0.5, 5.0, "GREEN")
    drive.tick(1.1, 40.0, 5.0, "RED", line_s=LINE + 50.0)
    drive.tracker.reset_current()
    assert drive.tracker.phase == FREE
    assert drive.tracker.current.event_id == "B"
    assert drive.tracker.cleared_ids == ["A"]


@pytest.mark.parametrize("second_line, ok", [(LINE + 30.0, True), (LINE + 29.9, False)])
def test_overlapping_approach_windows_are_refused(second_line, ok) -> None:
    events = [signal(event_id="A"), timed(event_id="B", line_s=second_line)]
    if ok:
        validate_stop_events(events)
    else:
        with pytest.raises(ValueError, match="approach"):
            validate_stop_events(events)


# Real course events ------------------------------------------------------------------------

@lru_cache(maxsize=None)
def course_events(course, mode="all"):
    route, _ = load_polyline_route(str(PACKAGE / "waypoints" / ROUTES[course]))
    stoplines, _ = load_landmark_stoplines(route, PACKAGE / "config" / f"{course}_landmarks.yaml")
    policy = load_zone_policy(PACKAGE / "config" / f"{course}_policy.yaml")
    return build_stop_events(
        policy, {item.landmark_id: item.progress_s for item in stoplines}, route.total_length, mode,
    )


# (course, mode) -> kinds in route order; FINISH always ends the run.
EVENT_TABLE = {
    ("qualifying", "all"): ("TIMED_STOP", "SIGNAL_OBEY", "TIMED_STOP", "SIGNAL_OBEY", "FINISH"),
    ("qualifying", "stopline_signal"): ("TIMED_STOP", "SIGNAL_OBEY", "TIMED_STOP", "SIGNAL_OBEY", "FINISH"),
    ("qualifying", "stopline"): ("TIMED_STOP",) * 4 + ("FINISH",),
    ("qualifying", "tracking"): ("FINISH",),
    ("final", "all"): ("SIGNAL_OBEY",) * 5 + ("FINISH",),
    ("final", "stopline"): ("TIMED_STOP",) * 5 + ("FINISH",),
    ("final", "avoidance"): ("FINISH",),
    ("final", "tracking"): ("FINISH",),
}


@pytest.mark.parametrize("course, mode", sorted(EVENT_TABLE))
def test_course_events_per_mode(course, mode) -> None:
    events = course_events(course, mode)
    assert tuple(event.kind for event in events) == EVENT_TABLE[(course, mode)]
    if mode == "stopline":
        assert all(e.hold_sec == 3.0 and not e.light_ids for e in events if e.kind == KIND_TIMED)
    assert [e.arrow for e in course_events("final")][:5] == ["NONE", "NONE", "LEFT", "LEFT", "NONE"]


# 2026-10-25 surveyed line: route s where the route crosses it, the brake station with the node's
# antenna offset (0.426 m) and pass margin (1.0 m), and the route end.
SURVEYED_FINISH = {
    "qualifying": (555.67, 557.244, 596.13),
    "final": (731.93, 733.503, 772.39),
}


@pytest.mark.parametrize("course", sorted(SURVEYED_FINISH))
def test_course_finish_brakes_past_the_surveyed_line_at_any_speed(course) -> None:
    line_s, brake_s, end_s = SURVEYED_FINISH[course]
    route, _ = load_polyline_route(str(PACKAGE / "waypoints" / ROUTES[course]))
    policy = load_zone_policy(PACKAGE / "config" / f"{course}_policy.yaml")
    settings = EventSettings(finish_line_reference_forward_m=0.426, finish_line_pass_margin_m=2.0)
    finish = build_stop_events(policy, {}, route.total_length, "tracking", settings)[-1]
    assert finish.kind == KIND_FINISH
    assert finish.line_s == pytest.approx(end_s, abs=0.01)
    assert finish.brake_s == pytest.approx(brake_s, abs=0.001)
    assert finish.watch_cap_mps is None
    machine = StopPointMachine(finish, **OPTIONS)
    for speed in (5.0, 10.0, 13.0):
        assert machine.brake_point_s(speed) == pytest.approx(brake_s)
    # The antenna is past the line before the rear axle reaches the brake station.
    assert brake_s + 0.426 - line_s == pytest.approx(2.0, abs=0.01)

    drive = Drive(finish)
    for now, s_rear, speed, phase in [
        (0.0, brake_s - 1.5, 13.0, WATCH),
        (0.1, brake_s - 0.05, 13.0, WATCH),
        (0.2, brake_s + 0.05, 13.0, BRAKING),
        (3.0, brake_s + 25.0, 0.0, HELD),
    ]:
        output = drive.tick(now, finish.line_s - s_rear, speed, line_s=finish.line_s)
        assert output.phase == phase, (now, output)
        # No finish slow-down before the brake: the course speed holds up to the station.
        assert output.cap_mps is None, (now, output)


def test_surveyed_finish_with_no_route_left_past_its_brake_station_is_refused() -> None:
    policy = load_zone_policy(PACKAGE / "config" / "final_policy.yaml")
    with pytest.raises(ValueError, match="not before the route end"):
        build_stop_events(policy, {}, 732.0, "tracking")


def test_brake_station_outside_the_finish_approach_is_refused() -> None:
    with pytest.raises(ValueError, match="inside its approach"):
        validate_stop_events([finish(brake_s=LINE - 25.0)])


STOP_EVENTS = [
    pytest.param(course, index, id=event.event_id + "_" + course)
    for course in ROUTES
    for index, event in enumerate(course_events(course))
    if event.kind in {KIND_SIGNAL, KIND_TIMED}
]


@pytest.mark.parametrize("course, index", STOP_EVENTS)
@pytest.mark.parametrize("speed", [2.0, 6.0, 8.0, 10.0])
def test_course_stop_brakes_at_the_stopping_distance(course, index, speed) -> None:
    event = course_events(course)[index]
    drive = Drive(event)
    gap = event.approach_m - 0.01
    while gap > 0.0:
        output = drive.tick(0.0, gap, speed, "RED", line_s=event.line_s)
        if output.phase == BRAKING:
            break
        assert output.phase == WATCH
        gap -= 0.05
    assert gap == pytest.approx(brake_gap(speed, event.brake_margin_m), abs=0.051)


@pytest.mark.parametrize("course, index", STOP_EVENTS)
def test_course_stop_requests_two_mps_from_the_brake_waypoint_minus_eight(course, index) -> None:
    event = course_events(course)[index]
    drive = Drive(event)
    speed = COURSE_SPEED[course]
    gap = event.approach_m - 0.01
    while gap > 0.0:
        output = drive.tick(0.0, gap, speed, "RED", line_s=event.line_s)
        if output.phase == BRAKING:
            break
        in_window = event.brake_wp_s - (event.line_s - gap) <= 8.0
        assert output.cap_mps == (2.0 if in_window else None), gap
        gap -= 0.05


@pytest.mark.parametrize("course, index", [p for p in STOP_EVENTS if "SIGNAL" in p.id])
@pytest.mark.parametrize("light", ["RED", "YELLOW", "UNKNOWN"])
def test_course_signal_commits_at_eight_metres_only_with_permission(course, index, light) -> None:
    event = course_events(course)[index]
    go = "ARROW" if event.arrow == "LEFT" else "GREEN"
    for state, expected in ((go, COMMITTED), (light, BRAKING)):
        drive = Drive(event)
        drive.tick(0.0, 20.0, 1.0, state, line_s=event.line_s)
        drive.tick(0.5, 8.05, 1.0, state, line_s=event.line_s)
        output = drive.tick(0.6, 7.95, 1.0, state, line_s=event.line_s)
        if expected == COMMITTED:
            assert output.phase == COMMITTED
        else:
            assert output.phase in {WATCH, BRAKING}
            output = drive.tick(0.7, min(7.9, brake_gap(1.0, event.brake_margin_m) - 0.05), 1.0,
                                state, line_s=event.line_s)
            assert output.phase == BRAKING


def test_cleared_phase_is_never_returned_as_the_current_event() -> None:
    drive = Drive(signal())
    assert drive.tick(0.0, -1.0, 5.0, "RED") is None
    assert drive.tracker.phase == CLEARED
