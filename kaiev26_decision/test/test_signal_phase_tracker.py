from itertools import product
import math
from pathlib import Path
import sys

import numpy as np
import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR))

from kaiev26_decision.signal_phase_tracker import (  # noqa: E402
    LAMP_ARROW,
    LAMP_GREEN,
    LAMP_RED,
    LAMP_YELLOW,
    TRAFFIC_UNKNOWN,
    MapSignals,
    MapTrafficLight,
    ProgramPhase,
    RouteSignal,
    SignalPhaseTracker,
    SignalProgram,
    TrackerConfig,
    build_light_cycle,
    lamps_permit,
    lamps_scene_state,
    load_map_signals,
    route_signals_from_policy,
)
from kaiev26_decision.zone_policy import load_zone_policy  # noqa: E402


KCITY_SEMANTICS = (
    PACKAGE_DIR.parents[1] / "Simulation/kaiev26_maps/maps/kcity/kcity_course_semantics.yaml"
)
G, Y, R, A = LAMP_GREEN, LAMP_YELLOW, LAMP_RED, LAMP_ARROW


def program(program_id: str, light_id: str, phases, offset_s: float = 0.0, period_s: float = 100.0):
    return SignalProgram(
        program_id=program_id,
        period_s=period_s,
        offset_s=offset_s,
        phases=tuple(ProgramPhase(duration, {light_id: lamps}) for duration, lamps in phases),
    )


def light(light_id: str, controller_id: str = "") -> MapTrafficLight:
    return MapTrafficLight(light_id=light_id, x=0.0, y=0.0, z=4.9, yaw=0.0, controller_id=controller_id)


STRAIGHT = ((32.0, G | A), (3.0, Y), (65.0, R))
FINAL_LEFT = ((35.0, G), (12.0, G | A), (3.0, Y), (50.0, R))


def single_tracker(phases=STRAIGHT, config: TrackerConfig | None = None, route=()):
    signals = MapSignals(lights=(light("L1", "c1"),), programs=(program("p1", "L1", phases),))
    return SignalPhaseTracker(signals, route, config or TrackerConfig())


def feed(tracker, light_id, truth_program, theta, start, end, rate_hz=30.0, light_name=None):
    """Perfect observations of the truth program shifted by theta, as the gateway reports them."""
    cycle = build_light_cycle(truth_program, light_name or light_id)
    for t in np.arange(start, end, 1.0 / rate_hz):
        lamps = cycle.lamps_at((t + truth_program.offset_s + theta) % truth_program.period_s)
        state, arrow = lamps_scene_state(lamps)
        tracker.observe(light_id, float(t), state, arrow, lamps)


def truth_now(truth_program, light_id, theta, now):
    cycle = build_light_cycle(truth_program, light_id)
    position = (now + truth_program.offset_s + theta) % truth_program.period_s
    segment = int(cycle.segment_index(np.asarray([position]))[0])
    remaining = cycle.lamp_end[segment] - position
    return int(cycle.lamps[segment]), float(remaining)


def estimate_of(tracker, light_id, now):
    return next(item for item in tracker.estimate(now) if item.light_id == light_id)


def test_one_transition_collapses_alignment_to_a_narrow_band() -> None:
    tracker = single_tracker()
    truth = program("p1", "L1", STRAIGHT)
    theta = 37.33
    # Cycle position (t + theta) mod 100: red until 100, green from there.
    start = 1000.0 + (60.0 - theta)
    feed(tracker, "L1", truth, theta, start, start + 20.0)
    steady = estimate_of(tracker, "L1", start + 20.0)
    assert steady.predicted_lamps == R
    assert not steady.aligned
    # A steady red only says the cycle is somewhere in the remaining red phase.
    assert steady.remaining_max_sec - steady.remaining_min_sec > 15.0

    # Red ends at cycle position 100, 40 s after the start; then 5 s of green and arrow.
    feed(tracker, "L1", truth, theta, start + 20.0, start + 45.0)
    now = start + 45.0
    estimate = estimate_of(tracker, "L1", now)
    lamps, remaining = truth_now(truth, "L1", theta, now)
    assert estimate.aligned
    assert estimate.alignment_width_sec <= 1.5
    assert estimate.predicted_lamps == lamps == G | A
    assert estimate.remaining_sec == pytest.approx(remaining, abs=0.3)
    assert estimate.remaining_min_sec <= remaining <= estimate.remaining_max_sec
    assert estimate.next_lamps == Y
    assert estimate.cycle_time_sec == pytest.approx((now + theta) % 100.0, abs=0.3)


def test_alignment_runs_forward_across_unobserved_time() -> None:
    tracker = single_tracker()
    truth = program("p1", "L1", STRAIGHT)
    theta = 12.7
    start = 500.0 + (95.0 - theta)
    feed(tracker, "L1", truth, theta, start, start + 10.0)
    for now in np.arange(start + 10.0, start + 1200.0, 7.3):
        estimate = estimate_of(tracker, "L1", float(now))
        lamps, remaining = truth_now(truth, "L1", theta, float(now))
        assert estimate.aligned
        if remaining > 0.5:
            assert estimate.predicted_lamps == lamps
            assert estimate.remaining_sec == pytest.approx(remaining, abs=0.4)
    assert estimate.observation_age_sec > 1000.0


def coupled_signals():
    lights = (light("A", "central"), light("B", "central"))
    programs = (
        program("pa", "A", STRAIGHT, offset_s=0.0),
        program("pb", "B", ((50.0, R), (32.0, G | A), (3.0, Y), (15.0, R)), offset_s=20.0),
    )
    return MapSignals(lights=lights, programs=programs)


def test_controller_coupling_predicts_an_unobserved_light() -> None:
    signals = coupled_signals()
    theta = 41.0
    truth_a, truth_b = signals.programs
    start = 2000.0 + (60.0 - theta)

    coupled = SignalPhaseTracker(signals, (), TrackerConfig(coupling="controller"))
    alone = SignalPhaseTracker(signals, (), TrackerConfig(coupling="program"))
    for tracker in (coupled, alone):
        feed(tracker, "A", truth_a, theta, start, start + 50.0)

    now = start + 80.0
    lamps, remaining = truth_now(truth_b, "B", theta, now)
    estimate = estimate_of(coupled, "B", now)
    assert len(coupled.groups) == 1
    assert estimate.aligned
    assert math.isnan(estimate.observation_age_sec)
    assert estimate.predicted_lamps == lamps
    assert estimate.remaining_sec == pytest.approx(remaining, abs=0.3)
    assert not estimate_of(alone, "B", now).aligned


def test_unknown_frames_carry_no_information() -> None:
    tracker = single_tracker()
    for t in np.arange(0.0, 30.0, 1.0 / 30.0):
        assert tracker.observe("L1", float(t), TRAFFIC_UNKNOWN, False) == []
    group = tracker.group_of("L1")
    assert not group.observed
    assert np.all(group.log_likelihood == 0.0)
    estimate = estimate_of(tracker, "L1", 30.0)
    assert not estimate.aligned
    assert estimate.last_observed_lamps == 0
    assert math.isnan(estimate.observation_age_sec)
    assert math.isnan(estimate.cycle_time_sec)


def test_correlated_frames_count_as_one_observation_per_correlation_time() -> None:
    truth = program("p1", "L1", STRAIGHT)
    theta = 70.0
    start = 300.0 + (40.0 - theta) % 100.0
    fast = single_tracker()
    slow = single_tracker()
    feed(fast, "L1", truth, theta, start, start + 3.0 + 1.0e-6, rate_hz=30.0)
    feed(slow, "L1", truth, theta, start, start + 3.0 + 1.0e-6, rate_hz=2.0)

    # Alignments that contradict all 3 s are penalised alike at 30 Hz and at 2 Hz.
    fast_depth = fast.group_of("L1").log_likelihood.min()
    assert fast_depth == pytest.approx(slow.group_of("L1").log_likelihood.min(), abs=0.6)
    assert fast_depth > -0.8 * TrackerConfig().max_log_odds
    independent = single_tracker(config=TrackerConfig(observation_correlation_sec=1.0 / 30.0))
    feed(independent, "L1", truth, theta, start, start + 3.0 + 1.0e-6, rate_hz=30.0)
    assert independent.group_of("L1").log_likelihood.min() == -TrackerConfig().max_log_odds

    estimate = estimate_of(fast, "L1", start + 3.0)
    assert estimate.predicted_lamps == R
    assert estimate.confidence > 0.99
    assert not estimate.aligned
    # 3 s of red leaves the whole rest of the 65 s red phase possible.
    assert estimate.alignment_width_sec > 50.0
    assert estimate.remaining_max_sec - estimate.remaining_min_sec > 45.0


@pytest.mark.parametrize(
    ("config", "recovers"),
    (
        (TrackerConfig(), True),
        (TrackerConfig(max_log_odds=1.0e9, forgetting_time_constant_sec=20.0), True),
        (TrackerConfig(max_log_odds=1.0e9, forgetting_time_constant_sec=0.0), False),
    ),
)
def test_wrong_lock_recovers_through_forgetting(config: TrackerConfig, recovers: bool) -> None:
    tracker = single_tracker(config=config)
    truth = program("p1", "L1", STRAIGHT)
    wrong_theta = 10.0
    feed(tracker, "L1", truth, wrong_theta, 0.0, 300.0, rate_hz=10.0)
    assert estimate_of(tracker, "L1", 300.0).cycle_time_sec == pytest.approx((300.0 + wrong_theta) % 100.0, abs=0.3)

    true_theta = 45.0
    feed(tracker, "L1", truth, true_theta, 300.0, 360.0, rate_hz=10.0)
    estimate = estimate_of(tracker, "L1", 360.0)
    error = (estimate.cycle_time_sec - (360.0 + true_theta) % 100.0 + 50.0) % 100.0 - 50.0
    assert (estimate.aligned and abs(error) < 0.5) is recovers


def test_left_arrow_permission_timing() -> None:
    route = (RouteSignal("L1", "Z3_SIGNAL_OBEY", "F_STAGE_3", "stop_line_manual_209", "LEFT"),)
    tracker = single_tracker(FINAL_LEFT, route=route)
    truth = program("p1", "L1", FINAL_LEFT)
    theta = 3.0
    # Observe red into the round green at cycle position 0.
    start = 700.0 + (90.0 - theta)
    feed(tracker, "L1", truth, theta, start, start + 15.0)

    green = estimate_of(tracker, "L1", start + 20.0)  # cycle position 10: round green only
    assert green.on_route and green.route_maneuver == "LEFT"
    assert green.predicted_lamps == G
    assert not green.permitted_now
    assert green.permission_probability < 0.01
    assert green.permission_change_sec == pytest.approx(25.0, abs=0.3)
    assert green.remaining_sec == pytest.approx(25.0, abs=0.3)
    assert green.next_lamps == G | A

    arrow = estimate_of(tracker, "L1", start + 50.0)  # cycle position 40: green and arrow
    assert arrow.permitted_now
    assert arrow.permission_change_sec == pytest.approx(7.0, abs=0.3)

    red = estimate_of(tracker, "L1", start + 70.0)  # cycle position 60: red
    assert not red.permitted_now
    assert red.permission_change_sec == pytest.approx(75.0, abs=0.3)


def test_missed_arrows_do_not_shift_the_alignment() -> None:
    # Gazebo PV26 reads light 305's green and arrow as plain green in almost every frame.
    tracker = single_tracker(FINAL_LEFT)
    cycle = build_light_cycle(program("p1", "L1", FINAL_LEFT), "L1")
    theta = 61.4
    for t in np.arange(0.0, 400.0, 1.0 / 30.0):
        state, _ = lamps_scene_state(cycle.lamps_at((t + theta) % 100.0))
        tracker.observe("L1", float(t), state, False)
    estimate = estimate_of(tracker, "L1", 400.0)
    assert estimate.aligned
    assert estimate.cycle_time_sec == pytest.approx((400.0 + theta) % 100.0, abs=0.3)


SECOND_LEFT = ((60.0, R), (37.0, R | A), (3.0, Y))  # K-City 375, program final_second_left


def aligned_second_left(theta: float):
    """Tracker on light 375's program aligned by red, the yellow of cycle 97-100 and red again."""
    route = (RouteSignal("L1", "Z4_SIGNAL_OBEY", "F_STAGE_4", "stop_line_manual_375", "LEFT"),)
    tracker = single_tracker(SECOND_LEFT, route=route)
    truth = program("p1", "L1", SECOND_LEFT)
    start = 1000.0 + (90.0 - theta)
    feed(tracker, "L1", truth, theta, start, start + 13.0)  # cycle 90 to 3
    return tracker, start + 13.0


def test_false_arrow_on_red_does_not_move_the_alignment() -> None:
    # pv26 reads 375's red-only head as red and arrow (Gazebo 375, real 305). The alignment comes
    # from the round lamps only (owner, 2026-10-16), so it stays and the arrow is not permitted.
    theta = 23.4
    tracker, t0 = aligned_second_left(theta)
    before = estimate_of(tracker, "L1", t0)
    assert before.aligned and before.predicted_lamps == R and not before.permitted_now
    likelihood = tracker.group_of("L1").log_likelihood.copy()

    for t in np.arange(t0, t0 + 6.0, 1.0 / 30.0):
        state, arrow = lamps_scene_state(R | A)
        tracker.observe("L1", float(t), state, arrow, R | A)
    for t in np.arange(t0 + 6.0, t0 + 8.0, 1.0 / 30.0):
        state, arrow = lamps_scene_state(A)  # an arrow with no circle colour at all
        tracker.observe("L1", float(t), state, arrow, A)
    # Red read with the arrow is red evidence, and red is what the program shows.
    assert int(np.argmax(tracker.group_of("L1").log_likelihood)) == int(np.argmax(likelihood))

    now = t0 + 8.0
    estimate = estimate_of(tracker, "L1", now)
    assert estimate.aligned
    assert estimate.cycle_time_sec == pytest.approx((now + theta) % 100.0, abs=0.3)
    assert estimate.predicted_lamps == R
    assert not estimate.permitted_now
    assert estimate.permission_probability < 0.01
    assert estimate.permission_change_sec == pytest.approx(60.0 - (now + theta) % 100.0, abs=0.3)


def test_arrow_permission_comes_from_the_program_at_the_round_lamp_alignment() -> None:
    # No arrow is ever read: the arrow phase and its permission follow from the program alone.
    theta = 71.0
    tracker, t0 = aligned_second_left(theta)
    for t in np.arange(t0, t0 + 70.0, 1.0 / 30.0):
        tracker.observe("L1", float(t), *lamps_scene_state(R), R)
    edge = t0 + 57.0  # cycle position 60
    red = estimate_of(tracker, "L1", edge - 0.5)
    arrow = estimate_of(tracker, "L1", edge + 0.5)
    assert red.aligned and not red.permitted_now and red.predicted_lamps == R
    assert arrow.aligned and arrow.permitted_now and arrow.predicted_lamps == R | A
    assert arrow.permission_change_sec == pytest.approx(36.5, abs=0.3)


def test_arrow_frames_carry_no_alignment_evidence() -> None:
    tracker = single_tracker(SECOND_LEFT)
    for t in np.arange(0.0, 30.0, 1.0 / 30.0):
        tracker.observe("L1", float(t), *lamps_scene_state(A), A)
    group = tracker.group_of("L1")
    assert not group.observed
    assert np.all(group.log_likelihood == 0.0)
    assert estimate_of(tracker, "L1", 30.0).last_observed_lamps == A


def test_red_read_during_red_and_yellow_is_no_evidence_against_it() -> None:
    # K-City 375 shows red and yellow together for 3 s after its left arrow; PV26 reads red.
    shown = ((60.0, R), (37.0, R | A), (3.0, R | Y))
    yellow_only = ((60.0, R), (37.0, R | A), (3.0, Y))
    ratios = {}
    for name, phases in (("shown", shown), ("yellow_only", yellow_only)):
        tracker = single_tracker(phases)
        gateway_frames(tracker, "L1", [("RED", 2.0)], 0.0)
        group = tracker.group_of("L1")
        posterior = group.posterior()
        inside = posterior[int(round(98.0 / group.bin_sec))]  # the 2 s of red fall on 98-100
        plain = posterior[int(round(30.0 / group.bin_sec))]  # the 2 s of red fall on 30-32
        ratios[name] = inside / plain
    assert ratios["shown"] == pytest.approx(1.0, rel=1.0e-6)
    assert ratios["yellow_only"] < 0.05


def test_lamp_permission_matches_the_event_permission_rule() -> None:
    from kaiev26_msgs.msg import SceneSummary

    from kaiev26_decision.event_permission import OwnLight, light_allows

    names = {
        SceneSummary.TRAFFIC_UNKNOWN: "UNKNOWN",
        SceneSummary.TRAFFIC_RED: "RED",
        SceneSummary.TRAFFIC_YELLOW: "YELLOW",
        SceneSummary.TRAFFIC_GREEN: "GREEN",
        SceneSummary.TRAFFIC_ARROW: "ARROW",
    }
    for bits in product((0, 1), repeat=4):
        lamps = sum(bit * value for bit, value in zip(bits, (R, Y, G, A)))
        state, arrow = lamps_scene_state(lamps)
        light = OwnLight(names[state], arrow)
        for maneuver in ("NONE", "LEFT", "RIGHT"):
            assert lamps_permit(lamps, maneuver) == light_allows(light, maneuver)
        assert SceneSummary.TRAFFIC_ARROW == 4 and SceneSummary.TRAFFIC_UNKNOWN == TRAFFIC_UNKNOWN


def test_lamp_state_matches_the_gateway_interpretation() -> None:
    from kaiev26_msgs.msg import TrafficLightObservation, TrafficSignalPhase

    from kaiev26_decision.perception_gateway_node import PerceptionGatewayNode

    assert (TrafficSignalPhase.LAMP_RED, TrafficSignalPhase.LAMP_YELLOW) == (R, Y)
    assert (TrafficSignalPhase.LAMP_GREEN, TrafficSignalPhase.LAMP_ARROW) == (G, A)
    assert (TrafficLightObservation.LAMP_RED, TrafficLightObservation.LAMP_YELLOW) == (R, Y)
    assert (TrafficLightObservation.LAMP_GREEN, TrafficLightObservation.LAMP_ARROW) == (G, A)

    def pv26_state(lamps: int) -> int:
        # pv26 light_state(): yellow > red+green > red+arrow > red > green > arrow.
        if lamps & Y:
            return TrafficLightObservation.STATE_YELLOW
        if lamps & R and lamps & G:
            return TrafficLightObservation.STATE_RED
        if lamps & R and lamps & A:
            return TrafficLightObservation.STATE_ARROW
        if lamps & R:
            return TrafficLightObservation.STATE_RED
        if lamps & G:
            return TrafficLightObservation.STATE_GREEN
        if lamps & A:
            return TrafficLightObservation.STATE_ARROW
        return TrafficLightObservation.STATE_UNKNOWN

    for bits in product((0, 1), repeat=4):
        observation = TrafficLightObservation()
        lamps = sum(bit * value for bit, value in zip(bits, (R, Y, G, A)))
        observation.lamps = lamps
        observation.state = pv26_state(lamps)
        assert lamps_scene_state(lamps) == (
            PerceptionGatewayNode.map_light_observation(observation),
            PerceptionGatewayNode.map_light_arrow_active(observation),
        )


def gateway_frames(tracker, light_id, labels, start, rate_hz=20.0):
    """Frames of circle colours, e.g. [("GREEN", 34.0), ...]; returns the warnings."""
    states = {"RED": 1, "YELLOW": 2, "GREEN": 3, "UNKNOWN": 0}
    warnings = []
    t = start
    for label, duration in labels:
        for _ in range(int(round(duration * rate_hz))):
            warnings.extend(tracker.observe(light_id, t, states[label], False))
            t += 1.0 / rate_hz
    return warnings, t


def test_clean_edges_measure_phase_durations_against_the_program() -> None:
    tracker = single_tracker(((30.0, G), (3.0, Y), (67.0, R)))
    # The light really shows green 34 s, yellow 3 s, red 63 s, with one misread frame
    # and an UNKNOWN stretch inside the green.
    real = [
        ("RED", 10.0),
        ("GREEN", 12.0), ("RED", 0.05), ("GREEN", 10.0), ("UNKNOWN", 0.3), ("GREEN", 11.65),
        ("YELLOW", 3.0),
        ("RED", 63.0),
        ("GREEN", 5.0),
    ]
    warnings, _ = gateway_frames(tracker, "L1", real, 100.0)
    evidence = tracker.evidence()["lights"]["L1"]

    assert [(edge["from"], edge["to"]) for edge in evidence["edges"]] == [
        ("RED", "GREEN"), ("GREEN", "YELLOW"), ("YELLOW", "RED"), ("RED", "GREEN"),
    ]
    assert [edge["t"] for edge in evidence["edges"]] == pytest.approx(
        [110.0, 144.0, 147.0, 210.0], abs=0.06
    )
    durations = {item["label"]: item for item in evidence["durations"]}
    assert durations["GREEN"]["observed_sec"] == pytest.approx(34.0, abs=0.1)
    assert durations["GREEN"]["program_sec"] == pytest.approx(30.0)
    assert durations["YELLOW"]["error_sec"] == pytest.approx(0.0, abs=0.1)
    assert durations["RED"]["observed_sec"] == pytest.approx(63.0, abs=0.1)
    assert evidence["periods"][0]["observed_sec"] == pytest.approx(100.0, abs=0.1)
    assert len(warnings) == 2
    assert any("GREEN lasted 34.0 s" in warning for warning in warnings)
    assert any("RED lasted 63.0 s" in warning for warning in warnings)


def test_edge_after_a_long_unobserved_gap_is_not_clean() -> None:
    tracker = single_tracker(((30.0, G), (3.0, Y), (67.0, R)))
    gateway_frames(tracker, "L1", [("GREEN", 5.0)], 0.0)
    gateway_frames(tracker, "L1", [("RED", 5.0)], 40.0)
    assert tracker.evidence()["lights"] == {}


def test_loads_the_kcity_map_programs_and_route_lights() -> None:
    if not KCITY_SEMANTICS.is_file():
        pytest.skip("kaiev26_maps source tree is not next to Decision")
    signals = load_map_signals(KCITY_SEMANTICS)
    policy = load_zone_policy(PACKAGE_DIR / "config" / "final_policy.yaml")
    route = route_signals_from_policy(policy)
    tracker = SignalPhaseTracker(signals, route)

    assert len(signals.lights) == 12
    assert {item.light_id: item.maneuver for item in route} == {
        "traffic_light_manual_142": "NONE",
        "traffic_light_manual_143": "NONE",
        "traffic_light_manual_305": "LEFT",
        "traffic_light_manual_375": "LEFT",
        "traffic_light_manual_322": "NONE",
    }
    # Independent of the controller name: the four approaches of intersection #200 share one.
    (central,) = [group for group in tracker.groups.values() if "traffic_light_manual_142" in group.light_ids]
    assert sorted(central.light_ids) == [
        "traffic_light_manual_142",
        "traffic_light_manual_145",
        "traffic_light_manual_146",
        "traffic_light_manual_147",
    ]
    assert tracker.warnings_at_build == []
    final_left = tracker.cycles["traffic_light_manual_305"]
    assert [(label, duration) for label, duration, _, _ in final_left.circle_runs] == [
        ("GREEN", 47.0), ("YELLOW", 3.0), ("RED", 50.0),
    ]
    assert set(tracker.cycles) == {
        light_id for item in signals.programs for light_id in item.light_ids()
    }


# Wall-clock prior (K-City cycles start on the UTC 100 s grid, theta 0 +- 0.5 s per run).

def prior_tracker(std: float = 0.5, mean: float = 0.0, **config):
    route = (RouteSignal("L1", "Z4_SIGNAL_OBEY", "F_STAGE_4", "stop_line_manual_375", "LEFT"),)
    return single_tracker(
        SECOND_LEFT, TrackerConfig(prior_mean_sec=mean, prior_std_sec=std, **config), route
    )


def test_prior_aligns_the_schedule_before_any_observation() -> None:
    tracker = prior_tracker()
    # Cycle position 70 s: red and arrow (60-97), permitted for the left turn.
    estimate = estimate_of(tracker, "L1", 1_792_204_970.0)
    assert estimate.aligned
    assert estimate.alignment_width_sec <= 2.0
    assert estimate.predicted_lamps == R | A
    assert estimate.permitted_now
    assert estimate.permission_change_sec == pytest.approx(27.0, abs=0.2)
    assert estimate.cycle_time_sec == pytest.approx(70.0, abs=0.05)


def test_without_prior_nothing_is_aligned_before_observation() -> None:
    tracker = prior_tracker(std=0.0)
    assert not estimate_of(tracker, "L1", 1_792_204_970.0).aligned
    tracker.set_prior(0.0, 0.5)
    assert estimate_of(tracker, "L1", 1_792_204_970.0).aligned
    tracker.set_prior(0.0, 0.0)
    assert not estimate_of(tracker, "L1", 1_792_204_970.0).aligned


def test_prior_survives_forgetting_the_log_odds_floor_and_reset() -> None:
    # Strong forgetting and a low floor wipe old evidence; the prior is kept apart from it.
    tracker = prior_tracker(forgetting_time_constant_sec=5.0, max_log_odds=5.0)
    truth = program("p1", "L1", SECOND_LEFT)
    start = 2000.0 + 85.0
    feed(tracker, "L1", truth, 0.6, start, start + 16.0)  # red, yellow at 97-100 (+0.6 s), red
    # The fresh edge pulls the estimate onto the observed +0.6 s; the prior alone says 0.
    assert tracker.evidence()["groups"]["c1"]["theta_sec"] == pytest.approx(0.6, abs=0.25)
    assert tracker.evidence()["groups"]["c1"]["evidence_theta_sec"] == pytest.approx(0.6, abs=0.25)
    # A minute of plain red forgets the edge; the prior is untouched and keeps it aligned.
    feed(tracker, "L1", truth, 0.6, start + 16.0, start + 76.0)
    group = tracker.group_of("L1")
    assert group.log_prior is not None and group.log_prior.max() == 0.0
    estimate = estimate_of(tracker, "L1", start + 76.0)
    assert estimate.aligned
    assert tracker.evidence()["groups"]["c1"]["theta_sec"] == pytest.approx(0.0, abs=0.25)
    tracker.reset()
    assert estimate_of(tracker, "L1", start + 76.0).aligned


def test_prior_does_not_widen_with_time() -> None:
    tracker = prior_tracker()
    early = estimate_of(tracker, "L1", 1000.0).alignment_width_sec
    late = estimate_of(tracker, "L1", 1000.0 + 30 * 86400.0).alignment_width_sec
    assert late == pytest.approx(early)


def test_kcity_programs_carry_the_measured_305_arrow_and_375_red_and_yellow() -> None:
    if not KCITY_SEMANTICS.is_file():
        pytest.skip("kaiev26_maps source tree is not next to Decision")
    signals = load_map_signals(KCITY_SEMANTICS)
    policy = load_zone_policy(PACKAGE_DIR / "config" / "final_policy.yaml")
    tracker = SignalPhaseTracker(signals, route_signals_from_policy(policy))
    left = tracker.cycles["traffic_light_manual_305"]
    assert left.lamps_at(39.9) == G and left.lamps_at(40.1) == G | A and left.lamps_at(47.1) == Y
    second = tracker.cycles["traffic_light_manual_375"]
    assert second.lamps_at(98.0) == R | Y
    assert not lamps_permit(R | Y, "LEFT")
