from pathlib import Path

from kaiev26_msgs.msg import RouteContext, SceneSummary, VehicleState
import pytest

from kaiev26_decision.priority_selector import PrioritySelector
from kaiev26_decision.scenario_modules.static_stop_fsm import StaticStopFSM
from kaiev26_decision.scenario_modules.traffic_light_fsm import TrafficLightFSM
from kaiev26_decision.zone_policy import (
    StagePolicy,
    ZonePolicy,
    ZonePolicyRuntime,
    load_zone_policy,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def route_context(
    zone: str,
    stop_line_id: str,
    distance_m: float,
    traffic_light_ids: list[str] | None = None,
) -> RouteContext:
    route = RouteContext()
    route.route_projection_valid = True
    route.current_zone = zone
    route.next_stop_line_id = stop_line_id
    route.distance_to_stopline = distance_m
    route.next_traffic_light_ids = traffic_light_ids or []
    return route


def test_qualifying_policy_builds_five_ordered_stage_ranges() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")

    assert [
        stage.event.brake_trigger_waypoint for stage in policy.stages[:4]
    ] == [203, 244, 296, 337]

    ranges = policy.zone_ranges(
        {
            "stop_line_manual_264": 203.0,
            "stop_line_manual_058": 243.0,
            "stop_line_manual_341": 294.0,
            "stop_line_manual_048": 351.0,
        },
        518.0,
    )

    assert [item["zone"] for item in ranges] == [
        "Q_STAGE_1",
        "Q_STAGE_2",
        "Q_STAGE_3",
        "Q_STAGE_4",
        "Q_STAGE_5",
    ]
    assert [(item["start_s"], item["end_s"]) for item in ranges] == [
        (0.0, 203.0),
        (203.0, 243.0),
        (243.0, 294.0),
        (294.0, 351.0),
        (351.0, 518.0),
    ]


def test_final_policy_defines_five_signals_then_obstacle_and_finish() -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")

    assert [stage.stage_id for stage in policy.stages] == [
        "F_STAGE_1",
        "F_STAGE_2",
        "F_STAGE_3",
        "F_STAGE_4",
        "F_STAGE_5",
        "F_STAGE_6",
    ]
    assert [stage.event.stop_line_id for stage in policy.stages[:5]] == [
        "stop_line_manual_052",
        "stop_line_manual_057",
        "stop_line_manual_209",
        "stop_line_manual_214",
        "stop_line_manual_221",
    ]
    assert policy.stages[-1].obstacle_enabled
    assert policy.stages[-1].event.event_type == "FINISH"
    assert [stage.event.arrow_maneuver for stage in policy.stages[:5]] == [
        "NONE",
        "NONE",
        "LEFT",
        "LEFT",
        "NONE",
    ]
    assert [
        stage.event.brake_trigger_waypoint for stage in policy.stages[:5]
    ] == [98, 153, 289, 346, 448]


def test_stage_runtime_never_regresses_after_advancing() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    runtime = ZonePolicyRuntime(policy)

    assert runtime.stage_for("Q_STAGE_2").stage_id == "Q_STAGE_2"
    assert runtime.stage_for("Q_STAGE_1").stage_id == "Q_STAGE_2"
    assert runtime.stage_for("Q_STAGE_3").stage_id == "Q_STAGE_3"


def test_route_progress_can_end_a_stage_without_a_stopline() -> None:
    policy = ZonePolicy(
        course_id="obstacle_course",
        ready=True,
        stages=(
            StagePolicy("F_STAGE_5", obstacle_enabled=True, boundary_route_s=80.0),
            StagePolicy("F_STAGE_6"),
        ),
    )

    assert policy.zone_ranges({}, 100.0) == [
        {"zone": "F_STAGE_5", "start_s": 0.0, "end_s": 80.0},
        {"zone": "F_STAGE_6", "start_s": 80.0, "end_s": 100.0},
    ]


def test_signal_policy_ignores_other_lights_and_requires_exact_identity() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED

    ignored = route_context(
        "Q_STAGE_1",
        "stop_line_manual_048",
        8.0,
        ["traffic_light_manual_144"],
    )
    assert selector.choose(scene, ignored, None, False).behavior == "LANE_FOLLOW"

    expected = route_context(
        "Q_STAGE_2",
        "stop_line_manual_058",
        8.0,
        ["traffic_light_manual_318"],
    )
    choice = selector.choose(scene, expected, None, False)
    assert choice.behavior == "TRAFFIC_LIGHT"
    assert choice.event_id == "Z2_SIGNAL_OBEY"
    assert choice.brake_trigger_waypoint == 244
    assert choice.brake_trigger_route_s == pytest.approx(243.954400)

    wrong_identity = route_context(
        "Q_STAGE_2",
        "stop_line_manual_058",
        8.0,
        ["traffic_light_manual_wrong"],
    )
    assert selector.choose(scene, wrong_identity, None, False).behavior == "RECOVERY"


def test_qualifying_policy_does_not_enable_obstacle_mission() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.obstacle_on_path = True
    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    route = route_context("Q_STAGE_5", "", 1.0e6)

    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


def test_qualifying_stage_three_stops_without_using_a_signal() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    route = route_context("Q_STAGE_3", "stop_line_manual_341", 8.0)

    choice = selector.choose(scene, route, None, False)

    assert choice.behavior == "STATIC_STOP"
    assert choice.event_id == "Z3_TIMED_STOP"
    assert choice.stop_line_id == "stop_line_manual_341"
    assert choice.hold_sec == pytest.approx(3.0)
    assert choice.brake_trigger_waypoint == 296
    assert choice.brake_trigger_route_s == pytest.approx(295.951674)


def test_stopline_test_mode_turns_signal_event_into_three_second_stop() -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")
    selector = PrioritySelector(
        18.0,
        zone_policy=policy,
        mission_mode="stopline",
        test_stop_hold_sec=3.0,
    )
    scene = SceneSummary()
    scene.perception_health = "LOST"
    route = route_context(
        "F_STAGE_3",
        "stop_line_manual_209",
        8.0,
        ["traffic_light_manual_305"],
    )

    choice = selector.choose(scene, route, None, False)

    assert choice.behavior == "STATIC_STOP"
    assert choice.event_id == "Z3_SIGNAL_OBEY"
    assert choice.hold_sec == pytest.approx(3.0)


def test_avoidance_test_mode_ignores_signal_events() -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")
    selector = PrioritySelector(
        18.0,
        zone_policy=policy,
        mission_mode="avoidance",
    )
    scene = SceneSummary()
    scene.perception_health = "GOOD"
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    route = route_context(
        "F_STAGE_3",
        "stop_line_manual_209",
        8.0,
        ["traffic_light_manual_305"],
    )

    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


def test_finish_takes_priority_over_final_stage_obstacle_state() -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.obstacle_on_path = True
    route = route_context("FINISH_ZONE", "", 1.0e6)

    assert selector.choose(scene, route, None, False).behavior == "FINISH"


def test_green_signal_is_committed_through_the_camera_blind_zone() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    route = route_context(
        "Q_STAGE_2",
        "stop_line_manual_058",
        5.0,
        ["traffic_light_manual_318"],
    )
    route.next_maneuver = "STRAIGHT"
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN

    committed = selector.choose(scene, route, None, False)
    assert committed.behavior == "LANE_FOLLOW"
    assert "committed" in committed.reason

    scene.traffic_light_state = SceneSummary.TRAFFIC_UNKNOWN
    blind_zone = selector.choose(scene, route, None, False)
    assert blind_zone.behavior == "LANE_FOLLOW"
    assert "already committed" in blind_zone.reason


@pytest.mark.parametrize(
    ("zone", "stop_line", "traffic_light"),
    [
        ("Q_STAGE_2", "stop_line_manual_058", "traffic_light_manual_318"),
        ("Q_STAGE_4", "stop_line_manual_048", "traffic_light_manual_144"),
    ],
)
@pytest.mark.parametrize(
    "signal_state",
    [
        SceneSummary.TRAFFIC_RED,
        SceneSummary.TRAFFIC_YELLOW,
        SceneSummary.TRAFFIC_UNKNOWN,
        SceneSummary.TRAFFIC_ARROW,
    ],
)
def test_qualifying_signals_use_binary_stop_or_go(
    zone: str,
    stop_line: str,
    traffic_light: str,
    signal_state: int,
) -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    route = route_context(
        zone,
        stop_line,
        8.0,
        [traffic_light],
    )
    route.next_maneuver = "STRAIGHT"
    scene = SceneSummary()
    scene.traffic_light_state = signal_state

    assert selector.choose(scene, route, None, False).behavior == "TRAFFIC_LIGHT"

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


@pytest.mark.parametrize(
    ("zone", "stop_line", "traffic_light", "maneuver", "expected_behavior"),
    [
        (
            "F_STAGE_1",
            "stop_line_manual_052",
            "traffic_light_manual_142",
            "STRAIGHT",
            "TRAFFIC_LIGHT",
        ),
        (
            "F_STAGE_3",
            "stop_line_manual_209",
            "traffic_light_manual_305",
            "LEFT",
            "LANE_FOLLOW",
        ),
        (
            "F_STAGE_3",
            "stop_line_manual_209",
            "traffic_light_manual_305",
            "RIGHT",
            "TRAFFIC_LIGHT",
        ),
        (
            "F_STAGE_4",
            "stop_line_manual_214",
            "traffic_light_manual_375",
            "LEFT",
            "LANE_FOLLOW",
        ),
        (
            "F_STAGE_4",
            "stop_line_manual_214",
            "traffic_light_manual_375",
            "RIGHT",
            "TRAFFIC_LIGHT",
        ),
        (
            "F_STAGE_5",
            "stop_line_manual_221",
            "traffic_light_manual_322",
            "STRAIGHT",
            "TRAFFIC_LIGHT",
        ),
    ],
)
def test_final_arrow_permission_is_defined_by_zone_policy(
    zone: str,
    stop_line: str,
    traffic_light: str,
    maneuver: str,
    expected_behavior: str,
) -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_ARROW
    route = route_context(zone, stop_line, 8.0, [traffic_light])
    route.next_maneuver = maneuver

    choice = selector.choose(scene, route, None, False)

    assert choice.behavior == expected_behavior


def test_final_stage_six_ignores_removed_central_signal_event() -> None:
    policy = load_zone_policy(CONFIG_DIR / "final_policy.yaml")
    selector = PrioritySelector(18.0, zone_policy=policy)
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    route = route_context(
        "F_STAGE_6",
        "stop_line_manual_053",
        8.0,
        ["traffic_light_manual_147"],
    )
    route.next_maneuver = "RIGHT"

    assert selector.choose(scene, route, None, False).behavior == "LANE_FOLLOW"


def test_timed_stop_counts_after_measured_stop_then_releases() -> None:
    fsm = StaticStopFSM(
        base_speed_mps=3.0,
        approach_distance_m=18.0,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=1.0,
        stopped_speed_mps=0.1,
    )
    scene = SceneSummary()
    route = route_context("Q_STAGE_1", "stop_line_manual_264", 3.0)
    vehicle = VehicleState()
    vehicle.speed_mps = 0.2

    moving = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        now_s=10.0,
    )
    assert moving.fsm_state == "STOP_CONFIRM"

    vehicle.speed_mps = 0.0
    holding = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        now_s=10.0,
    )
    assert holding.fsm_state == "HOLD_3S"

    releasing = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        now_s=13.0,
    )
    assert releasing.fsm_state == "RELEASE"
    assert not releasing.need_stop

    route.next_stop_line_id = "stop_line_manual_058"
    complete = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        now_s=13.1,
    )
    assert complete.fsm_state == "COMPLETE"
    assert "TIMED_STOP_COMPLETE" in complete.constraints


def test_timed_stop_pre_decelerates_then_hard_brakes_at_configured_waypoint() -> None:
    fsm = StaticStopFSM(
        base_speed_mps=3.2,
        approach_distance_m=30.0,
        vehicle_front_overhang_m=1.98,
        stopline_clearance_m=1.5,
        pre_brake_decel_distance_m=6.0,
        brake_entry_speed_mps=2.0,
    )
    scene = SceneSummary()
    route = route_context("Q_STAGE_1", "stop_line_manual_264", 8.0)
    vehicle = VehicleState()
    vehicle.speed_mps = 2.5
    trigger_s = 202.966612

    route.progress_s = trigger_s - 3.0
    approach = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        brake_trigger_route_s=trigger_s,
    )
    assert approach.fsm_state == "PRE_DECEL"
    assert approach.need_stop is False
    assert approach.target_speed_limit_mps == pytest.approx(2.6)

    route.progress_s = trigger_s + 0.05
    braking = fsm.update(
        scene,
        route,
        vehicle,
        event_id="Z1_TIMED_STOP",
        stop_line_id="stop_line_manual_264",
        hold_sec=3.0,
        brake_trigger_route_s=trigger_s,
    )
    assert braking.fsm_state == "STOP_CONFIRM"
    assert braking.need_stop is True
    assert braking.stop_target_distance == 0.0


def test_signal_fsm_uses_expected_stopline_to_confirm_pass() -> None:
    fsm = TrafficLightFSM(3.0, 18.0, release_confirm_sec=0.0)
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 3.0
    route = route_context("Q_STAGE_2", "stop_line_manual_058", 3.0)

    stopped = fsm.update(
        scene,
        route,
        VehicleState(),
        expected_stop_line_id="stop_line_manual_058",
    )
    assert stopped.need_stop

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    passing = fsm.update(
        scene,
        route,
        VehicleState(),
        expected_stop_line_id="stop_line_manual_058",
    )
    assert passing.fsm_state == "PASSING"

    route.next_stop_line_id = "stop_line_manual_341"
    complete = fsm.update(
        scene,
        route,
        VehicleState(),
        expected_stop_line_id="stop_line_manual_058",
    )
    assert "SIGNAL_PASS_COMPLETE" in complete.constraints


def test_signal_hard_brake_cannot_release_before_measured_stop() -> None:
    fsm = TrafficLightFSM(
        4.0,
        30.0,
        release_confirm_sec=0.0,
        pre_brake_decel_distance_m=6.0,
        brake_entry_speed_mps=2.0,
    )
    scene = SceneSummary()
    scene.traffic_light_state = SceneSummary.TRAFFIC_RED
    scene.stopline_distance = 5.0
    route = route_context(
        "F_STAGE_1",
        "stop_line_manual_052",
        5.0,
        ["traffic_light_manual_142"],
    )
    vehicle = VehicleState()
    trigger_s = 97.984659

    route.progress_s = trigger_s - 3.0
    approach = fsm.update(
        scene,
        route,
        vehicle,
        expected_stop_line_id="stop_line_manual_052",
        brake_trigger_route_s=trigger_s,
    )
    assert approach.fsm_state == "PRE_DECEL"
    assert approach.target_speed_limit_mps == pytest.approx(3.0)

    route.progress_s = trigger_s
    vehicle.speed_mps = 2.0
    braking = fsm.update(
        scene,
        route,
        vehicle,
        expected_stop_line_id="stop_line_manual_052",
        brake_trigger_route_s=trigger_s,
    )
    assert braking.fsm_state == "STOP_CONFIRM"
    assert braking.stop_target_distance == 0.0

    scene.traffic_light_state = SceneSummary.TRAFFIC_GREEN
    still_moving = fsm.update(
        scene,
        route,
        vehicle,
        expected_stop_line_id="stop_line_manual_052",
        brake_trigger_route_s=trigger_s,
    )
    assert still_moving.fsm_state == "STOP_CONFIRM"
    assert still_moving.need_stop is True

    vehicle.speed_mps = 0.0
    released = fsm.update(
        scene,
        route,
        vehicle,
        expected_stop_line_id="stop_line_manual_052",
        brake_trigger_route_s=trigger_s,
    )
    assert released.fsm_state == "PASSING"
    assert released.need_stop is False
