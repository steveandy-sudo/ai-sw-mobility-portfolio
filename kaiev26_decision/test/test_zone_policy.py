from pathlib import Path
import re

import pytest

from kaiev26_decision.common import StoppingModel
from kaiev26_decision.zone_policy import (
    StagePolicy,
    ZonePolicy,
    ZonePolicyRuntime,
    load_zone_policy,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
STOPPING = StoppingModel()


def test_qualifying_policy_builds_five_ordered_stage_ranges() -> None:
    policy = load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml")

    assert [
        stage.event.brake_trigger_waypoint for stage in policy.stages[:4]
    ] == [204, 249, 296, 340]

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
    ] == [120, 165, 302, 356, 454]


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


def test_stopping_distance_uses_brake_delay_and_deceleration() -> None:
    assert STOPPING.distance_m(0.0) == 0.0
    assert STOPPING.distance_m(7.0) == pytest.approx(7.0 * 0.70 + 49.0 / 7.8)
    assert STOPPING.distance_m(-8.0) == pytest.approx(STOPPING.distance_m(8.0))
    assert StoppingModel(0.63, 7.65).distance_m(8.0) == pytest.approx(8.0 * 0.63 + 64.0 / 15.3)


def test_policies_set_a_brake_margin_for_every_stop_event() -> None:
    margins = {}
    for course in ("qualifying", "final"):
        policy = load_zone_policy(CONFIG_DIR / f"{course}_policy.yaml")
        for stage in policy.stages:
            if stage.event is not None and stage.event.event_type in {"TIMED_STOP", "SIGNAL_OBEY"}:
                margins[stage.stage_id] = stage.event.brake_margin_m
    assert margins == {
        "Q_STAGE_1": 2.75, "Q_STAGE_2": 2.8, "Q_STAGE_3": 2.85, "Q_STAGE_4": 3.2,
        "F_STAGE_1": 2.15, "F_STAGE_2": 1.75, "F_STAGE_3": 3.25, "F_STAGE_4": 2.35, "F_STAGE_5": 7.5,
    }


@pytest.mark.parametrize("course, event_id", [("qualifying", "Z4_SIGNAL_OBEY"), ("qualifying", "Z5_FINISH")])
def test_policy_keeps_the_finish_line_on_finish_events(tmp_path, course, event_id) -> None:
    text = (CONFIG_DIR / f"{course}_policy.yaml").read_text()
    if event_id == "Z5_FINISH":
        text = text.replace("finish_line_route_s: 555.669720", "finish_line_route_s: -1.0")
        match = "finish_line_route_s must be positive"
    else:
        text = text.replace(f"id: {event_id}\n", f"id: {event_id}\n      finish_line_route_s: 100.0\n")
        match = "for FINISH events only"
    path = tmp_path / "policy.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match=match):
        load_zone_policy(path)


@pytest.mark.parametrize("margin", [-0.1, 10.5])
def test_policy_rejects_brake_margin_out_of_range(tmp_path, margin) -> None:
    text = (CONFIG_DIR / "qualifying_policy.yaml").read_text()
    text = re.sub(r"brake_margin_m: \S+", f"brake_margin_m: {margin}", text, count=1)
    path = tmp_path / "policy.yaml"
    path.write_text(text)
    with pytest.raises(ValueError, match="brake_margin_m"):
        load_zone_policy(path)


def test_final_policy_owns_the_obstacle_zone_polygon_and_map_lane_lines() -> None:
    zone = load_zone_policy(CONFIG_DIR / "final_policy.yaml").obstacle_zone

    assert zone is not None
    assert len(zone.polygon) >= 3
    assert zone.map_semantics.endswith("kcity_course_semantics.yaml")
    assert zone.right_boundary_ids and zone.left_boundary_ids
    assert load_zone_policy(CONFIG_DIR / "qualifying_policy.yaml").obstacle_zone is None


@pytest.mark.parametrize("change", [
    ("polygon:", "polygon_missing:"),
    ("    right:", "    right_missing:"),
])
def test_obstacle_zone_without_polygon_or_lane_lines_is_rejected(tmp_path, change) -> None:
    text = (CONFIG_DIR / "final_policy.yaml").read_text()
    head, zone = text.split("obstacle_zone:", 1)
    path = tmp_path / "policy.yaml"
    path.write_text(head + "obstacle_zone:" + zone.replace(*change, 1))
    with pytest.raises(ValueError):
        load_zone_policy(path)
