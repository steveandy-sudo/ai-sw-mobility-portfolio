from pathlib import Path

import yaml

from kaiev26_export.cli import select_runs
from kaiev26_export.exporter import (
    REQUIRED_TRACKING_TOPICS,
    TRACKING_TOPICS,
    decision_runs,
    export_run,
)


def make_run(root: Path, name: str, *, finished: bool = True) -> Path:
    run = root / name
    bag = run / "rosbag"
    bag.mkdir(parents=True)
    scenario = {"case": 1, "course": "qualifying"}
    if finished:
        scenario["finished_at"] = "2026-09-12T10:00:00+09:00"
    (run / "scenario.yaml").write_text(yaml.safe_dump(scenario), encoding="utf-8")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    (bag / "rosbag_0.mcap").write_bytes(b"source")
    return run


def fake_converter(_source: Path, output: Path, topics: tuple[str, ...]) -> dict:
    output.mkdir(parents=True)
    (output / "metadata.yaml").write_text("converted: true\n", encoding="utf-8")
    (output / "tracking_0.mcap").write_bytes(b"small")
    return {topic: {"type": "test/Message", "count": 1} for topic in topics[:3]}


def test_tracking_profile_has_required_command_and_response_topics() -> None:
    assert len(TRACKING_TOPICS) == len(set(TRACKING_TOPICS))
    assert set(REQUIRED_TRACKING_TOPICS) <= set(TRACKING_TOPICS)
    for topic in (
        "/decision/raw_command",
        "/planning/command",
        "/vehicle/command",
        "/vehicle/state",
        "/steering/status",
        "/planning/route_context",
        "/localization/odometry",
    ):
        assert topic in TRACKING_TOPICS


def test_discovery_ignores_an_open_run(tmp_path: Path) -> None:
    complete = make_run(tmp_path, "20260912_100000_decision_case_01_pure_pursuit_v6")
    make_run(
        tmp_path,
        "20260912_100100_decision_case_01_stanley_v6",
        finished=False,
    )
    assert decision_runs(tmp_path) == [complete]


def test_default_and_today_selection_accept_multiple_runs(tmp_path: Path) -> None:
    first = make_run(tmp_path, "20260912_100000_decision_case_01_pure_pursuit_v6")
    second = make_run(tmp_path, "20260912_100100_decision_case_01_stanley_v6")
    assert select_runs([], tmp_path) == [first, second]
    assert select_runs(["latest"], tmp_path) == [second]


def test_export_is_atomic_and_skips_completed_output(tmp_path: Path) -> None:
    records = tmp_path / "records"
    outputs = tmp_path / "exports"
    run = make_run(records, "20260912_100000_decision_case_01_pp_stanley_v6")

    first = export_run(run, outputs, converter=fake_converter)
    assert first.status == "exported"
    assert first.detail
    assert (first.output / "manifest.json").is_file()
    assert not list(outputs.glob("*.partial-*"))

    second = export_run(run, outputs, converter=fake_converter)
    assert second.status == "skipped"
