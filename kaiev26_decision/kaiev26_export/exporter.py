"""Create a lightweight, analysis-ready MCAP from one completed Decision run."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
from typing import Callable
from zoneinfo import ZoneInfo

import yaml


TRACKING_TOPICS = (
    "/planning/command",
    "/decision/raw_command",
    "/vehicle/command",
    "/decision/scenario",
    "/drive/status",
    "/steering/status",
    "/steering/angle",
    "/brake/status",
    "/vehicle/state",
    "/vehicle/wheel_twist",
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/gnss/status",
    "/gnss/status_verbose",
    "/imu/data",
    "/imu/data_raw",
    "/ebimu/status",
    "/localization/odometry",
    "/localization/odometry_gnss",
    "/planning/route_ready",
    "/planning/readiness_status",
    "/planning/route_context",
    "/planning/scene_summary",
    "/planning/events",
    "/planning/behavior_decision",
    "/planning/target_path",
    "/planning/target_speed",
    "/debug/active_behavior",
    "/debug/fsm_state",
    "/debug/planning_latency",
    "/parameter_events",
    "/tf_static",
)

REQUIRED_TRACKING_TOPICS = (
    "/decision/raw_command",
    "/planning/command",
    "/vehicle/command",
    "/drive/status",
    "/steering/status",
    "/brake/status",
    "/vehicle/state",
    "/gnss/fix",
    "/gnss/fix_velocity",
    "/localization/odometry",
    "/planning/route_context",
    "/planning/target_speed",
)

SIDECAR_FILES = ("scenario.yaml", "result.json", "console.log")


@dataclass(frozen=True)
class ExportResult:
    run: Path
    output: Path
    status: str
    source_bytes: int = 0
    output_bytes: int = 0
    detail: str = ""


def decision_runs(records_root: Path) -> list[Path]:
    """Return completed Decision TUI runs, oldest first."""
    root = records_root.expanduser()
    if not root.is_dir():
        return []
    return sorted(
        path
        for path in root.glob("*_decision_case_*")
        if path.is_dir() and completed_run(path)
    )


def completed_run(run: Path) -> bool:
    scenario_path = run / "scenario.yaml"
    metadata_path = run / "rosbag" / "metadata.yaml"
    if not scenario_path.is_file() or not metadata_path.is_file():
        return False
    try:
        scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return False
    return bool(scenario.get("finished_at")) and bool(list((run / "rosbag").glob("*.mcap")))


def source_size(run: Path) -> int:
    return sum(path.stat().st_size for path in (run / "rosbag").glob("*.mcap"))


def exported_size(output: Path) -> int:
    return sum(path.stat().st_size for path in output.rglob("*") if path.is_file())


def complete_manifest(output: Path) -> dict | None:
    path = output / "manifest.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return document if document.get("status") == "complete" else None


def export_run(
    run: Path,
    output_root: Path,
    converter: Callable[[Path, Path, tuple[str, ...]], dict] | None = None,
) -> ExportResult:
    run = run.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    output = output_root / run.name
    source_bytes = source_size(run) if run.is_dir() else 0

    if not completed_run(run):
        return ExportResult(run, output, "incomplete", source_bytes, detail="기록 저장 미완료")
    if complete_manifest(output) is not None:
        return ExportResult(
            run,
            output,
            "skipped",
            source_bytes,
            exported_size(output),
            "이미 변환됨",
        )
    if output.exists():
        return ExportResult(
            run,
            output,
            "failed",
            source_bytes,
            detail="완료 manifest가 없는 기존 출력 디렉터리",
        )

    output_root.mkdir(parents=True, exist_ok=True)
    temporary = output_root / f".{run.name}.partial-{os.getpid()}"
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir()

    try:
        inventory = (converter or rewrite_tracking_bag)(
            run / "rosbag", temporary / "rosbag", TRACKING_TOPICS
        )
        for name in SIDECAR_FILES:
            source = run / name
            if source.is_file():
                shutil.copy2(source, temporary / name)
        snapshots = copy_configuration_snapshot(run, temporary / "snapshot")
        profile = {
            "name": "route_tracking",
            "topics": list(TRACKING_TOPICS),
        }
        (temporary / "profile.yaml").write_text(
            yaml.safe_dump(profile, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        output_bytes = exported_size(temporary)
        recorded_topics = sorted(inventory)
        missing_required = sorted(
            set(REQUIRED_TRACKING_TOPICS) - set(recorded_topics)
        )
        manifest = {
            "version": 1,
            "status": "complete",
            "exported_at": datetime.now(ZoneInfo("Asia/Seoul")).isoformat(),
            "source_run": str(run),
            "source_bag": str(run / "rosbag"),
            "source_bytes": source_bytes,
            "output_bytes": output_bytes,
            "reduction_percent": (
                round((1.0 - output_bytes / source_bytes) * 100.0, 4)
                if source_bytes
                else None
            ),
            "selected_topics": list(TRACKING_TOPICS),
            "recorded_topics": inventory,
            "missing_topics": sorted(set(TRACKING_TOPICS) - set(recorded_topics)),
            "required_tracking_topics": list(REQUIRED_TRACKING_TOPICS),
            "missing_required_topics": missing_required,
            "analysis_ready": not missing_required,
            "snapshots": snapshots,
        }
        (temporary / "manifest.json").write_text(
            json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.rename(output)
        return ExportResult(
            run,
            output,
            "exported",
            source_bytes,
            exported_size(output),
            (
                "필수 토픽 누락: " + ", ".join(missing_required)
                if missing_required
                else ""
            ),
        )
    except Exception as error:
        if temporary.exists():
            shutil.rmtree(temporary)
        return ExportResult(run, output, "failed", source_bytes, detail=str(error))


def rewrite_tracking_bag(
    source_bag: Path, output_bag: Path, topics: tuple[str, ...]
) -> dict:
    """Use rosbag2's native converter so timestamps and message schemas remain intact."""
    import rosbag2_py

    options = {
        "output_bags": [
            {
                "uri": str(output_bag),
                "storage_id": "mcap",
                "topics": list(topics),
            }
        ]
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8") as stream:
        yaml.safe_dump(options, stream, sort_keys=False)
        stream.flush()
        rosbag2_py.bag_rewrite(
            [rosbag2_py.StorageOptions(uri=str(source_bag), storage_id="mcap")],
            stream.name,
        )

    metadata = rosbag2_py.Info().read_metadata(str(output_bag), "mcap")
    return {
        item.topic_metadata.name: {
            "type": item.topic_metadata.type,
            "count": int(item.message_count),
        }
        for item in metadata.topics_with_message_count
    }


def copy_configuration_snapshot(run: Path, destination: Path) -> dict:
    destination.mkdir(parents=True, exist_ok=True)
    scenario = yaml.safe_load((run / "scenario.yaml").read_text(encoding="utf-8")) or {}
    case = int(scenario.get("case", 0) or 0)
    course = scenario.get("course") or ("qualifying" if 1 <= case <= 3 else "final")
    route_name = "kcity_quali_route.yaml" if course == "qualifying" else "kcity_final_route.yaml"

    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory

        decision_share = Path(get_package_share_directory("kaiev26_decision"))
        motion_share = Path(get_package_share_directory("kaiev26_motion_control"))
        candidates.extend(
            [
                (decision_share / "config/decision_pipeline.yaml", "decision_pipeline.yaml"),
                (decision_share / f"config/{course}_policy.yaml", "course_policy.yaml"),
                (decision_share / f"config/{course}_landmarks.yaml", "course_landmarks.yaml"),
                (decision_share / f"waypoints/{route_name}", "route.yaml"),
                (motion_share / "config/motion_control.yaml", "motion_control.yaml"),
            ]
        )
    except (ImportError, LookupError):
        pass

    copied = {}
    for source, name in candidates:
        if not source.is_file():
            continue
        target = destination / name
        shutil.copy2(source, target)
        copied[name] = {
            "source": str(source),
            "sha256": file_sha256(target),
        }
    if not copied:
        destination.rmdir()
    return copied


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
