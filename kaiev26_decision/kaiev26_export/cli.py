"""Command-line entry point for batch Decision tracking exports."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

from .exporter import ExportResult, decision_runs, export_run


DEFAULT_RECORDS_ROOT = Path.home() / "KAI_ws/records"
DEFAULT_OUTPUT_ROOT = Path.home() / "KAI_ws/exports/kaiev26_tracking"


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(
        description="완료된 Decision MCAP을 경로 추종 분석용 slim MCAP으로 변환합니다."
    )
    value.add_argument(
        "selection",
        nargs="*",
        help="생략, latest, today, all 또는 하나 이상의 Decision Run 디렉터리",
    )
    value.add_argument("--records-root", type=Path, default=DEFAULT_RECORDS_ROOT)
    value.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    return value


def select_runs(selection: list[str], records_root: Path) -> list[Path]:
    discovered = decision_runs(records_root)
    if not selection or selection == ["all"]:
        return discovered
    if selection == ["latest"]:
        return discovered[-1:] if discovered else []
    if selection == ["today"]:
        prefix = datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_")
        return [run for run in discovered if run.name.startswith(prefix)]
    if any(value in {"all", "latest", "today"} for value in selection):
        raise ValueError("latest, today, all은 다른 경로와 함께 사용할 수 없습니다")
    return [Path(value).expanduser() for value in selection]


def human_size(value: int) -> str:
    size = float(value)
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} TiB"


def print_result(index: int, total: int, result: ExportResult) -> None:
    prefix = f"[{index}/{total}] {result.run.name}"
    if result.status == "exported":
        reduction = (
            (1.0 - result.output_bytes / result.source_bytes) * 100.0
            if result.source_bytes
            else 0.0
        )
        print(
            f"{prefix}\n  {human_size(result.source_bytes)} -> "
            f"{human_size(result.output_bytes)} ({reduction:.2f}% 감소)  OK"
        )
        if result.detail:
            print(f"  WARN: {result.detail}")
    elif result.status == "skipped":
        print(f"{prefix}\n  SKIP: {result.detail}")
    else:
        print(f"{prefix}\n  {result.status.upper()}: {result.detail}")


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        runs = select_runs(args.selection, args.records_root)
    except ValueError as error:
        print(f"입력 오류: {error}", file=sys.stderr)
        return 2

    print("Decision route-tracking export")
    if not runs:
        print("변환할 완료된 Decision 기록이 없습니다.")
        return 0

    results = []
    for index, run in enumerate(runs, 1):
        result = export_run(run, args.output_root)
        results.append(result)
        print_result(index, len(runs), result)

    counts = {
        status: sum(result.status == status for result in results)
        for status in ("exported", "skipped", "incomplete", "failed")
    }
    print(
        "\n완료 "
        f"{counts['exported']} / 건너뜀 {counts['skipped']} / "
        f"미완료 {counts['incomplete']} / 실패 {counts['failed']}"
    )
    print(f"저장: {args.output_root.expanduser()}")
    return 1 if counts["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
