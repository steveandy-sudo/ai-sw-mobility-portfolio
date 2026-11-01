"""Scenario catalog: find the scenario files, run each scenario in its own isolated process.

Every scenario runs in a fresh Python process (`python3 -m scenario_harness.worker`) because the
Decision nodes live in the default rclpy context. The process is a Fast DDS discovery-server client
of a server nobody runs, so its nodes talk to each other in-process but are invisible to, and
cannot hear, anything else on the machine; that is what lets several scenarios run at once.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys

from .runner import expand_file

TEST_DIR = Path(__file__).resolve().parents[1]
SCENARIO_DIR = TEST_DIR / "scenarios"
RESULT_PREFIX = "SCENARIO_RESULT "
# No discovery server listens here; a client of it discovers nothing outside its own process.
ISOLATED_DISCOVERY_SERVER = "127.0.0.1:47011"


def scenario_files(directory: Path = SCENARIO_DIR) -> list[Path]:
    return sorted(path for path in directory.glob("*.yaml") if not path.name.startswith("_"))


def discover(directory: Path = SCENARIO_DIR) -> list[tuple[str, dict]]:
    found = []
    seen = set()
    for path in scenario_files(directory):
        for scenario in expand_file(path):
            name = str(scenario.get("name") or path.stem)
            scenario_id = f"{path.stem}::{name}" if name != path.stem else path.stem
            if scenario_id in seen:
                raise ValueError(f"duplicate scenario id {scenario_id}; give each matrix entry a distinct name")
            seen.add(scenario_id)
            found.append((scenario_id, scenario))
    return found


def isolated_env() -> dict:
    env = dict(os.environ)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOCALHOST_ONLY"] = "0"
    env["ROS_DISCOVERY_SERVER"] = ISOLATED_DISCOVERY_SERVER
    env.setdefault("ROS_DOMAIN_ID", "42")
    env["PYTHONPATH"] = str(TEST_DIR) + os.pathsep + env.get("PYTHONPATH", "")
    env["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = env.get("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "WARN")
    return env


def run_isolated(scenario_id: str, scenario: dict, timeout_s: float = 300.0) -> dict:
    payload = json.dumps({"id": scenario_id, "scenario": scenario})
    try:
        completed = subprocess.run(
            [sys.executable, "-m", "scenario_harness.worker"],
            input=payload, capture_output=True, text=True, env=isolated_env(), cwd=str(TEST_DIR),
            timeout=timeout_s, check=False,
        )
    except subprocess.TimeoutExpired:
        return {"id": scenario_id, "error": f"timed out after {timeout_s} s", "checks": []}
    for line in reversed(completed.stdout.splitlines()):
        if line.startswith(RESULT_PREFIX):
            return json.loads(line[len(RESULT_PREFIX):])
    tail = "\n".join((completed.stderr or completed.stdout).splitlines()[-30:])
    return {"id": scenario_id, "error": f"worker exited {completed.returncode} without a result:\n{tail}", "checks": []}


def default_workers() -> int:
    value = os.environ.get("DECISION_SCENARIO_WORKERS")
    if value:
        return max(1, int(value))
    return max(1, min(12, (os.cpu_count() or 2) - 2))


def run_many(items: list[tuple[str, dict]], workers: int | None = None) -> dict[str, dict]:
    """Longest scenarios start first so the pool does not wait on one long run at the end."""
    workers = workers or default_workers()
    ordered = sorted(items, key=lambda item: -float(item[1].get("duration_s", 0.0)))
    with ThreadPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(lambda item: run_isolated(*item), ordered))
    return {result["id"]: result for result in results}


def format_result(result: dict) -> str:
    lines = []
    if result.get("error"):
        lines.append(f"ERROR {result['error']}")
    for check in result.get("checks", []):
        lines.append(f"{'PASS' if check['passed'] else 'FAIL'} {check['check']}: {check['detail']}")
    if result.get("trace"):
        lines.append("trace (t, v, front gap to the scenario event, cmd speed, brake cmd/applied, behavior:fsm):")
        lines.extend("  " + row for row in result["trace"])
    return "\n".join(lines)
