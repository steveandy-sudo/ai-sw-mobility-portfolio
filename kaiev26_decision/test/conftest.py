"""Scenario catalog switch: the `scenario` tests run only when asked for.

    python3 -m pytest -m scenario kaiev26_decision/test/test_scenarios.py
    python3 -m pytest -m scenario kaiev26_decision/test/test_scenarios.py -k final_red --scenario-report DIR

Without `-m scenario` they are collected and skipped, so the unit run stays fast.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest


def pytest_addoption(parser):
    group = parser.getgroup("decision scenarios")
    group.addoption("--scenario-workers", type=int, default=None,
                    help="scenarios run at once (default: CPUs - 2, at most 12; env DECISION_SCENARIO_WORKERS)")
    group.addoption("--scenario-report", default=None,
                    help="directory for one result file per scenario and a summary table")


def pytest_configure(config):
    config.addinivalue_line("markers", "scenario: topic-level Decision scenario from test/scenarios (run with -m scenario)")


def pytest_collection_modifyitems(config, items):
    if "scenario" in (config.getoption("markexpr") or ""):
        return
    skip = pytest.mark.skip(reason="scenario catalog runs only with -m scenario")
    for item in items:
        if "scenario" in item.keywords:
            item.add_marker(skip)


@pytest.fixture(scope="session")
def scenario_results(request):
    """Run every selected scenario once, several at a time, before the first one is reported."""
    from scenario_harness.catalog import run_many

    selected = [
        item.callspec.params["scenario_case"] for item in request.session.items
        if "scenario" in item.keywords and hasattr(item, "callspec") and "scenario_case" in item.callspec.params
    ]
    results = run_many([(case.id, case.scenario) for case in selected], request.config.getoption("--scenario-workers"))
    report = request.config.getoption("--scenario-report")
    if report:
        _write_report(Path(report), selected, results)
    return results


def _write_report(directory: Path, selected, results) -> None:
    from scenario_harness.catalog import format_result

    directory.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in selected:
        result = results[case.id]
        safe = case.id.replace("::", "__").replace("/", "_")
        (directory / f"{safe}.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
        (directory / f"{safe}.txt").write_text(format_result(result) + "\n", encoding="utf-8")
        failed = [check["check"] for check in result.get("checks", []) if not check["passed"]]
        verdict = "ERROR" if result.get("error") else ("FAIL" if failed else "PASS")
        rows.append(f"{verdict}\t{case.id}\t{result.get('wall_s', '')}\t{', '.join(failed)}")
    (directory / "summary.tsv").write_text("verdict\tscenario\twall_s\tfailed checks\n" + "\n".join(rows) + "\n", encoding="utf-8")
