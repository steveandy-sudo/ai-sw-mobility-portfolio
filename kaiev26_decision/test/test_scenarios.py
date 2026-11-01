"""Every scenario in test/scenarios/*.yaml, judged by the fixed check vocabulary.

A scenario file describes a situation (course, start, light script, dropouts, drums, plant) and the
expected result as checks from scenario_harness/checks.py. Adding a scenario is adding a yaml file.
"""
from __future__ import annotations

from dataclasses import dataclass

import pytest

from scenario_harness.catalog import discover, format_result


@dataclass(frozen=True)
class ScenarioCase:
    id: str
    scenario: dict


CASES = [ScenarioCase(scenario_id, scenario) for scenario_id, scenario in discover()]


@pytest.mark.scenario
@pytest.mark.parametrize("scenario_case", CASES, ids=[case.id for case in CASES])
def test_scenario(scenario_case, scenario_results):
    result = scenario_results[scenario_case.id]
    if result.get("error") or not all(check["passed"] for check in result.get("checks", [])):
        pytest.fail(f"{scenario_case.id}\n{format_result(result)}", pytrace=False)
    assert result.get("checks"), f"{scenario_case.id} has no expectations"
