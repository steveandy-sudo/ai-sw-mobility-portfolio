import runpy
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest

from scenario_harness.launch_nodes import node_specs


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "decision.launch.py"


def launch_parameters(**arguments):
    return {spec.executable: spec.parameters for spec in node_specs(arguments)}


@pytest.mark.parametrize('options,expected', [([], False), (['--sim-time'], True)])
def test_test_tui_entry_preserves_sim_time_selection(monkeypatch, options, expected) -> None:
    from kaiev26_decision import test_tui

    calls = []
    monkeypatch.setattr(test_tui, 'run_tui', lambda **kwargs: calls.append(kwargs) or 0)
    assert test_tui.main(options) == 0
    assert calls == [{
        'command_hint': 'ros2 run kaiev26_decision decision_test', 'sim_time': expected,
    }]


def test_vehicle_decision_launch_opens_one_operator_without_inline_arguments() -> None:
    from launch.actions import ExecuteProcess, Shutdown
    module = runpy.run_path(str(LAUNCH_FILE))
    actions = module["generate_launch_description"]().entities
    assert len(actions) == 1
    process = actions[0]
    assert isinstance(process, ExecuteProcess)
    assert not any(isinstance(action, DeclareLaunchArgument) for action in actions)
    assert [part[0].perform(LaunchContext()) for part in process.cmd][1:] == [
        '-m', 'kaiev26_decision.operating_tui']
    assert any(isinstance(action, Shutdown) for action in process.get_sub_entities())


@pytest.mark.parametrize("case", (1, 4, 13, 17))
def test_decision_test_rejects_unsupported_simulation_cases(case: int) -> None:
    with pytest.raises(RuntimeError, match="simulation platform supports only cases 3, 6, and 7"):
        launch_parameters(platform="simulation", case=case)


@pytest.mark.parametrize("case", (3, 6, 7))
def test_supported_simulation_cases_include_perception_gateway(case: int) -> None:
    assert "perception_gateway_node" in launch_parameters(platform="simulation", case=case)


@pytest.mark.parametrize('case,course,mode', [
    (1, 'qualifying', 'all'), (1, 'final', 'all'),
    (2, 'qualifying', 'tracking'), (2, 'final', 'tracking'),
])
def test_tui_cases_launch_the_expected_pipeline(case, course, mode):
    from kaiev26_decision.test_tui import TestSettings
    from kaiev26_decision.zone_policy import load_zone_policy
    settings = TestSettings(case=case, course=course, speed_mps=6.)
    configurations = dict(argument.split(':=', 1) for argument in settings.launch_arguments())
    parameters = launch_parameters(**configurations)
    planner = parameters['main_planning_engine_node']
    assert planner['mission_mode'] == mode
    assert planner['base_speed_mps'] == 6.
    assert parameters['motion_control_node']['command_topic'] == '/decision/raw_command'
    assert ('perception_gateway_node' in parameters) == (mode != 'tracking')
    if mode == 'all':
        policy = load_zone_policy(planner['mission_policy_config'])
        events = [stage.event.event_type for stage in policy.stages if stage.event]
        assert events.count('SIGNAL_OBEY') == (2 if course == 'qualifying' else 5)
        assert events.count('TIMED_STOP') == (2 if course == 'qualifying' else 0)
        assert events.count('FINISH') == 1


# situation: start mode -> route manager start_mode, route_readiness allows a mid-route start.
@pytest.mark.parametrize(('start_mode', 'midroute'), [('from_start', False), ('resume', True)])
def test_start_mode_reaches_the_route_manager_and_readiness(start_mode, midroute):
    parameters = launch_parameters(case='7', start_mode=start_mode)

    assert parameters['route_zone_manager_node']['start_mode'] == start_mode
    assert parameters['route_readiness_node']['allow_midroute_start'] is midroute


def test_start_mode_defaults_to_resume_and_rejects_unknown_values():
    assert launch_parameters(case='7')['route_zone_manager_node']['start_mode'] == 'resume'
    with pytest.raises(RuntimeError):
        launch_parameters(case='7', start_mode='middle')


@pytest.mark.parametrize(("case", "policy"), ((3, "qualifying_policy.yaml"), (6, "final_policy.yaml"), (7, "final_policy.yaml")))
def test_perception_cases_start_the_signal_phase_tracker(case, policy) -> None:
    parameters = launch_parameters(platform="simulation", case=str(case))

    tracker = parameters["signal_phase_tracker_node"]
    assert tracker["use_sim_time"] is True
    assert Path(tracker["mission_policy_config"]).name == policy
    assert tracker["mission_policy_config"] == parameters["route_zone_manager_node"]["mission_policy_config"]
    assert tracker["map_semantics_path"].endswith("maps/kcity/kcity_course_semantics.yaml")


def test_signal_phase_tracker_skips_cases_without_perception() -> None:
    for case in ("1", "4"):
        assert "signal_phase_tracker_node" not in launch_parameters(case=case)
