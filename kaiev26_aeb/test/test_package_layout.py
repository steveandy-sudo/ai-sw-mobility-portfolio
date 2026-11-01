from pathlib import Path
import runpy

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from kaiev26_aeb.trial_profiles import PROFILES


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('scenario', [1, 2])
def test_trial_launch_fixes_acceleration_for_both_controllers(monkeypatch, scenario):
    module = runpy.run_path(str(PACKAGE_ROOT / 'launch/aeb_test.launch.py'))
    description = module['generate_launch_description']()
    arguments = {item.name for item in description.entities
                 if isinstance(item, DeclareLaunchArgument)}
    assert 'acceleration_mps2' not in arguments
    build = module['_build']
    monkeypatch.setitem(build.__globals__, 'get_package_share_directory',
                        lambda _: str(PACKAGE_ROOT))
    monkeypatch.setitem(build.__globals__, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'scenario': str(scenario), 'use_sim_time': 'false', 'enabled': 'false',
        'command_topic': '/planning/command', 'target_speed_kph': '30',
        'red_brake_lead_m': '5', 'acceleration_mps2': '2.0',
    })
    tracker = build(context)[-1]
    assert tracker['executable'] == PROFILES[scenario].controller_node
    assert tracker['parameters'][-1]['acceleration_mps2'] == 1.0


def test_trial_launch_defaults_to_stanley():
    module = runpy.run_path(str(PACKAGE_ROOT / 'launch/aeb_test.launch.py'))
    defaults = {item.name: item.default_value for item in module['generate_launch_description']().entities
                if isinstance(item, DeclareLaunchArgument)}
    assert ''.join(part.text for part in defaults['scenario']) == '2'


def test_trial_launch_passes_the_vehicle_brake(monkeypatch):
    module = runpy.run_path(str(PACKAGE_ROOT / 'launch/aeb_test.launch.py'))
    build = module['_build']
    monkeypatch.setitem(build.__globals__, 'get_package_share_directory', lambda _: str(PACKAGE_ROOT))
    monkeypatch.setitem(build.__globals__, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'scenario': '2', 'use_sim_time': 'false', 'enabled': 'false',
        'command_topic': '/planning/command', 'target_speed_kph': '30',
        'red_brake_lead_m': '5', 'vehicle_brake': 'false',
    })
    assert 'vehicle_brake' in build(context)[-1]['parameters'][-1]
