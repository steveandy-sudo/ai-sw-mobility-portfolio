from pathlib import Path

import pytest

from kaiev26_aeb.trial_processes import ProcessManager
from kaiev26_aeb.trial_profiles import PROFILES, TrialSettings, profile_for


def test_four_scenario_matrix_is_stable():
    assert {
        number: (profile.perception, profile.controller)
        for number, profile in PROFILES.items()
    } == {
        1: ('fusion', 'stanley'),
        2: ('fusion', 'pure_pursuit'),
        3: ('yolo', 'stanley'),
        4: ('yolo', 'pure_pursuit'),
    }


def test_only_existing_component_pairs_are_available(tmp_path: Path):
    assert profile_for(2).missing_launches(tmp_path) == ('aeb_test.launch.py',)
    (tmp_path / 'aeb_test.launch.py').touch()
    assert all(not p.missing_launches(tmp_path) for p in PROFILES.values())


def test_settings_validate_scenario_speed_and_model():
    assert TrialSettings(2, 5.0, '/tmp/best.pt').profile.name == (
        '라바콘 퓨전 + Pure Pursuit')
    for scenario, speed, model in ((0, 5.0, '/tmp/best.pt'),
                                   (5, 5.0, '/tmp/best.pt'),
                                   (2, 0.0, '/tmp/best.pt'),
                                   (2, float('nan'), '/tmp/best.pt'),
                                   (2, 5.0, '')):
        with pytest.raises(ValueError):
            TrialSettings(scenario, speed, model)


def test_process_command_starts_disabled_through_governor():
    settings = TrialSettings(2, 7.5, '/models/best.pt')
    argv = ProcessManager.launch_argv(settings)
    assert argv[:4] == ['ros2', 'launch', 'kaiev26_aeb', 'aeb_test.launch.py']
    assert 'scenario:=2' in argv
    assert 'enabled:=false' in argv
    assert 'command_topic:=/planning/command' in argv
    assert 'target_speed_kph:=7.5' in argv
    assert 'model_path:=/models/best.pt' in argv
