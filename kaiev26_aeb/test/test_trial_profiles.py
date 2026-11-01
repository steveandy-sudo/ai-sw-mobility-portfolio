from dataclasses import replace
from pathlib import Path

import pytest

from kaiev26_aeb.trial_processes import RECORDER_CODE, ProcessManager
from kaiev26_aeb.trial_profiles import (PROFILES, TrialSettings,
                                        default_target_speed_kph, profile_for)


def test_scenario_matrix_is_stable():
    assert {
        number: (profile.perception, profile.controller)
        for number, profile in PROFILES.items()
    } == {
        1: ('colored', 'pure_pursuit'),
        2: ('colored', 'stanley'),
    }


def test_only_existing_component_pairs_are_available(tmp_path: Path):
    assert profile_for(2).missing_launches(tmp_path) == ('aeb_test.launch.py',)
    (tmp_path / 'aeb_test.launch.py').touch()
    assert all(not p.missing_launches(tmp_path) for p in PROFILES.values())


def test_renumbered_cases_keep_old_colored_case_default_speed():
    assert [default_target_speed_kph(scenario) for scenario in PROFILES] == [
        30.0, 30.0,
    ]
    assert TrialSettings().target_speed_kph == 30.0
    assert TrialSettings().acceleration_mps2 == 1.0


def test_aeb_mcap_uses_fast_zstd_storage_preset():
    assert "storage_preset_profile='zstd_fast'" in RECORDER_CODE


def test_settings_validate_case_speed_and_brake_lead():
    assert TrialSettings(2, 30.0).profile.name == '컬러 콘 3D 인지 + Stanley'
    for scenario, speed in ((0, 5.0), (3, 5.0), (2, 0.0),
                            (2, float('nan'))):
        with pytest.raises(ValueError):
            TrialSettings(scenario, speed)
    assert TrialSettings(1, 30.0).profile.perception == 'colored'
    with pytest.raises(ValueError):
        TrialSettings(2, 30.0, red_brake_lead_m=-1)


@pytest.mark.parametrize('scenario', [1, 2])
def test_trial_acceleration_is_fixed_and_recorded(scenario):
    settings = TrialSettings(scenario, 30.0)
    assert settings.acceleration_mps2 == 1.0
    assert settings.document()['acceleration_mps2'] == 1.0
    assert replace(settings, target_speed_kph=25.0).acceleration_mps2 == 1.0
    with pytest.raises(TypeError):
        TrialSettings(scenario, acceleration_mps2=1.5)
    with pytest.raises(ValueError):
        replace(settings, acceleration_mps2=1.5)


def test_process_command_starts_disabled_through_governor():
    settings = TrialSettings(1, 30.0, red_brake_lead_m=4.5)
    argv = ProcessManager.launch_argv(settings)
    assert argv[:4] == ['ros2', 'launch', 'kaiev26_aeb', 'aeb_test.launch.py']
    assert 'scenario:=1' in argv
    assert 'enabled:=false' in argv
    assert 'command_topic:=/planning/command' in argv
    assert 'target_speed_kph:=30' in argv
    assert not any(arg.startswith('acceleration_mps2:=') for arg in argv)
    assert 'red_brake_lead_m:=4.5' in argv
    assert 'model_path' not in settings.document()


def test_second_case_passes_speed_and_brake_settings():
    settings = TrialSettings(2, 30.0, red_brake_lead_m=5.0)
    argv = ProcessManager.launch_argv(settings)
    assert 'scenario:=2' in argv
    assert 'target_speed_kph:=30' in argv
    assert not any(arg.startswith('acceleration_mps2:=') for arg in argv)
    assert 'red_brake_lead_m:=5' in argv
    assert 'model_path' not in settings.document()


def test_trial_tui_runs_stanley_with_speed_and_brake_lead_defaults():
    settings = TrialSettings()
    assert settings.profile.controller == 'stanley'
    assert (settings.target_speed_kph, settings.red_brake_lead_m) == (30.0, 5.0)
    assert 'scenario:=2' in ProcessManager.launch_argv(settings)


@pytest.mark.parametrize('estop_use,vehicle_brake', [(False, 'true'), (True, 'false')])
def test_estop_mode_sets_the_vehicle_brake(estop_use, vehicle_brake):
    settings = TrialSettings(estop_use=estop_use)
    assert f'vehicle_brake:={vehicle_brake}' in ProcessManager.launch_argv(settings)
    assert settings.document()['estop'] == ('USE' if estop_use else 'NOT-USE')
    assert TrialSettings().estop_use is False


def test_mcap_keeps_vehicle_response_steering_detail_and_logs():
    from kaiev26_aeb.trial_processes import RECORD_TOPICS
    for topic in ('/imu/data', '/vehicle/wheel_twist', '/steering/requested_angle',
                  '/steering/angle_error', '/steering/torque_command_raw', '/steering/torque_saturated',
                  '/rosout', '/parameter_events'):
        assert topic in RECORD_TOPICS
    assert len(RECORD_TOPICS) == len(set(RECORD_TOPICS))


def test_scenario_keeps_the_stanley_values_it_runs_with(monkeypatch):
    from kaiev26_aeb import stanley
    from kaiev26_aeb.trial_processes import stanley_document
    source = Path(__file__).resolve().parents[2] / 'kaiev26_motion_control/config/motion_control.yaml'
    monkeypatch.setattr(stanley, 'motion_control_yaml_path', lambda: source)
    document = stanley_document(TrialSettings())
    assert document['source'] == str(source)
    assert document == {'source': str(source), **stanley.motion_control_defaults(source)}
    assert stanley_document(TrialSettings(1)) is None
    monkeypatch.setattr(stanley, 'motion_control_yaml_path', lambda: Path('/missing.yaml'))
    assert 'error' in stanley_document(TrialSettings())
