from pathlib import Path

import yaml

from kaiev26_aeb.trial_profiles import PROFILES


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_public_launch_surface_is_compact():
    assert {path.name for path in (PACKAGE_ROOT / 'launch').glob('*.launch.py')} == {
        'aeb_test.launch.py',
        'aeb_sim.launch.py',
    }


def test_profiles_reference_installed_node_aliases():
    assert {profile.perception_node for profile in PROFILES.values()} == {
        'cone_path_node',
        'cone_yolo_path_node',
    }
    assert {profile.controller_node for profile in PROFILES.values()} == {
        'cone_pursuit_node',
        'cone_stanley_node',
    }


def test_combined_parameter_files_cover_all_nodes():
    perception = yaml.safe_load(
        (PACKAGE_ROOT / 'config/perception.yaml').read_text(encoding='utf-8'))
    tracking = yaml.safe_load(
        (PACKAGE_ROOT / 'config/tracking.yaml').read_text(encoding='utf-8'))
    assert set(perception) == {'cone_path_node', 'cone_yolo_path_node'}
    assert set(tracking) == {'cone_pursuit_node', 'cone_stanley_node'}


def test_perception_modes_accept_narrower_observed_corridors():
    perception = yaml.safe_load(
        (PACKAGE_ROOT / 'config/perception.yaml').read_text(encoding='utf-8'))
    assert perception['cone_path_node']['ros__parameters']['corridor_width_min'] == 1.0
    assert perception['cone_yolo_path_node']['ros__parameters']['corridor_width_min'] == 1.0
