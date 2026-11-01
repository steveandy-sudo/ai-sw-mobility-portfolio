"""Captured-route launches: curve caps from the captured corners in tracking (13) and avoidance
(17); avoidance has no finish zone ahead of the route end."""
import math

import pytest
import yaml

from kaiev26_decision.curve_speed_policy import load_curve_speed_policy
from kaiev26_decision.route_capture import make_tracking_profile
from scenario_harness.launch_nodes import node_specs


def captured_route(tmp_path):
    points = [(float(x), 0.0) for x in range(41)]
    for k in range(1, 16):
        angle = k / 15 * math.pi / 2
        points.append((40.0 + 10.0 * math.sin(angle), 10.0 * (1 - math.cos(angle))))
    points += [(50.0, 10.0 + float(y)) for y in range(1, 81)]
    document = dict(route_name='curve', route_id='run', capture_mode='manual_tui', frame_id='map',
                    reference_point='base_footprint', map_origin={'x': 0.0, 'y': 0.0},
                    waypoints=[dict(x=x, y=y, z=0.0, zone='NORMAL_ZONE') for x, y in points])
    document['tracking_profile'] = make_tracking_profile(document)
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump(document))
    return str(path)


def node_parameters(route, case):
    specs = node_specs({'case': case, 'speed_mps': 7.0, 'route_config': route})
    return {spec.executable: spec.parameters for spec in specs}


@pytest.mark.parametrize('case', ['13', '17'])
def test_captured_route_reaches_the_route_manager_and_motion_control(tmp_path, case):
    route = captured_route(tmp_path)
    nodes = node_parameters(route, case)
    assert nodes['route_zone_manager_node']['route_config'] == route
    assert not nodes['route_zone_manager_node']['landmark_config']
    phases = nodes['motion_control_node']['hybrid_route_segment_phases']
    assert phases[0] == 'straight' and 'turning' in phases
    if case == '17':
        # Only the gateway's perceived lanes are moved out of the way.
        gateway = nodes['perception_gateway_node']
        assert gateway['centerline_topic'].startswith('/decision/midan_unused/')
        assert gateway['road_segments_topic'].startswith('/decision/midan_unused/')


@pytest.mark.parametrize('case', ['13', '17'])
def test_captured_route_curves_are_capped_in_tracking_and_avoidance(tmp_path, case):
    engine = node_parameters(captured_route(tmp_path), case)['main_planning_engine_node']
    policy = load_curve_speed_policy(
        engine['route_config'], lateral_accel_mps2=float(engine.get('curve_lateral_accel_mps2', 0.0)))
    assert policy is not None and len(policy.zones) == 1
    assert policy.speed_scale == pytest.approx(0.8)
    assert math.isfinite(policy.zones[0].radius_m)


@pytest.mark.parametrize('case,finish_zone,finish_approach', [('13', 30.0, 20.0), ('17', 0.0, 0.1)])
def test_only_captured_route_avoidance_drops_the_finish_zone(tmp_path, case, finish_zone, finish_approach):
    nodes = node_parameters(captured_route(tmp_path), case)
    assert nodes['route_zone_manager_node']['finish_zone_distance_m'] == pytest.approx(finish_zone)
    engine = nodes['main_planning_engine_node']
    assert engine.get('finish_approach_distance_m', 20.0) == pytest.approx(finish_approach)


@pytest.mark.parametrize('case', ['13', '17'])
def test_bypass_speeds_and_stanley_only_tracking_are_the_common_defaults(tmp_path, case):
    # The bypass speeds are the defaults for every course, not a case 17 override (10/24: 5 m/s
    # for forced paths too, so the gap no longer slows the bypass).
    nodes = node_parameters(captured_route(tmp_path), case)
    engine = nodes['main_planning_engine_node']
    assert (engine.get('obstacle_bypass_speed_mps'), engine.get('obstacle_forced_speed_mps')) == (5.0, 5.0)
    assert engine.get('obstacle_lateral_accel_mps2') == 6.0
    motion = nodes['motion_control_node']
    assert [p for p in motion.get('stanley_only_path_source_prefixes', []) if p] == [
        'sequential_bypass', 'forced_sequential_bypass']
