import math

import numpy as np
import pytest

from kaiev26_aeb.core import Config, plan_corridor
from kaiev26_aeb.messages import output_messages, empty_result
from kaiev26_aeb.pursuit import Pursuit, PursuitConfig


def tick(control, gate=None, stamp=None, age=0., speed=0., steering=0., points=None, **kwargs):
    path = np.array([[0., 0.], [20., 0.]]) if points is None else np.asarray(points, dtype=float)
    return control.step(path, speed=speed, steering=steering,
                        path_age=0., state_age=0., dt=.02, red_gate=gate,
                        red_gate_stamp=stamp, red_gate_age=age, **kwargs)


def test_repeated_control_ticks_are_not_new_camera_confirmations():
    c = Pursuit()
    for _ in range(10):
        r = tick(c, [[1.9, 2.5], [1.9, -2.5]], 1)
    assert r['tracking'] and r['red_confirm_count'] == 1
    r = tick(c, [[1.9, 2.5], [1.9, -2.5]], 2)
    assert r['reason'] == 'red_stop_latched'
    assert r['brake_engage'] and r['speed_target_mps'] == 0


@pytest.mark.parametrize('yaw_deg', [-25, -15, 0, 15, 25])
def test_gate_crossing_uses_front_bumper_and_rotated_gate(yaw_deg):
    a = math.radians(yaw_deg)
    rotation = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    for forward in [5., 4., 2.1, 1.7]:
        c = Pursuit()
        gate = np.array([[forward, 2.5], [forward, -2.5]]) @ rotation
        tick(c, gate, 1)
        r = tick(c, gate, 2)
        distance = forward - c.cfg.red_front_offset_m * math.cos(a)
        assert r['red_distance_m'] == pytest.approx(distance)
        assert r['red_latched'] == (distance <= 0)


def test_confirmed_entry_survives_occlusion_and_ignores_later_red_rows():
    c = Pursuit()
    tick(c, [[5., 2.5], [5., -2.5]], 1)
    r = tick(c, [[5., 2.5], [5., -2.5]], 2)
    assert r['red_state'] == 'approaching'
    tick(c, [[8., 2.5], [8., -2.5]], 3)
    assert c.red.gate[0, 0] == pytest.approx(5.)
    for stamp in range(4, 100):
        r = tick(c, [], stamp, speed=8.)
        if r['red_latched']:
            break
    assert r['red_latched']
    for _ in range(20):
        r = c.step([], speed=8., steering=0., path_age=100., state_age=100., dt=.02)
        assert r['reason'] == 'stale_state' and r['brake_engage']
        assert r['speed_target_mps'] == 0
    assert tick(c, enabled=False)['red_latched']
    c.red.reset()
    assert tick(c)['tracking'] and not c.red.latched


@pytest.mark.parametrize('gate,age', [([[1.8, 2.5]], 0.),
    ([[1.8, 2.5], [1.8, -2.5]], .36), ([[1.8, 2.5], [1.8, -2.5]], -.1),
    ([[1.8, float('nan')], [1.8, -2.5]], 0.), ([[1.8, -2.5], [1.8, 2.5]], 0.)])
def test_invalid_or_stale_red_observations_do_not_trigger(gate, age):
    c = Pursuit()
    tick(c, gate, 1, age)
    assert not tick(c, gate, 2, age)['red_latched']


def test_one_frame_red_detection_is_not_remembered_as_confirmed():
    c = Pursuit()
    tick(c, [[5., 2.5], [5., -2.5]], 1)
    tick(c, [], 2)
    for stamp in range(3, 60):
        assert not tick(c, [], stamp, speed=8.)['red_latched']


def test_gate_acquisition_age_is_compensated_before_entry_check():
    c = Pursuit()
    gate = [[3.2, 2.5], [3.2, -2.5]]
    tick(c, gate, 1, age=.2, speed=8.)
    r = tick(c, gate, 2, age=.2, speed=8.)
    assert r['red_latched'] and r['red_distance_m'] == pytest.approx(3.2-1.6-1.98)


def test_new_case_can_latch_five_meters_before_red_line():
    c = Pursuit(PursuitConfig(red_brake_lead_m=5.0))
    gate = [[6.8, 2.5], [6.8, -2.5]]
    tick(c, gate, 1)
    r = tick(c, gate, 2)
    assert r['red_latched']
    assert r['red_distance_m'] == pytest.approx(6.8 - 1.98)
    assert r['speed_target_mps'] == 0.0 and r['brake_engage']
    assert r['steering_target_rad'] == pytest.approx(0.0)
    assert not tick(Pursuit(), gate, 1)['red_latched']


def test_red_braking_tracks_centerline_until_half_meter_per_second_then_freezes():
    c = Pursuit(PursuitConfig(red_brake_lead_m=5.0))
    gate = [[6.8, 2.5], [6.8, -2.5]]
    path = [[0., 0.8], [20., 0.8]]
    tick(c, gate, 1, speed=3.0, points=path)
    braking = tick(c, gate, 2, speed=3.0, points=path)

    assert braking['reason'] == 'red_stop_tracking'
    assert braking['tracking'] and braking['brake_engage']
    assert braking['speed_target_mps'] == 0.0
    assert braking['steering_target_rad'] > 0.0
    assert not braking['red_steering_frozen']

    stopped = tick(c, speed=0.5, points=[[0., -0.8], [20., -0.8]])
    assert stopped['reason'] == 'red_stop_latched'
    assert not stopped['tracking'] and stopped['brake_engage']
    assert stopped['steering_target_rad'] == braking['steering_target_rad']
    assert stopped['red_steering_frozen']

    rebound = tick(c, speed=0.6, points=[[0., -0.8], [20., -0.8]])
    assert rebound['reason'] == 'red_stop_latched'
    assert rebound['steering_target_rad'] == braking['steering_target_rad']


def test_red_braking_keeps_brake_engaged_when_centerline_is_lost():
    c = Pursuit(PursuitConfig(red_brake_lead_m=5.0))
    gate = [[6.8, 2.5], [6.8, -2.5]]
    tick(c, gate, 1, speed=3.0)
    tick(c, gate, 2, speed=3.0)
    result = tick(c, speed=3.0, points=[])
    assert result['reason'] == 'empty_path'
    assert result['red_latched'] and result['brake_engage']
    assert result['speed_target_mps'] == 0.0


def test_geometry_to_ros_red_gate_and_explicit_empty_message():
    cones = [dict(xyz=np.array([x, y, .1]), color=color, side='unassigned',
                  score=.9, indices=[0, 1, 2], matches=[])
             for x in [4., 7., 10., 13.] for y, color in [(2.5, 'red'), (-2.5, 'red')]]
    plan = plan_corridor(cones, Config())
    result = dict(cones=cones, plan=plan, plane=np.array([0., 0., -.24]))
    messages = output_messages(result, 1234567890, 'base_link')
    gate = messages['/aeb/red_gate']
    assert gate.header.frame_id == 'base_link'
    assert [(p.position.x, p.position.y) for p in gate.poses] == [(4., 2.5), (4., -2.5)]
    assert any(m.ns == 'red_entry' for m in messages['/aeb/boundaries'].markers)
    assert not output_messages(empty_result('waiting_for_points'), 1234567891, 'base_link')['/aeb/red_gate'].poses
