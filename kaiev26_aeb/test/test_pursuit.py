import math

import numpy as np
import pytest

from kaiev26_aeb.pursuit import Pursuit, PursuitConfig, advance_path, pursuit_target


def line(offset=0.0, end=20):
    return np.column_stack((np.linspace(0, end, 81), np.full(81, offset)))


def step(controller, points=None, **kw):
    args = dict(points=line() if points is None else points, speed=8.333333,
                steering=0.0, path_age=0.02, state_age=0.01, dt=0.02)
    args.update(kw)
    return controller.step(**args)


def test_straight_reaches_30_kph_without_steering():
    c = Pursuit()
    out = [step(c) for _ in range(500)]
    assert out[-1]['speed_target_mps'] == pytest.approx(30 / 3.6)
    assert all(x['steering_target_rad'] == 0 and not x['brake_engage'] for x in out)
    assert max(np.diff([x['speed_target_mps'] for x in out])) <= 0.020000001


@pytest.mark.parametrize('offset', [-0.8, 0.8])
def test_turn_sign_and_coulter_geometry(offset):
    c = Pursuit()
    for _ in range(100):
        r = step(c, line(offset))
    x, y = r['target_xy']
    assert r['steering_target_rad'] == pytest.approx(math.atan(2 * c.cfg.wheelbase_m * y / (x*x + y*y)))
    assert r['steering_target_rad'] * offset > 0


@pytest.mark.parametrize('kwargs,reason', [
    ({'state_age': 0.251}, 'stale_state'),
    ({'path_age': -1}, 'clock_mismatch'),
    ({'speed': float('nan')}, 'invalid_state_or_time'),
    ({'steering': float('inf')}, 'invalid_state_or_time'),
    ({'points': [[0, 0], [2, float('nan')]]}, 'invalid_path'),
    ({'points': [[5, 0], [0, 0]]}, 'invalid_path'),
    ({'frame': 'map'}, 'wrong_path_frame'),
    ({'estop': True}, 'estop'),
    ({'reverse': True}, 'reverse'),
    ({'enabled': False}, 'disabled'),
])
def test_invalid_inputs_brake_immediately(kwargs, reason):
    c = Pursuit()
    for _ in range(300):
        step(c, line(0.5))
    r = step(c, **kwargs)
    assert r['reason'] == reason
    assert r['brake_engage'] and r['speed_target_mps'] == 0


@pytest.mark.parametrize('kwargs,reason', [({'points': []}, 'empty_path'),
                                           ({'path_age': 0.351}, 'stale_path')])
def test_cone_path_gap_holds_the_command_for_half_a_second(kwargs, reason):
    c = Pursuit()
    for _ in range(300):
        previous = step(c, line(0.5))
    held = [step(c, **kwargs) for _ in range(25)]  # 0.5 s at 50 Hz
    assert all(r['reason'] == reason and r['command_held'] and not r['brake_engage'] for r in held)
    assert all(r['speed_target_mps'] == previous['speed_target_mps'] for r in held)
    assert all(r['steering_target_rad'] == previous['steering_target_rad'] for r in held)
    r = step(c, **kwargs)
    assert r['brake_engage'] and r['speed_target_mps'] == 0


def test_path_gap_timer_restarts_after_a_valid_path():
    c = Pursuit()
    for _ in range(300):
        step(c)
    for _ in range(3):
        assert all(not step(c, points=[])['brake_engage'] for _ in range(20))
        step(c)
    assert c.path_gap_s == 0.0


def test_tracking_after_a_stop_ramps_on_from_the_measured_speed():
    c = Pursuit()
    for _ in range(300):
        step(c)
    for _ in range(30):
        stop = step(c, points=[])
    assert stop['brake_engage'] and stop['speed_target_mps'] == 0
    r = step(c, speed=6.0)
    assert r['speed_target_mps'] == pytest.approx(6.0 + c.cfg.acceleration_mps2 * 0.02)


def test_path_gap_before_tracking_and_after_red_latch_still_brakes():
    assert step(Pursuit(), points=[])['brake_engage']
    c = Pursuit(PursuitConfig(red_brake_lead_m=5.0))
    for _ in range(100):
        step(c)
    gate = [[3.0, 2.5], [3.0, -2.5]]
    step(c, red_gate=gate, red_gate_stamp=1, red_gate_age=0.)
    step(c, red_gate=gate, red_gate_stamp=2, red_gate_age=0.)
    assert c.red.latched
    assert step(c, points=[])['brake_engage']


def test_short_target_loss_holds_previous_command_without_braking():
    c = Pursuit()
    for _ in range(300):
        previous = step(c, line(0.5))
    result = step(c, points=line(end=2))
    assert result['reason'] == 'short_path_or_no_forward_target'
    assert not result['tracking'] and result['command_held']
    assert not result['brake_engage']
    assert result['speed_target_mps'] == previous['speed_target_mps']
    assert result['steering_target_rad'] == previous['steering_target_rad']


def test_acquisition_motion_compensation():
    p = np.array([[10.0, 0.0], [15.0, 0.0]])
    assert np.allclose(advance_path(p, 8, 0, 0.2, 1.3), p - [1.6, 0])
    result = advance_path(p, 8, 0.1, 0.2, 1.3)
    assert result[0, 1] < 0  # A left-turning car sees the old straight line on its right.


def test_interpolated_target_and_no_extension():
    assert pursuit_target(np.array([[0., 0.], [10., 0.]]), 3.3)[0] == pytest.approx(3.3)
    assert pursuit_target(np.array([[0., 0.], [2., 0.]]), 3.3) is None


def test_kinematic_30_kph_converges_from_offset_and_heading():
    # Independent bicycle integration, ideal long straight road; not a Gazebo claim.
    c = Pursuit()
    x, y, yaw, speed = 0., 0.8, 0.05, 30/3.6
    errors = []
    for _ in range(800):
        world = np.column_stack((np.linspace(x - 2, x + 25, 100), np.zeros(100)))
        cs, sn = math.cos(yaw), math.sin(yaw)
        path = (world - [x, y]) @ np.array([[cs, -sn], [sn, cs]])
        r = step(c, path, path_age=0., speed=speed)
        assert r['tracking']
        yaw += speed / c.cfg.wheelbase_m * math.tan(r['steering_target_rad']) * 0.02
        x += speed * math.cos(yaw) * 0.02
        y += speed * math.sin(yaw) * 0.02
        errors.append(y)
    assert max(abs(v) for v in errors) < 1.0
    assert abs(y) < 0.02


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        Pursuit(PursuitConfig(target_speed_kph=float('nan')))
    with pytest.raises(ValueError):
        Pursuit(PursuitConfig(red_tracking_stop_speed_mps=-0.1))


def test_fixed_seven_meters_at_all_speeds():
    for speed in (0., 3., 30/3.6, 12.):
        r = step(Pursuit(), speed=speed)
        assert r['lookahead_m'] == 7.0
        assert np.linalg.norm(r['target_xy']) == pytest.approx(7.0)
