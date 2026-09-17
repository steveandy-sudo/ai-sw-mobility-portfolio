import math
from collections import deque

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from kaiev26_aeb.messages import header, predicted_path_marker
from kaiev26_aeb.stanley import Stanley, StanleyConfig


def step(c, path=None, **kwargs):
    inputs = dict(points=[[0., 0.], [15., 0.]] if path is None else path,
                  speed=35/3.6, steering=0., path_age=0., state_age=0., dt=.02)
    inputs.update(kwargs)
    return c.step(**inputs)


@pytest.mark.parametrize('offset', [-.8, .8])
def test_front_axle_projection_sign_and_speed_softening(offset):
    c = Stanley()
    slow = step(c, [[0., offset], [15., offset]], speed=0.)
    fast = step(c, [[0., offset], [15., offset]])
    assert fast['target_xy'] == pytest.approx([c.cfg.wheelbase_m, offset])
    assert fast['cross_track_error_m'] == pytest.approx(offset)
    assert fast['raw_steering_rad'] * offset > 0
    assert abs(fast['raw_steering_rad']) < abs(slow['raw_steering_rad'])
    assert math.isfinite(slow['raw_steering_rad'])
    assert fast['lookahead_m'] is None  # No PP target/circle in Stanley.


@pytest.mark.parametrize('yaw', [-15., 15.])
def test_heading_error_includes_front_axle_offset(yaw):
    c = Stanley()
    angle = math.radians(yaw)
    path = np.array([[0., 0.], [15., -15*math.tan(angle)]])
    r = step(c, path)
    assert r['heading_error_rad'] == pytest.approx(-angle)
    assert r['cross_track_error_m'] == pytest.approx(-c.cfg.wheelbase_m*math.sin(angle))
    assert r['raw_steering_rad'] * yaw < 0


@pytest.mark.parametrize('inputs,reason', [
    ({'points': []}, 'empty_path'), ({'path_age': .36}, 'stale_path'),
    ({'state_age': .26}, 'stale_state'), ({'enabled': False}, 'disabled'),
    ({'estop': True}, 'estop'), ({'reverse': True}, 'reverse'),
    ({'speed': float('nan')}, 'invalid_state_or_time'),
    ({'frame': 'map'}, 'wrong_path_frame'),
    ({'points': [[0, 0], [.5, 0]]}, 'no_front_axle_reference'),
    ({'points': [[5, 0], [10, 0]]}, 'no_front_axle_reference'),
    ({'points': [[1, 0], [1, 0]]}, 'no_front_axle_reference'),
])
def test_shared_stop_and_unsupported_reference(inputs, reason):
    c = Stanley()
    for _ in range(100):
        step(c, [[0., .5], [15., .5]])
    previous = c.steering_command
    r = step(c, **inputs)
    assert r['reason'] == reason
    assert r['brake_engage'] and r['speed_target_mps'] == 0
    assert r['steering_target_rad'] == previous


def test_red_stop_latches_and_visuals_work_without_pp_lookahead():
    c = Stanley()
    r = step(c)
    marker = predicted_path_marker(r, c.cfg.wheelbase_m, header(0, 'base_link'))
    assert marker.action == Marker.ADD and marker.points[-1].x == pytest.approx(7.)
    gate = [[1.9, 2.5], [1.9, -2.5]]
    step(c, red_gate=gate, red_gate_stamp=1, red_gate_age=0.)
    r = step(c, red_gate=gate, red_gate_stamp=2, red_gate_age=0.)
    assert r['red_latched'] and r['brake_engage']
    assert step(c, [], path_age=99., state_age=99.)['reason'] == 'red_stop_latched'
    assert predicted_path_marker(r, c.cfg.wheelbase_m, header(1, 'base_link')).action == Marker.DELETE


@pytest.mark.parametrize('yaw_deg', [-15, 15])
def test_ideal_bicycle_recovers_from_15_degree_start(yaw_deg):
    c = Stanley(StanleyConfig(target_speed_kph=35.))
    x, y, yaw, speed = 0., 0., math.radians(yaw_deg), 0.
    errors = []
    for _ in range(600):
        world = np.array([[x - 3, 0.], [x + 20, 0.]])
        cs, sn = math.cos(yaw), math.sin(yaw)
        path = (world - [x, y]) @ np.array([[cs, -sn], [sn, cs]])
        r = step(c, path, speed=speed)
        assert r['tracking']
        speed = r['speed_target_mps']
        yaw += speed/c.cfg.wheelbase_m*math.tan(r['steering_target_rad'])*.02
        x += speed*math.cos(yaw)*.02
        y += speed*math.sin(yaw)*.02
        errors.append(y)
    assert abs(y) < .02 and abs(yaw) < .01
    assert max(map(abs, errors)) < .5
    assert speed == pytest.approx(35/3.6)


@pytest.mark.parametrize('yaw_deg', [-15, 15])
def test_delayed_steering_does_not_develop_growing_oscillation(yaw_deg):
    # Supplemental tuning regression, not a substitute for full Gazebo sensors/plant.
    # Same measured delay/time constant as the plant, 10 Hz path / 50 Hz control.
    c = Stanley(StanleyConfig(target_speed_kph=35.))
    x = y = speed = actual_steering = 0.
    yaw = math.radians(yaw_deg)
    commands = deque([0.] * 11)  # 0.22 s at 50 Hz, rounds the plant's 0.215 s.
    errors = []
    for tick in range(1000):
        if tick % 5 == 0:
            world = np.array([[x-3, 0.], [x+20, 0.]])
            cs, sn = math.cos(yaw), math.sin(yaw)
            path = (world - [x, y]) @ np.array([[cs, -sn], [sn, cs]])
        r = step(c, path, speed=speed, steering=actual_steering, path_age=tick % 5 * .02)
        assert r['tracking']
        commands.append(r['steering_target_rad'])
        actual_steering += (commands.popleft()-actual_steering)*(1-math.exp(-.02/.439))
        speed = min(r['speed_target_mps'], max(0., (tick*.02-2.5)*.8))
        yaw += speed/c.cfg.wheelbase_m*math.tan(actual_steering)*.02
        x += speed*math.cos(yaw)*.02
        y += speed*math.sin(yaw)*.02
        errors.append(abs(y))
    assert max(errors) < .5
    assert max(errors[-200:]) < .25


@pytest.mark.parametrize('kw', [{'heading_gain': 0.}, {'cross_track_gain': 0.}, {'soft_speed_mps': float('nan')},
                               {'max_steering_rad': math.pi}])
def test_invalid_config(kw):
    with pytest.raises(ValueError):
        Stanley(StanleyConfig(**kw))
