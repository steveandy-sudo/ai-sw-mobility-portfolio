"""The cone-trial Stanley steers with the qualifying/final tracker's values and corrections."""
import math
from pathlib import Path

import numpy as np
import pytest

from kaiev26_aeb.stanley import MOTION_CONTROL_PARAMETERS, Stanley, StanleyConfig, motion_control_defaults
from kaiev26_motion_control.motion_control_node import scheduled_stanley_gains

MOTION_YAML = Path(__file__).resolve().parents[2] / 'kaiev26_motion_control/config/motion_control.yaml'


def straight(c, offset=0.0, speed=30/3.6, steering=0.0, **kwargs):
    inputs = dict(points=[[0., offset], [15., offset]], speed=speed, steering=steering,
                  path_age=0., state_age=0., dt=.02)
    inputs.update(kwargs)
    return c.step(**inputs)


def test_defaults_are_the_mission_tracker_values():
    shipped = motion_control_defaults(MOTION_YAML)
    assert set(shipped) == set(MOTION_CONTROL_PARAMETERS)
    defaults = vars(StanleyConfig())
    for key, value in shipped.items():
        assert defaults[key] == pytest.approx(value), key


def test_tracking_yaml_does_not_override_the_shared_values():
    import yaml
    tracking = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/tracking.yaml')
                              .read_text())['cone_stanley_node']['ros__parameters']
    assert not set(tracking) & set(MOTION_CONTROL_PARAMETERS)


@pytest.mark.parametrize('speed', [0.0, 2.0, 4.5, 30/3.6, 12.0])
def test_straight_gains_follow_the_mission_schedule(speed):
    c = Stanley()
    cfg = c.cfg
    full = scheduled_stanley_gains(max(speed, cfg.gain_full_speed_mps), cfg.natural_frequency_radps,
                                   cfg.damping_ratio, cfg.wheelbase_m, cfg.soft_speed_mps,
                                   cfg.gain_full_speed_mps)
    share = min(1.0, max(0.0, (speed - 3.0) / 3.0))
    expected = (0.30 + share * (full[0] - 0.30), 0.25 + share * (full[1] - 0.25))
    assert c.straight_gains(speed) == pytest.approx(expected)
    if speed >= 6.0:
        assert c.straight_gains(speed) == pytest.approx(full)


def test_neutral_offset_is_added_when_the_encoder_follows():
    c = Stanley()
    # Encoder already at the offset command: no friction push, only the neutral offset.
    offset = c.cfg.steering_neutral_offset_deg
    r = straight(c, steering=math.radians(offset))
    assert r['steering_compensation_rad'] == pytest.approx(math.radians(offset))


@pytest.mark.parametrize('encoder_deg,sign', [(-3.0, 1.0), (2.0, -1.0)])
def test_friction_push_drives_toward_the_encoder_shortfall(encoder_deg, sign):
    c = Stanley()
    for _ in range(20):
        r = straight(c, steering=math.radians(encoder_deg))
    push = r['steering_compensation_rad'] - math.radians(c.cfg.steering_neutral_offset_deg)
    assert push * sign > 0
    # 30 km/h lies between the 6.5 and 12 m/s friction values.
    low, high = c.cfg.steering_friction_comp_deg[1:]
    assert abs(push) == pytest.approx(math.radians(low + (high - low) * (30/3.6 - 6.5) / 5.5), rel=1e-6)


def test_corrections_are_clamped_to_the_straight_limits():
    c = Stanley()
    r = straight(c, offset=3.0)
    assert r['cross_track_correction_rad'] == pytest.approx(0.05)
    angle = math.radians(30)
    r = c.step(points=[[0., 0.], [15., 15 * math.tan(angle)]], speed=30/3.6, steering=0.,
               path_age=0., state_age=0., dt=.02)
    assert r['heading_correction_rad'] == pytest.approx(0.10)


def test_integral_holds_on_large_errors_low_speed_and_latched_braking():
    c = Stanley()
    straight(c, offset=0.3)
    assert c.integral_rad > 0
    held = c.integral_rad
    straight(c, offset=0.8)  # beyond integral_max_error_m
    straight(c, offset=0.3, speed=3.0)  # below integral_min_speed_mps
    assert c.integral_rad == pytest.approx(held)
    c.red.latched = True
    straight(c, offset=0.3)
    assert c.integral_rad == pytest.approx(held)
    c.reset_lateral()
    assert c.integral_rad == 0.0 and c.compensator.added_rad == 0.0


def run_plant(controller, seconds=30.0):
    """Straight run on the real car's steering: the wheel sits 0.32 deg left of the command and
    the car runs straight with the encoder 0.20 deg right (2026-10-25 after the wheel alignment)."""
    x = y = yaw = 0.
    speed = encoder = 0.
    lateral = []
    for _ in range(int(seconds / .02)):
        world = np.array([[x - 3, 0.], [x + 20, 0.]])
        cs, sn = math.cos(yaw), math.sin(yaw)
        path = (world - [x, y]) @ np.array([[cs, -sn], [sn, cs]])
        r = controller.step(points=path, speed=speed, steering=encoder, path_age=0., state_age=0., dt=.02)
        encoder += (r['steering_target_rad'] + math.radians(0.32) - encoder) * (1 - math.exp(-.02 / .1))
        speed = r['speed_target_mps']
        yaw += speed / controller.cfg.wheelbase_m * math.tan(encoder + math.radians(0.20)) * .02
        x += speed * math.cos(yaw) * .02
        y += speed * math.sin(yaw) * .02
        lateral.append(y)
    return np.array(lateral)


def test_compensation_removes_the_straight_offset_of_the_old_law():
    old = StanleyConfig(steering_neutral_offset_deg=0.0, steering_friction_comp_deg=[0.0, 0.0, 0.0],
                        integral_limit_deg=0.0)
    without = run_plant(Stanley(old))
    with_dev = run_plant(Stanley())
    # Without the offset the car holds the 0.52 deg the actuator needs by running off the path.
    assert abs(without[-250:].mean()) > 0.15
    assert abs(with_dev[-250:].mean()) < 0.03


def test_estop_use_leaves_the_red_line_to_the_person():
    braking = Stanley()
    estop = Stanley(StanleyConfig(vehicle_brake=False))
    gate = [[1.9, 2.5], [1.9, -2.5]]
    for c in (braking, estop):
        for _ in range(100):
            straight(c)
        straight(c, red_gate=gate, red_gate_stamp=1, red_gate_age=0.)
    before = estop.speed_command
    a = straight(braking, red_gate=gate, red_gate_stamp=2, red_gate_age=0.)
    b = straight(estop, red_gate=gate, red_gate_stamp=2, red_gate_age=0.)
    assert a['red_latched'] and a['brake_engage'] and a['speed_target_mps'] == 0
    # USE: the red line is reported, but speed, steering and the brake are left as they were.
    assert b['red_latched'] and b['reason'] == 'tracking' and not b['brake_engage']
    assert b['speed_target_mps'] >= before > 0
    for _ in range(10):
        b = straight(estop, offset=0.3, speed=0.2)
    assert b['tracking'] and not estop.red.steering_frozen


@pytest.mark.parametrize('kwargs', [dict(enabled=False), dict(estop=True), dict(path_age=1.0)])
def test_estop_use_never_engages_the_vehicle_brake(kwargs):
    braking, estop = Stanley(), Stanley(StanleyConfig(vehicle_brake=False))
    a, b = straight(braking, **kwargs), straight(estop, **kwargs)
    assert a['brake_engage'] and not b['brake_engage']
    assert a['speed_target_mps'] == b['speed_target_mps'] == 0
