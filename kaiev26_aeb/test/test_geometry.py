import numpy as np
import pytest

from kaiev26_aeb.core import (Config, plan_corridor, ground_plane, cluster_cones,
                              associate_camera, resolve_colors, project, fuse_and_plan)
from kaiev26_aeb.messages import cloud_xyz
from sensor_msgs.msg import PointCloud2, PointField


def cone(x, y, color):
    return {'xyz': np.array([x, y, .1]), 'color': color, 'side': 'unassigned'}


def test_staggered_cones_produce_center_at_equal_x():
    # Deliberately unequal cone counts/spacing; midpoint of paired indices is wrong.
    left = [cone(x, .03*x + 2.7, 'blue') for x in [2, 5, 12, 18]]
    right = [cone(x, .03*x - 2.3, 'yellow') for x in [3, 10, 16]]
    result = plan_corridor(left + right, Config())
    assert result['valid']
    np.testing.assert_allclose(result['path'][:, 1], .03*result['path'][:, 0] + .2, atol=1e-10)
    assert result['path'][-1, 0] <= 16


@pytest.mark.parametrize('yaw_deg', [-15, 15])
def test_colored_boundaries_survive_vehicle_heading_error(yaw_deg):
    # The car is centered but rotated. Far cones cross its forward axis even
    # though their blue/left and yellow/right course labels have not changed.
    yaw = np.deg2rad(yaw_deg)
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)]])
    cones = []
    for color, lateral in [('blue', 2.5), ('yellow', -2.5)]:
        for distance in [4, 7, 10, 13, 16]:
            x, y = np.array([distance, lateral]) @ rotation
            cones.append(cone(x, y, color))
    result = plan_corridor(cones, Config())
    assert result['valid']
    assert all(c['side'] == ('left' if c['color'] == 'blue' else 'right') for c in cones)
    np.testing.assert_allclose(result['path'][:, 1],
                               -np.tan(yaw)*result['path'][:, 0], atol=1e-10)
    from kaiev26_aeb.pursuit import Pursuit
    command = Pursuit().step(result['path'][:, :2], speed=0, steering=0,
                             path_age=0, state_age=0, dt=.02)
    assert command['tracking']
    assert command['lookahead_m'] == 7
    assert command['steering_target_rad']*yaw_deg < 0


def test_red_section_continues_both_boundaries():
    cones = [cone(x, y, 'red') for x in [3, 7, 13] for y in [-2.5, 2.5]]
    result = plan_corridor(cones, Config())
    assert result['valid']
    assert all(c['color'] == 'red' for c in cones)
    assert {c['side'] for c in cones} == {'left', 'right'}
    np.testing.assert_allclose(result['path'][:, 1], 0, atol=1e-10)


@pytest.mark.parametrize('yaw_deg', [-25, -15, 15, 25])
@pytest.mark.parametrize('mixed', [False, True])
def test_red_rows_and_entry_are_independent_of_vehicle_y_sign(yaw_deg, mixed):
    angle = np.deg2rad(yaw_deg)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    cones, expected = [], []
    for side, lateral, color in [('left', 2.5, 'blue'), ('right', -2.5, 'yellow')]:
        for distance in [4, 7, 10, 13, 16, 19]:
            xy = np.array([distance, lateral]) @ rotation
            cones.append(cone(*xy, color if mixed and distance < 10 else 'red'))
            expected.append(side)
    result = plan_corridor(cones, Config())
    assert result['valid']
    assert [c['side'] for c in cones] == expected
    assert result['path'][-1, 0] > 7
    np.testing.assert_allclose(result['path'][:, 1], -np.tan(angle)*result['path'][:, 0], atol=1e-10)
    distance = 10 if mixed else 4
    np.testing.assert_allclose(np.array(result['red_gate']) @ rotation.T,
                               [[distance, 2.5], [distance, -2.5]], atol=1e-10)


def test_single_red_side_or_center_outlier_does_not_define_entry():
    cones = [cone(x, y, color) for x in [3, 6, 9]
             for y, color in [(2.5, 'blue'), (-2.5, 'yellow')]]
    cones += [cone(x, 2.5, 'red') for x in [12, 15]]
    cones.append(cone(10, 0, 'red'))
    result = plan_corridor(cones, Config())
    assert result['valid'] and result['red_gate'] == []
    assert cones[-1]['side'] == 'unassigned'


def test_one_red_row_cannot_be_used_as_both_boundaries():
    result = plan_corridor([cone(x, 2.5, 'red') for x in [3, 6, 9, 12]], Config())
    assert not result['valid'] and not result['red_gate']


def test_missing_first_red_on_one_side_does_not_pair_with_next_row():
    cones = [cone(x, y, color) for x in [3, 6, 9]
             for y, color in [(2.5, 'blue'), (-2.5, 'yellow')]]
    cones += [cone(x, 2.5, 'red') for x in [12, 15, 18]]
    cones += [cone(x, -2.5, 'red') for x in [15, 18]]
    result = plan_corridor(cones, Config())
    assert result['valid'] and result['red_gate'] == []


def test_no_invented_opposite_boundary_or_unknown_color():
    cones = [cone(x, 2.5, 'blue') for x in [3, 7, 13]]
    cones += [cone(x, -2.5, 'unknown') for x in [3, 7, 13]]
    result = plan_corridor(cones, Config())
    assert not result['valid']
    assert not len(result['path'])


def test_outlier_and_invalid_corridor():
    cones = [cone(x, y, color) for x in [3, 7, 13] for y, color in [(2.5, 'blue'), (-2.5, 'yellow')]]
    cones.append(cone(9, 5, 'blue'))
    assert plan_corridor(cones, Config())['valid']
    cones = [cone(x, y, color) for x in [3, 7, 13] for y, color in [(5, 'blue'), (-5, 'yellow')]]
    assert plan_corridor(cones, Config())['reason'] == 'invalid_width'


def test_ground_cluster_and_tall_object_veto():
    x, y = np.meshgrid(np.linspace(1, 22, 50), np.linspace(-5, 5, 40))
    ground = np.column_stack((x.ravel(), y.ravel(), -.24 + .01*x.ravel()))
    good = np.array([[6+dx, 2.5+dy, -.18+h] for dx, dy, h in [(-.05, 0, .16), (0, .03, .3), (.03, -.02, .55)]])
    leg = good + [2, -5, 0]
    body = np.array([[8, -2.5, 1.3], [8.05, -2.5, 1.4]])
    xyz = np.vstack((ground, good, leg, body))
    cfg = Config()
    plane = ground_plane(xyz, cfg)
    np.testing.assert_allclose(plane, [.01, 0, -.24], atol=.01)
    cones = cluster_cones(xyz, plane, cfg)
    assert len(cones) == 1
    np.testing.assert_allclose(cones[0]['xyz'][:2], [6, 2.5], atol=.06)


def test_association_uses_calibration_and_rejects_color_conflict():
    xyz = np.array([[6, 2.5, .1], [6.03, 2.5, .3], [5.98, 2.51, .5]])
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [0, 0, -1], [1, 0, 0]]
    transform[:3, 3] = [0, 1.2, 0]
    info = {'k': [500, 0, 400, 0, 500, 300, 0, 0, 1], 'd': [0]*5,
            'width': 800, 'height': 600, 'distortion_model': 'plumb_bob'}
    uv, valid = project(xyz, transform, info)
    assert valid.all()
    bounds = [uv[:, 0].min()-5, uv[:, 1].min()-5, uv[:, 0].max()+5, uv[:, 1].max()+5]
    cones = [{'indices': np.arange(3), 'matches': [], 'color': 'unknown', 'score': 0.0}]
    associate_camera(cones, xyz, [{'xyxy': bounds, 'score': .95, 'color': 'blue'}], transform, info, 'left')
    resolve_colors(cones)
    assert cones[0]['color'] == 'blue'
    associate_camera(cones, xyz, [{'xyxy': bounds, 'score': .95, 'color': 'red'}], transform, info, 'right')
    resolve_colors(cones)
    assert cones[0]['color'] == 'unknown'


def test_pointcloud_organized_padding_and_big_endian():
    msg = PointCloud2(height=2, width=2, point_step=16, row_step=40, is_bigendian=True)
    msg.fields = [PointField(name=n, offset=i*4, datatype=7, count=1) for i, n in enumerate('xyz')]
    buf = bytearray(80)
    points = np.ndarray((2, 2, 4), dtype='>f4', buffer=buf, strides=(40, 16, 4))
    points[:, :, :3] = np.arange(12).reshape(2, 2, 3)
    msg.data = bytes(buf)
    np.testing.assert_allclose(cloud_xyz(msg), np.arange(12).reshape(4, 3))


def test_empty_lidar_clears_path_without_projection_crash():
    info = {'k': [500, 0, 400, 0, 500, 300, 0, 0, 1], 'd': [0]*5,
            'width': 800, 'height': 600, 'distortion_model': 'plumb_bob'}
    cameras = {s: {'info': info, 'transform': np.eye(4), 'detections': []} for s in ['left', 'right']}
    result = fuse_and_plan(np.empty((0, 3)), cameras, Config())
    assert result['plan']['reason'] == 'ground_not_found'
    assert not result['plan']['valid'] and not len(result['plan']['path'])
