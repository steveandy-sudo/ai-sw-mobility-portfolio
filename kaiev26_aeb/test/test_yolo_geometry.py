import numpy as np

from kaiev26_aeb.core import Config
from kaiev26_aeb.yolo_core import (bbox_ground_point, merge_camera_cones,
                                   yolo_only_plan)


INFO = {
    'width': 200, 'height': 200,
    'k': [100., 0., 50., 0., 100., 50., 0., 0., 1.],
    'd': [0., 0., 0., 0., 0.], 'distortion_model': 'plumb_bob',
    'frame': 'camera',
}
BASE_FROM_CAMERA = np.array([
    [0., 0., 1., 0.],
    [-1., 0., 0., 0.],
    [0., -1., 0., 1.],
    [0., 0., 0., 1.],
])


def box_for(x, y, color):
    u = 50. - y * 100. / x
    bottom = 50. + 100. / x
    return {'xyxy': [u - 3., bottom - 12., u + 3., bottom],
            'color': color, 'score': .9}


def test_bbox_bottom_ray_intersects_ground():
    point = bbox_ground_point(box_for(10., 2., 'blue'), INFO,
                              BASE_FROM_CAMERA, 0.)
    np.testing.assert_allclose(point, [10., 2., 0.], atol=1e-9)


def test_yolo_boxes_generate_same_corridor_contract_and_red_gate():
    left = [box_for(x, 2., 'blue') for x in (5., 8.)]
    right = [box_for(x, -2., 'yellow') for x in (5., 8.)]
    left.append(box_for(11., 2., 'red'))
    right.append(box_for(11., -2., 'red'))
    cameras = {
        'left': {'detections': left, 'info': INFO,
                 'transform': BASE_FROM_CAMERA},
        'right': {'detections': right, 'info': INFO,
                  'transform': BASE_FROM_CAMERA},
    }
    cfg = Config(ground_z=0., roi_x_max=15., path_max_forward=15.)
    result = yolo_only_plan(cameras, cfg, bottom_margin_px=0.)
    assert result['plan']['valid']
    np.testing.assert_allclose(result['plan']['path'][:, 1], 0., atol=1e-9)
    np.testing.assert_allclose(result['plan']['red_gate'], [[11., 2.], [11., -2.]],
                               atol=1e-9)


def test_duplicate_same_color_cone_is_merged():
    def cone(x, y, score, camera):
        return {'xyz': np.array([x, y, 0.]), 'indices': np.empty(0, int),
                'height': 20., 'color': 'red', 'score': score,
                'matches': [{'camera': camera, 'box': 0, 'color': 'red',
                             'score': score}], 'side': 'unassigned'}
    merged = merge_camera_cones([
        cone(8., 2., .9, 'left'), cone(8.2, 2.1, .8, 'right')])
    assert len(merged) == 1
    assert {row['camera'] for row in merged[0]['matches']} == {'left', 'right'}
