import math

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from kaiev26_aeb.messages import header, predicted_path_marker


@pytest.mark.parametrize('steering', [0.0, 0.2, -0.2])
def test_prediction_matches_independent_bicycle_integration(steering):
    # A deliberately different raw angle ensures the picture uses the limited command.
    result = dict(tracking=True, brake_engage=False, lookahead_m=7.0,
                  steering_target_rad=steering, raw_steering_rad=0.4)
    marker = predicted_path_marker(result, 1.3, header(123_000_000_000, 'base_link'))
    assert marker.action == Marker.ADD
    assert marker.header.frame_id == 'base_link'
    assert marker.header.stamp.sec == 123
    assert marker.points[0].x == marker.points[0].y == 0.0
    x = y = yaw = 0.0
    ds = .001
    for _ in range(7000):
        turn = math.tan(steering) / 1.3 * ds
        x += ds * math.cos(yaw + turn / 2)
        y += ds * math.sin(yaw + turn / 2)
        yaw += turn
    assert [marker.points[-1].x, marker.points[-1].y] == pytest.approx([x, y], abs=1e-6)
    points = np.array([[p.x, p.y] for p in marker.points])
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).sum() == pytest.approx(7.0, abs=.001)


@pytest.mark.parametrize('tracking,brake', [(False, True), (True, True), (False, False)])
def test_inactive_or_braking_deletes_previous_prediction(tracking, brake):
    marker = predicted_path_marker(dict(tracking=tracking, brake_engage=brake),
                                   1.3, header(0, 'base_link'))
    assert marker.action == Marker.DELETE
    assert not marker.points
    assert marker.lifetime.nanosec == 500_000_000
