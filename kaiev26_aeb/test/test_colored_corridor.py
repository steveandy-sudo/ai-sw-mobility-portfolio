import numpy as np
import pytest

from kaiev26_aeb.colored_corridor import plan_colored_corridor
from kaiev26_aeb.core import Config
from kaiev26_aeb.messages import output_messages


def cone(x, y, color):
    return dict(xyz=np.array([x, y, 0.0]), color=color, score=0.9,
                side='unassigned', indices=np.empty(0, dtype=int), matches=[],
                size_xy=np.array([0.25, 0.25]))


def config():
    return Config(corridor_width_min=1.0)


def test_blue_yellow_and_red_form_center_path_from_independent_boundaries():
    cones = []
    for x in (4.0, 7.0, 10.0):
            cones.extend((cone(x, 1.8 + 0.02 * x, 'blue'),
                          cone(x + 0.4, -1.8 + 0.03 * (x + 0.4), 'yellow')))
    for x in (13.0, 16.0):
        cones.extend((cone(x, 1.8 + 0.02 * x, 'red'),
                      cone(x, -1.8 + 0.03 * x, 'red')))
    plan = plan_colored_corridor(cones, config())
    assert plan['valid'], plan['reason']
    assert plan['left']['m'] == pytest.approx(0.02)
    assert plan['right']['m'] == pytest.approx(0.03)
    assert plan['path'][0, 1] == pytest.approx(
        (plan['left']['b'] + plan['right']['b']) / 2)
    assert len(plan['red_gate']) == 2
    assert plan['red_gate'][0][0] == pytest.approx(13.0)
    assert plan['clear_width_est_m'] < plan['width_min']
    messages = output_messages(dict(cones=cones, plane=None, plan=plan),
                               1_000_000_000, 'base_link')
    assert len(messages['/aeb/center_path'].poses) > 2
    assert len(messages['/aeb/red_gate'].poses) == 2


def test_one_boundary_slope_does_not_rotate_the_other():
    left = [cone(x, 2.0 + 0.02 * x, 'blue') for x in (4.0, 8.0, 12.0)]
    right_a = [cone(x, -2.0 + 0.04 * x, 'yellow') for x in (4.0, 8.0, 12.0)]
    right_b = [cone(x, -2.0 + 0.08 * x, 'yellow') for x in (4.0, 8.0, 12.0)]
    plans = [plan_colored_corridor(left + right, config())
             for right in (right_a, right_b)]
    assert all(plan['valid'] for plan in plans)
    for plan in plans:
        assert plan['left']['m'] == pytest.approx(0.02)
        x = plan['path'][-1, 0]
        assert plan['path'][-1, 1] == pytest.approx(
            ((plan['left']['m'] * x + plan['left']['b']) +
             (plan['right']['m'] * x + plan['right']['b'])) / 2)
    assert plans[0]['right']['m'] == pytest.approx(0.04)
    assert plans[1]['right']['m'] == pytest.approx(0.08)


def test_large_boundary_angle_disagreement_is_rejected():
    cones = [cone(x, 2.0, 'blue') for x in (4.0, 8.0, 12.0)]
    cones += [cone(x, -4.0 + 0.2 * x, 'yellow') for x in (4.0, 8.0, 12.0)]
    plan = plan_colored_corridor(cones, config())
    assert not plan['valid']
    assert plan['reason'] == 'boundaries_diverge'


def test_red_only_rows_do_not_need_blue_or_yellow_history():
    cones = [cone(x, y, 'red') for x in (4.0, 7.0, 10.0, 13.0)
             for y in (2.0, -2.0)]
    plan = plan_colored_corridor(cones, config())
    assert plan['valid'], plan['reason']
    assert plan['width_min'] == pytest.approx(4.0)
    np.testing.assert_allclose(plan['red_gate'], [[4.0, 2.0], [4.0, -2.0]])


def test_single_boundary_is_not_invented():
    cones = [cone(x, 1.8, 'blue') for x in (4.0, 7.0, 10.0)]
    plan = plan_colored_corridor(cones, config())
    assert not plan['valid']
    assert len(plan['path']) == 0
