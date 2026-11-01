"""Build a cone corridor from independently fitted boundary rows."""

import math

import numpy as np

from .core import plan_corridor


COLOR_CLASSES = {
    'blue cone': 'blue',
    'yellow cone': 'yellow',
    'red cone': 'red',
}


def plan_colored_corridor(cones, cfg):
    """Keep each observed boundary fit and use their midpoint as the path."""
    plan = plan_corridor(cones, cfg)
    if not plan['valid']:
        return plan

    rows = {side: [cone for cone in cones if cone['side'] == side]
            for side in ('left', 'right')}
    normal = np.abs(np.array([-math.sin(plan['heading']), math.cos(plan['heading'])]))
    if all('size_xy' in cone for items in rows.values() for cone in items):
        extents = [np.median([normal @ cone['size_xy'] for cone in rows[side]])
                   for side in ('left', 'right')]
        plan['clear_width_est_m'] = float(plan['width_min'] - (extents[0] + extents[1]) / 2)
    return plan
