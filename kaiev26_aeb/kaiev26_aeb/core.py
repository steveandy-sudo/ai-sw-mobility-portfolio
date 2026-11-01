"""ROS-independent cone-boundary and center-path geometry."""
from dataclasses import dataclass
from itertools import combinations
import math

import numpy as np


@dataclass
class Config:
    roi_x_min: float = 0.5
    roi_x_max: float = 25.0
    roi_y_abs: float = 7.0
    boundary_min_cones: int = 2
    boundary_min_span: float = 2.5
    boundary_residual: float = 0.3
    boundary_max_heading_deg: float = 30.0
    boundary_max_difference_deg: float = 10.0
    corridor_width_min: float = 3.5
    corridor_width_max: float = 6.5
    path_max_forward: float = 20.0
    path_min_forward: float = 4.0
    path_step: float = 0.25
    red_gate_max_stagger_m: float = 2.0

    def __post_init__(self):
        if min(self.path_step, self.boundary_min_span, self.red_gate_max_stagger_m) <= 0:
            raise ValueError('Path spacing and minimum span must be positive')
        if self.boundary_min_cones < 2:
            raise ValueError('At least two cones per boundary are required')
        if not (0 < self.corridor_width_min < self.corridor_width_max):
            raise ValueError('Invalid corridor width limits')
        if not (0 < self.path_min_forward <= self.path_max_forward <= self.roi_x_max):
            raise ValueError('Path lengths must be positive and inside the LiDAR ROI')

def fit_line(points, cfg):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) < cfg.boundary_min_cones:
        return None
    best = None
    for i, j in combinations(range(len(points)), 2):
        dx = points[j, 0] - points[i, 0]
        if abs(dx) < cfg.boundary_min_span:
            continue
        m = (points[j, 1] - points[i, 1]) / dx
        b = points[i, 1] - m * points[i, 0]
        if abs(math.degrees(math.atan(m))) > cfg.boundary_max_heading_deg:
            continue
        mask = abs(points[:, 1] - (m * points[:, 0] + b)) / math.sqrt(1 + m * m) < cfg.boundary_residual
        if mask.sum() < cfg.boundary_min_cones or np.ptp(points[mask, 0]) < cfg.boundary_min_span:
            continue
        score = (int(mask.sum()), float(np.ptp(points[mask, 0])))
        if best is None or score > best[0]:
            best = score, mask
    if best is None:
        return None
    mask = best[1]
    m, b = np.linalg.lstsq(np.column_stack((points[mask, 0], np.ones(mask.sum()))), points[mask, 1], rcond=None)[0]
    if abs(math.degrees(math.atan(m))) > cfg.boundary_max_heading_deg:
        return None
    return {'m': float(m), 'b': float(b), 'inliers': np.flatnonzero(mask).tolist(),
            'x_min': float(points[mask, 0].min()), 'x_max': float(points[mask, 0].max()),
            'count': int(mask.sum())}


def red_boundary_pair(sides, cfg):
    """Fit two observed rows jointly; red membership follows line distance, not Y sign.

    Blue can support only left, yellow only right, red either (never both).
    Ordering the lines and checking width also works for a red-only corridor.
    """
    candidates = {}
    for side, cones in sides.items():
        points = np.array([c['xyz'][:2] for c in cones]).reshape(-1, 2)
        lines = {}
        for i, j in combinations(range(len(points)), 2):
            dx = points[j, 0] - points[i, 0]
            if abs(dx) < cfg.boundary_min_span:
                continue
            m = (points[j, 1] - points[i, 1]) / dx
            if abs(math.degrees(math.atan(m))) > cfg.boundary_max_heading_deg:
                continue
            b = points[i, 1] - m * points[i, 0]
            ids = np.flatnonzero(abs(points[:, 1] - m * points[:, 0] - b)
                                 / math.hypot(1, m) < cfg.boundary_residual)
            key = tuple(ids)
            if key in lines:
                continue
            line = fit_line(points[ids], cfg)
            if line:
                line['inliers'] = ids[line['inliers']].tolist()
                lines[key] = line
        candidates[side] = list(lines.values())
    best = None
    for left in candidates['left']:
        for right in candidates['right']:
            if abs(math.degrees(math.atan(left['m']) - math.atan(right['m']))) > cfg.boundary_max_difference_deg:
                continue
            mean_m = (left['m'] + right['m']) / 2
            xmax = min(left['x_max'], right['x_max'], cfg.path_max_forward)
            widths = np.array([left['b'] - right['b'],
                (left['m'] - right['m']) * xmax + left['b'] - right['b']]) / math.hypot(1, mean_m)
            if widths.min() < cfg.corridor_width_min or widths.max() > cfg.corridor_width_max:
                continue
            used = {side: [sides[side][i] for i in line['inliers']]
                    for side, line in [('left', left), ('right', right)]}
            if {id(c) for c in used['left']} & {id(c) for c in used['right']}:
                continue
            score = (left['count'] + right['count'],
                     sum(c['color'] != 'red' for row in used.values() for c in row),
                     min(left['x_max'] - left['x_min'], right['x_max'] - right['x_min']))
            if best is None or score > best[0]:
                best = score, left, right
    return (best[1], best[2]) if best else (None, None)


def red_entry_gate(cones, heading, cfg):
    """Nearest red inlier on each boundary; a single red side cannot define entry."""
    forward = np.array([math.cos(heading), math.sin(heading)])
    reds = {side: [c for c in cones if c['color'] == 'red' and c['side'] == side]
            for side in ('left', 'right')}
    if not all(reds.values()):
        return []
    gate = [min(reds[side], key=lambda c: c['xyz'][:2] @ forward)['xyz'][:2]
            for side in ('left', 'right')]
    if abs((gate[0] - gate[1]) @ forward) > cfg.red_gate_max_stagger_m:
        return []
    return np.asarray(gate).tolist()


def plan_corridor(cones, cfg):
    result = {'valid': False, 'reason': 'insufficient_boundaries', 'path': np.empty((0, 3)),
              'left': None, 'right': None, 'red_gate': []}
    sides = {'left': [], 'right': []}
    for cone in cones:
        cone['side'] = 'unassigned'
        x, y = cone['xyz'][:2]
        if not cfg.roi_x_min < x < cfg.roi_x_max:
            continue
        # Color labels describe the course boundary, not the sign of vehicle Y.
        # A yawed car can see far left-boundary cones on its right (and vice versa).
        if cone['color'] == 'blue':
            sides['left'].append(cone)
        elif cone['color'] == 'yellow':
            sides['right'].append(cone)
        elif cone['color'] == 'red':
            sides['left'].append(cone)
            sides['right'].append(cone)
    if any(c['color'] == 'red' for c in sides['left']):
        lines = red_boundary_pair(sides, cfg)
    else:
        lines = [fit_line([c['xyz'][:2] for c in selected], cfg) for selected in sides.values()]
    for (side, selected), line in zip(sides.items(), lines):
        result[side] = line
        if line:
            for i in line['inliers']:
                selected[i]['side'] = side
    left, right = result['left'], result['right']
    if left is None or right is None:
        return result
    if abs(math.degrees(math.atan(left['m']) - math.atan(right['m']))) > cfg.boundary_max_difference_deg:
        result['reason'] = 'boundaries_diverge'
        return result
    xmax = min(left['x_max'], right['x_max'], cfg.path_max_forward)
    if xmax < cfg.path_min_forward:
        result['reason'] = 'short_observation'
        return result
    # Evaluate both fitted boundaries at the SAME forward coordinate. Irregular
    # longitudinal cone spacing never requires pairing the ith left/right cones.
    m, b = (left['m'] + right['m']) / 2, (left['b'] + right['b']) / 2
    widths = np.array([left['b'] - right['b'],
                       (left['m'] - right['m']) * xmax + left['b'] - right['b']]) / math.sqrt(1 + m * m)
    if widths.min() < cfg.corridor_width_min or widths.max() > cfg.corridor_width_max:
        result['reason'] = 'invalid_width'
        return result
    x = np.arange(0.0, xmax + 1e-6, cfg.path_step)
    z = np.zeros_like(x)
    result.update(valid=True, reason='ok', path=np.column_stack((x, m * x + b, z)),
                  width_min=float(widths.min()), width_max=float(widths.max()), heading=math.atan(m))
    result['red_gate'] = red_entry_gate(cones, result['heading'], cfg)
    return result
