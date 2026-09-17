"""ROS-independent, frame-by-frame geometry shared by live and bag processing.

All geometry uses x forward, y left, z up in output_frame. Only camera-matched
LiDAR clusters enter either boundary. No guessed opposite boundary or tracking.
"""
from dataclasses import dataclass
from itertools import combinations
import math

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree


@dataclass
class Config:
    roi_x_min: float = 0.5
    roi_x_max: float = 25.0
    roi_y_abs: float = 7.0
    ground_z: float = -0.2408
    ground_search_half_height: float = 0.6
    ground_threshold: float = 0.08
    ground_min_fraction: float = 0.45
    cluster_radius: float = 0.25
    cluster_min_points: int = 3
    cone_min_height: float = 0.22
    cone_max_height: float = 0.95
    cone_max_width: float = 0.7
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
        if min(self.path_step, self.cluster_radius, self.boundary_min_span, self.red_gate_max_stagger_m) <= 0:
            raise ValueError('Path spacing, cluster radius and minimum span must be positive')
        if self.boundary_min_cones < 2 or self.cluster_min_points < 2:
            raise ValueError('At least two cones per boundary and two points per cluster are required')
        if not (0 < self.corridor_width_min < self.corridor_width_max):
            raise ValueError('Invalid corridor width limits')
        if not (0 < self.path_min_forward <= self.path_max_forward <= self.roi_x_max):
            raise ValueError('Path lengths must be positive and inside the LiDAR ROI')


def transform_points(points, transform):
    return np.asarray(points) @ transform[:3, :3].T + transform[:3, 3]


def ground_plane(xyz, cfg):
    p = xyz[(xyz[:, 0] > 1.0) & (xyz[:, 0] < cfg.roi_x_max)
            & (abs(xyz[:, 1]) < cfg.roi_y_abs)
            & (abs(xyz[:, 2] - cfg.ground_z) < cfg.ground_search_half_height)]
    if len(p) < 60:
        return None
    rng = np.random.default_rng(21)
    if len(p) > 1800:
        p = p[rng.choice(len(p), 1800, replace=False)]
    a = np.column_stack((p[:, :2], np.ones(len(p))))
    best = np.zeros(len(p), dtype=bool)
    for _ in range(80):
        ids = rng.choice(len(p), 3, replace=False)
        try:
            plane = np.linalg.solve(a[ids], p[ids, 2])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(plane[:2]) > 0.25:
            continue
        mask = abs(a @ plane - p[:, 2]) < cfg.ground_threshold
        if mask.sum() > best.sum():
            best = mask
    if best.mean() < cfg.ground_min_fraction:
        return None
    return np.linalg.lstsq(a[best], p[best, 2], rcond=None)[0]


def cluster_cones(xyz, plane, cfg):
    height = xyz[:, 2] - (xyz[:, :2] @ plane[:2] + plane[2])
    roi = ((xyz[:, 0] > cfg.roi_x_min) & (xyz[:, 0] < cfg.roi_x_max)
           & (abs(xyz[:, 1]) < cfg.roi_y_abs))
    indices = np.flatnonzero(roi & (height > 0.12) & (height < cfg.cone_max_height))
    if not len(indices):
        return []
    points = xyz[indices]
    # Two-dimensional connected components keep a cone's vertical returns together.
    pairs = cKDTree(points[:, :2]).query_pairs(cfg.cluster_radius, output_type='ndarray')
    parent = np.arange(len(points))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in pairs:
        a, b = root(a), root(b)
        if a != b:
            parent[b] = a
    groups = {}
    for i in range(len(points)):
        groups.setdefault(root(i), []).append(i)
    tall = xyz[roi & (height > 1.0), :2]
    tall_tree = cKDTree(tall) if len(tall) else None
    cones = []
    for group in groups.values():
        if len(group) < cfg.cluster_min_points:
            continue
        ids = indices[group]
        cloud = xyz[ids]
        center = np.median(cloud, axis=0)
        peak = height[ids].max()
        if np.ptp(cloud[:, :2], axis=0).max() > cfg.cone_max_width:
            continue
        if not cfg.cone_min_height <= peak <= cfg.cone_max_height:
            continue
        if np.linalg.norm(center[:2]) < 8.0 and height[ids].min() > 0.32:
            continue
        if tall_tree is not None and len(tall_tree.query_ball_point(center[:2], 0.32)) >= 2:
            continue
        cones.append({'xyz': center, 'indices': ids, 'height': float(peak),
                      'color': 'unknown', 'score': 0.0, 'matches': [], 'side': 'unassigned'})
    return sorted(cones, key=lambda c: c['xyz'][0])


def project(xyz, camera_from_base, info):
    if len(xyz) == 0:
        return np.empty((0, 2)), np.empty(0, dtype=bool)
    optical = transform_points(xyz, camera_from_base)
    valid = ((optical[:, 2] > 0.1)
             & (abs(optical[:, 0]) < optical[:, 2] * 1.2)
             & (abs(optical[:, 1]) < optical[:, 2]))
    k = np.asarray(info['k'], dtype=float).reshape(3, 3)
    d = np.asarray(info['d'], dtype=float)
    model = info.get('distortion_model', 'plumb_bob')
    if model in ('plumb_bob', 'rational_polynomial', ''):
        uv = cv2.projectPoints(optical, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
    elif model == 'equidistant':
        uv = cv2.fisheye.projectPoints(optical.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
    else:
        raise ValueError(f'Unsupported camera distortion model: {model}')
    valid &= (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < info['width'])
              & (uv[:, 1] >= 0) & (uv[:, 1] < info['height']))
    return uv, valid


def associate_camera(cones, xyz, detections, transform, info, camera_name):
    """One-to-one 2D box / 3D cluster association for a single camera."""
    uv, visible = project(xyz, transform, info)
    if not cones or not detections:
        return uv, visible
    costs = np.full((len(cones), len(detections)), 1e6)
    for i, cone in enumerate(cones):
        pixels = uv[cone['indices']][visible[cone['indices']]]
        if len(pixels) < 2:
            continue
        center = np.median(pixels, axis=0)
        for j, box in enumerate(detections):
            x1, y1, x2, y2 = box['xyxy']
            w, h = max(x2 - x1, 1), max(y2 - y1, 1)
            margin = max(2.0, 0.04 * w)
            inside = ((pixels[:, 0] >= x1 - margin) & (pixels[:, 0] <= x2 + margin)
                      & (pixels[:, 1] >= y1 - margin) & (pixels[:, 1] <= y2 + margin))
            distance = np.linalg.norm((center - [(x1 + x2) / 2, (y1 + y2) / 2]) / [w, h])
            if inside.sum() >= 2 and inside.mean() >= 0.5 and distance <= 0.85:
                costs[i, j] = 0.55 * (1 - inside.mean()) + 0.35 * distance + 0.1 * (1 - box['score'])
    for i, j in zip(*linear_sum_assignment(costs)):
        if costs[i, j] >= 1e5:
            continue
        if any(detections[k]['color'] != detections[j]['color']
               and costs[i, k] < costs[i, j] + 0.08 for k in range(len(detections)) if k != j):
            continue
        box = detections[j]
        cones[i]['matches'].append({'camera': camera_name, 'box': int(j),
                                    'color': box['color'], 'score': box['score']})
    return uv, visible


def resolve_colors(cones):
    for cone in cones:
        cone['color'], cone['score'] = 'unknown', 0.0
        colors = {m['color'] for m in cone['matches']}
        # Conflicting left/right image classifications remain unknown.
        if len(colors) == 1:
            cone['color'] = next(iter(colors))
            cone['score'] = max(m['score'] for m in cone['matches'])


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


def fuse_and_plan(xyz, cameras, cfg):
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xyz = xyz[(xyz[:, 0] > cfg.roi_x_min) & (xyz[:, 0] < cfg.roi_x_max)
              & (abs(xyz[:, 1]) < cfg.roi_y_abs) & (xyz[:, 2] > -2.0) & (xyz[:, 2] < 3.0)]
    plane = ground_plane(xyz, cfg)
    cones = [] if plane is None else cluster_cones(xyz, plane, cfg)
    projections = {}
    for name, camera in cameras.items():
        projections[name] = associate_camera(cones, xyz, camera['detections'],
                                             camera['transform'], camera['info'], name)
    resolve_colors(cones)
    result = plan_corridor(cones, cfg)
    if plane is None:
        result['reason'] = 'ground_not_found'
    return {'xyz': xyz, 'cones': cones, 'plan': result, 'plane': plane, 'projections': projections}
