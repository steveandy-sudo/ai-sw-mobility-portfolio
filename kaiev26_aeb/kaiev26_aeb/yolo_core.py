"""Camera-only cone geometry shared by the live node and bag processor."""

import cv2
import numpy as np

from .core import plan_corridor


def _undistort(pixel, info):
    k = np.asarray(info['k'], dtype=float).reshape(3, 3)
    d = np.asarray(info['d'], dtype=float)
    source = np.asarray(pixel, dtype=float).reshape(1, 1, 2)
    model = info.get('distortion_model', 'plumb_bob')
    if model in ('plumb_bob', 'rational_polynomial', ''):
        return cv2.undistortPoints(source, k, d).reshape(2)
    if model == 'equidistant':
        return cv2.fisheye.undistortPoints(source, k, d).reshape(2)
    raise ValueError(f'Unsupported camera distortion model: {model}')


def bbox_ground_point(box, info, base_from_camera, ground_z, bottom_offset_px=0.0):
    """Intersect the ray through a bbox bottom center with the base-frame ground."""
    x1, y1, x2, y2 = map(float, box['xyxy'])
    if not np.isfinite([x1, y1, x2, y2]).all() or x2 <= x1 or y2 <= y1:
        return None
    normalized = _undistort(((x1 + x2) / 2, y2 - bottom_offset_px), info)
    ray_camera = np.array([normalized[0], normalized[1], 1.0])
    transform = np.asarray(base_from_camera, dtype=float).reshape(4, 4)
    origin = transform[:3, 3]
    direction = transform[:3, :3] @ ray_camera
    if not np.isfinite(direction).all() or abs(direction[2]) < 1e-6:
        return None
    distance = (float(ground_z) - origin[2]) / direction[2]
    if distance <= 0:
        return None
    point = origin + distance * direction
    return point if np.isfinite(point).all() else None


def camera_cones(detections, camera_name, info, base_from_camera, cfg,
                 min_bbox_height_px=8.0, min_bbox_width_px=3.0,
                 bottom_margin_px=2.0, bottom_offset_px=0.0):
    cones = []
    for box_index, box in enumerate(detections):
        x1, y1, x2, y2 = map(float, box['xyxy'])
        width, height = x2 - x1, y2 - y1
        if (width < min_bbox_width_px or height < min_bbox_height_px
                or y2 >= float(info['height']) - bottom_margin_px):
            continue
        point = bbox_ground_point(box, info, base_from_camera, cfg.ground_z,
                                  bottom_offset_px)
        if point is None or not (cfg.roi_x_min < point[0] < cfg.roi_x_max
                                 and abs(point[1]) < cfg.roi_y_abs):
            continue
        cones.append({
            'xyz': point,
            'indices': np.empty(0, dtype=int),
            'height': float(height),
            'color': box['color'],
            'score': float(box['score']),
            'matches': [{'camera': camera_name, 'box': int(box_index),
                         'color': box['color'], 'score': float(box['score'])}],
            'side': 'unassigned',
        })
    return cones


def merge_camera_cones(cones, radius_m=0.75):
    """Merge the same physical cone when it appears in both camera images."""
    merged = []
    for cone in sorted(cones, key=lambda item: -item['score']):
        match = next((item for item in merged
                      if item['color'] == cone['color']
                      and np.linalg.norm(item['xyz'][:2] - cone['xyz'][:2]) < radius_m), None)
        if match is None:
            merged.append(cone)
            continue
        old_score, new_score = match['score'], cone['score']
        match['xyz'] = ((match['xyz'] * old_score + cone['xyz'] * new_score)
                        / (old_score + new_score))
        match['score'] = max(old_score, new_score)
        match['matches'].extend(cone['matches'])
    return sorted(merged, key=lambda item: item['xyz'][0])


def yolo_only_plan(cameras, cfg, min_bbox_height_px=8.0,
                   min_bbox_width_px=3.0, bottom_margin_px=2.0,
                   bottom_offset_px=0.0, duplicate_merge_radius_m=0.75):
    cones = []
    for name, camera in cameras.items():
        cones.extend(camera_cones(
            camera['detections'], name, camera['info'], camera['transform'], cfg,
            min_bbox_height_px, min_bbox_width_px, bottom_margin_px,
            bottom_offset_px))
    cones = merge_camera_cones(cones, duplicate_merge_radius_m)
    plan = plan_corridor(cones, cfg)
    plane = np.array([0.0, 0.0, cfg.ground_z])
    return {'xyz': np.empty((0, 3)), 'cones': cones, 'plan': plan,
            'plane': plane, 'projections': {}}


def overlay_yolo_only(image, camera_name, detections, result):
    from .detector import BGR

    output = image.copy()
    accepted = {}
    for cone in result['cones']:
        for match in cone['matches']:
            if match['camera'] == camera_name:
                accepted[match['box']] = cone
    for index, box in enumerate(detections):
        x1, y1, x2, y2 = np.rint(box['xyxy']).astype(int)
        cone = accepted.get(index)
        color = BGR[box['color']] if cone is not None else (150, 150, 150)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        suffix = f" {cone['xyz'][0]:.1f}m" if cone is not None else ' rejected'
        label = f"{box['color']} {box['score']:.2f}{suffix}"
        cv2.putText(output, label, (x1, max(y1 - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, .38, color, 1, cv2.LINE_AA)
    cv2.putText(output, f"{camera_name} | YOLO only | path: {result['plan']['reason']}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5,
                (255, 255, 255), 2, cv2.LINE_AA)
    return output
