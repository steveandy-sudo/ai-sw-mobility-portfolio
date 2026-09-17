"""Unmodified local YOLO checkpoint, with explicit cone class-name validation."""
from pathlib import Path
import os
import time

import cv2
import numpy as np


class Detector:
    def __init__(self, model_path, device='auto', image_size=800, confidence=0.5):
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'Local YOLO model does not exist: {path}')
        config_dir = Path.home() / '.cache' / 'kaiev26_aeb' / 'ultralytics'
        (config_dir / 'Ultralytics').mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('YOLO_CONFIG_DIR', str(config_dir))
        os.environ.setdefault('YOLO_AUTOINSTALL', 'False')
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(2)
        cv2.setNumThreads(2)
        self.device = ('0' if torch.cuda.is_available() else 'cpu') if device == 'auto' else device
        self.model = YOLO(str(path))
        names = {int(i): str(name).lower().replace('_', ' ').strip() for i, name in self.model.names.items()}
        self.colors = {i: name.split()[0] for i, name in names.items() if name in ('blue cone', 'yellow cone', 'red cone')}
        if set(self.colors.values()) != {'blue', 'yellow', 'red'}:
            raise ValueError(f'Expected blue/red/yellow cone classes, received {names}')
        self.image_size = int(image_size)
        self.confidence = float(confidence)
        # CUDA initialization occurs before the node reports readiness.
        self.predict({'left': np.zeros((600, 800, 3), np.uint8),
                      'right': np.zeros((600, 800, 3), np.uint8)})

    def predict(self, images):
        start = time.monotonic()
        results = self.model.predict(list(images.values()), imgsz=self.image_size,
                                     conf=self.confidence, iou=0.7, device=self.device,
                                     verbose=False, save=False, rect=True)
        output = {}
        for name, result in zip(images, results):
            output[name] = [{'xyxy': row[:4].tolist(), 'score': float(row[4]),
                             'color': self.colors[int(row[5])]}
                            for row in result.boxes.data.cpu().numpy()
                            if int(row[5]) in self.colors]
        return output, (time.monotonic() - start) * 1000


BGR = {'blue': (255, 100, 30), 'yellow': (0, 230, 255),
       'red': (40, 40, 255), 'unknown': (160, 160, 160)}


def overlay(image, camera_name, detections, result):
    out = image.copy()
    matched = {m['box'] for c in result['cones'] for m in c['matches'] if m['camera'] == camera_name}
    for i, box in enumerate(detections):
        x1, y1, x2, y2 = np.rint(box['xyxy']).astype(int)
        color = BGR[box['color']] if i in matched else (150, 150, 150)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f"{box['color']} {box['score']:.2f}" + (' +LiDAR' if i in matched else '')
        cv2.putText(out, label, (x1, max(y1 - 4, 12)), cv2.FONT_HERSHEY_SIMPLEX, .38, color, 1, cv2.LINE_AA)
    uv, visible = result['projections'][camera_name]
    for cone in result['cones']:
        for point in uv[cone['indices']][visible[cone['indices']]]:
            cv2.circle(out, tuple(np.rint(point).astype(int)), 2, BGR[cone['color']], -1)
    cv2.putText(out, f"{camera_name} | path: {result['plan']['reason']}", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2, cv2.LINE_AA)
    return out
