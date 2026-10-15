"""Ultralytics housing detector. Candidates carry no route ID or driving decision."""
import cv2
import numpy as np
from pathlib import Path


class SignalDetector:
    def __init__(self, weights, device='0', imgsz=1280, confidence=0.15):
        from ultralytics import YOLO
        if not Path(weights).is_file():
            raise FileNotFoundError('Camera3 checkpoint not found: ' + weights)
        self.model = YOLO(weights)
        expected = {0: 'route_signal_3', 1: 'route_signal_4'}
        if self.model.names != expected:
            raise ValueError('Camera3 requires the trained 3/4-lamp housing checkpoint')
        self.options = dict(device=device, imgsz=imgsz, conf=confidence, verbose=False)
        # Initialize predictor/CUDA before the first 30 m window opens.
        self.model.predict(np.zeros((1440, 1920, 3), dtype=np.uint8), **self.options)

    def __call__(self, bgr):
        result = self.model.predict(bgr, **self.options)[0]
        candidates = [dict(class_id=int(box.cls.item()),
                           layout=self.model.names[int(box.cls.item())],
                           confidence=float(box.conf.item()),
                           xyxy=box.xyxy[0].cpu().tolist()) for box in result.boxes]
        result.names = {0: '3-lamp candidate', 1: '4-lamp candidate'}
        annotated = result.plot(line_width=1, font_size=12)
        cv2.putText(annotated, 'HOUSING CANDIDATES / ROUTE TARGET NOT SELECTED',
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 200, 255), 2)
        return annotated, candidates
