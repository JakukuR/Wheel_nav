"""YOLO11 person detection with persistent ByteTrack IDs."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Track:
    track_id: int
    box: tuple
    confidence: float
    appearance: np.ndarray


def appearance_histogram(frame, box):
    x1, y1, x2, y2 = [int(v) for v in box]
    h, w = frame.shape[:2]
    x1 = max(0, min(w, x1)); x2 = max(0, min(w, x2))
    y1 = max(0, min(h, y1)); y2 = max(0, min(h, y2))
    if x2 - x1 < 8 or y2 - y1 < 12:
        return None
    # Central upper torso; omit face and most background near box edges.
    left = x1 + (x2 - x1) // 4
    right = x2 - (x2 - x1) // 4
    top = y1 + (y2 - y1) // 4
    bottom = y1 + 3 * (y2 - y1) // 5
    roi = frame[top:bottom, left:right]
    if roi.size == 0:
        return None
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    norm = np.linalg.norm(hist)
    return hist / norm if norm > 0 else None


class PersonDetector:
    def __init__(self, model_path, confidence=0.35, image_size=640):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        # COCO person class must be present in the supplied detection model.
        names = self.model.names
        self.person_class = next((int(k) for k, v in names.items() if v == 'person'), None)
        if self.person_class is None:
            raise ValueError('model does not contain COCO person class')
        self.confidence = confidence
        self.image_size = image_size

    def track(self, frame):
        results = self.model.track(
            frame, persist=True, tracker='bytetrack.yaml', classes=[self.person_class],
            conf=self.confidence, imgsz=self.image_size, verbose=False)
        boxes = results[0].boxes
        if boxes is None or boxes.id is None:
            return []
        output = []
        for box, track_id, confidence in zip(
                boxes.xyxy.cpu().numpy(), boxes.id.cpu().numpy(),
                boxes.conf.cpu().numpy()):
            desc = appearance_histogram(frame, box)
            if desc is not None:
                output.append(Track(int(track_id), tuple(float(v) for v in box),
                                    float(confidence), desc))
        return output
