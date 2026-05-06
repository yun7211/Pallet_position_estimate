"""YOLO-based detector for the pallet forkable front surface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


class NoDetectionError(RuntimeError):
    """Raised when no pallet-front detection is available."""


@dataclass(frozen=True)
class Detection:
    """A single image-space detection."""

    bbox: list[float]
    confidence: float
    class_name: str


@dataclass(frozen=True)
class YoloBox:
    """A filtered YOLO bbox in image xyxy coordinates."""

    xyxy: tuple[int, int, int, int]
    class_name: str
    confidence: float
    class_id: int


class YOLODetector:
    """Detect the pallet forkable front surface with an Ultralytics YOLO model.

    The target is the local forkable front surface, e.g. classes named
    ``pallet_front`` or ``forkable_front``. This module intentionally does not
    fall back to a whole-pallet bounding box because downstream registration
    uses a front-surface template rather than a full pallet model.
    """

    def __init__(
        self,
        model_path: str,
        class_name: str = "pallet_front",
        confidence_threshold: float = 0.5,
    ) -> None:
        self.model_path = model_path
        self.class_name = class_name
        self.confidence_threshold = float(confidence_threshold)
        self.allowed_class_names = {class_name, "pallet_front", "forkable_front"}
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required for YOLODetector. Install dependencies with "
                "`pip install -r requirements.txt`."
            ) from exc
        self.model = YOLO(model_path)

    def detect(self, image: np.ndarray) -> Detection:
        """Return the highest-confidence forkable-front detection."""

        boxes = self.detect_all(image)
        best_box = max(boxes, key=lambda box: box.confidence)
        best = Detection(
            bbox=[float(v) for v in best_box.xyxy],
            confidence=best_box.confidence,
            class_name=best_box.class_name,
        )

        return best

    def detect_all(self, image: np.ndarray) -> list[YoloBox]:
        """Return all filtered pallet-front/forkable-front candidate boxes."""

        if image is None or image.size == 0:
            raise ValueError("Input image for YOLO detection is empty")
        height, width = image.shape[:2]
        results = self.model(image, verbose=False)
        if not results:
            raise NoDetectionError("YOLO returned no results for the input image")

        class_names = self._class_names()
        detections: list[YoloBox] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.detach().cpu().numpy()
            conf = boxes.conf.detach().cpu().numpy()
            cls = boxes.cls.detach().cpu().numpy().astype(int)
            for bbox, score, class_id in zip(xyxy, conf, cls):
                detected_name = class_names.get(int(class_id), str(class_id))
                if detected_name not in self.allowed_class_names:
                    continue
                if float(score) < self.confidence_threshold:
                    continue
                x1, y1, x2, y2 = self._clip_xyxy(bbox, width, height)
                if x2 <= x1 or y2 <= y1:
                    continue
                detections.append(
                    YoloBox(
                        xyxy=(x1, y1, x2, y2),
                        class_name=detected_name,
                        confidence=float(score),
                        class_id=int(class_id),
                    )
                )
        if not detections:
            raise NoDetectionError(
                f"No pallet-front/forkable-front detection above confidence "
                f"{self.confidence_threshold:.3f}. Check YOLO class names and weights."
            )
        return sorted(detections, key=lambda box: box.confidence, reverse=True)

    def detect_pallet_front(self, image: np.ndarray) -> tuple[int, int, int, int]:
        """Return ``[u_min, v_min, u_max, v_max]`` for the best detection."""

        box = self.detect_all(image)[0]
        return box.xyxy

    def _class_names(self) -> dict[int, str]:
        names: Any = getattr(self.model, "names", {})
        if isinstance(names, dict):
            return {int(k): str(v) for k, v in names.items()}
        if isinstance(names, (list, tuple)):
            return {i: str(v) for i, v in enumerate(names)}
        return {}

    @staticmethod
    def _clip_xyxy(bbox: np.ndarray, width: int, height: int) -> tuple[int, int, int, int]:
        x1 = max(0, min(width, int(np.floor(float(bbox[0])))))
        y1 = max(0, min(height, int(np.floor(float(bbox[1])))))
        x2 = max(0, min(width, int(np.ceil(float(bbox[2])))))
        y2 = max(0, min(height, int(np.ceil(float(bbox[3])))))
        return x1, y1, x2, y2
