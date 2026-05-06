"""Auxiliary in-box label OCR for selecting a target pallet candidate."""

from __future__ import annotations

from dataclasses import dataclass
import re
import time
from typing import Optional

import numpy as np


@dataclass
class PalletCandidate:
    """YOLO pallet-front candidate enriched with optional OCR label metadata."""

    candidate_id: int
    box_xyxy: tuple[int, int, int, int]
    class_name: str
    confidence: float
    label_text: Optional[str]
    ocr_confidence: Optional[float]
    ocr_ms: float
    label_crop: Optional[np.ndarray] = None

    def to_dict(self, include_crop: bool = False) -> dict:
        """Return a JSON-friendly candidate representation."""

        data = {
            "candidate_id": int(self.candidate_id),
            "box": [int(v) for v in self.box_xyxy],
            "class_name": self.class_name,
            "confidence": float(self.confidence),
            "label_text": self.label_text,
            "ocr_confidence": None if self.ocr_confidence is None else float(self.ocr_confidence),
            "ocr_ms": float(self.ocr_ms),
        }
        if include_crop and self.label_crop is not None:
            data["label_crop_shape"] = list(self.label_crop.shape)
        return data


class AssistLabelRecognizer:
    """Crop and OCR an auxiliary label strictly inside a YOLO detection box."""

    def __init__(
        self,
        pillar_width: int = 100,
        pillar_height: int = 100,
        hole_width: int = 200,
        ocr_languages: Optional[list[str]] = None,
        use_gpu: bool = True,
        enable_ocr: bool = True,
    ) -> None:
        self.pillar_width = int(pillar_width)
        self.pillar_height = int(pillar_height)
        self.hole_width = int(hole_width)
        self.ocr_languages = ocr_languages or ["en"]
        self.use_gpu = bool(use_gpu)
        self.enable_ocr = bool(enable_ocr)
        self._reader = None

    @staticmethod
    def normalize_label_text(raw_text: str) -> Optional[str]:
        """Normalize OCR text to a strict ``letter + digits`` label, e.g. ``A102``."""

        if raw_text is None:
            return None
        clean = re.sub(r"[^A-Za-z0-9]", "", str(raw_text).upper())
        if not clean:
            return None

        match = re.search(r"[A-Z]", clean)
        if match is None:
            return None
        clean = clean[match.start() :]
        if len(clean) < 2:
            return None

        head = clean[0]
        if not any(ch.isdigit() for ch in clean[1:]):
            return None
        confusable_map = {
            "O": "0",
            "Q": "0",
            "D": "0",
            "I": "1",
            "L": "1",
            "Z": "2",
            "S": "5",
            "B": "8",
            "G": "6",
        }
        tail = "".join(confusable_map.get(ch, ch) for ch in clean[1:])
        digits = "".join(ch for ch in tail if ch.isdigit())
        if not digits:
            return None

        label = f"{head}{digits}"
        return label if re.fullmatch(r"[A-Z]\d+", label) else None

    def crop_label_from_box(self, detection_box_image: np.ndarray) -> np.ndarray:
        """Crop the expected middle-pillar label region inside a YOLO box crop."""

        if detection_box_image is None or detection_box_image.size == 0:
            return detection_box_image

        img_height, img_width = detection_box_image.shape[:2]
        if img_width < 7 or img_height < 2:
            return detection_box_image[0:0, 0:0]

        pillar_width = max(float(self.pillar_width), 1.0)
        pillar_height = max(float(self.pillar_height), 1.0)
        hole_width = max(float(self.hole_width), 1.0)
        theoretical_width = 3.0 * pillar_width + 2.0 * hole_width
        pillar_ratio_error = abs(pillar_width - pillar_height) / max(pillar_width, 1.0)
        hole_ratio_error = abs(hole_width - 2.0 * pillar_width) / max(hole_width, 1.0)
        use_given_geometry = theoretical_width > 1.0 and pillar_ratio_error < 0.3 and hole_ratio_error < 0.3

        if use_given_geometry:
            scale_ratio = img_width / theoretical_width
            estimated_pillar_w = pillar_width * scale_ratio
            estimated_pillar_h = pillar_height * scale_ratio
        else:
            estimated_pillar_w = img_width / 7.0
            estimated_pillar_h = estimated_pillar_w

        estimated_pillar_h = float(np.clip(estimated_pillar_h, 1.0, img_height))
        middle_center_x = img_width * 0.5
        half_w = estimated_pillar_w * 0.5
        pad_x = int(round(estimated_pillar_w * 0.12))
        pad_top = int(round(estimated_pillar_h * 0.15))

        label_left = int(round(middle_center_x - half_w)) - pad_x
        label_right = int(round(middle_center_x + half_w)) + pad_x
        label_bottom = img_height
        label_top = int(round(img_height - estimated_pillar_h)) - pad_top

        label_left = max(0, label_left)
        label_right = min(img_width, label_right)
        label_top = max(0, label_top)
        label_bottom = min(img_height, label_bottom)

        if label_right - label_left < 2 or label_bottom - label_top < 2:
            return detection_box_image[0:0, 0:0]
        return detection_box_image[label_top:label_bottom, label_left:label_right]

    def recognize_label(self, label_image: np.ndarray) -> tuple[Optional[str], Optional[float], float]:
        """Run EasyOCR on a label crop and return normalized text, confidence, time_ms."""

        start = time.perf_counter()
        if not self.enable_ocr:
            return None, None, 0.0
        if label_image is None or label_image.size == 0:
            return None, None, 0.0
        try:
            reader = self._get_reader()
            result = reader.readtext(label_image)
        except Exception:
            return None, None, (time.perf_counter() - start) * 1000.0
        if not result:
            return None, None, (time.perf_counter() - start) * 1000.0

        texts: list[str] = []
        confidences: list[float] = []
        for item in result:
            if len(item) >= 2:
                texts.append(str(item[1]))
            if len(item) >= 3:
                try:
                    confidences.append(float(item[2]))
                except (TypeError, ValueError):
                    pass
        label = self.normalize_label_text(" ".join(texts).strip())
        confidence = max(confidences) if confidences else None
        return label, confidence, (time.perf_counter() - start) * 1000.0

    def _get_reader(self):
        if self._reader is not None:
            return self._reader
        try:
            import easyocr
        except ImportError as exc:
            raise ImportError(
                "easyocr is required when target_selection.enable_ocr=true. "
                "Install it with `pip install easyocr` or set enable_ocr=false."
            ) from exc
        try:
            self._reader = easyocr.Reader(self.ocr_languages, gpu=self.use_gpu, verbose=False)
        except Exception:
            self._reader = easyocr.Reader(self.ocr_languages, gpu=False, verbose=False)
        return self._reader
