"""Target pallet selection using YOLO candidates and auxiliary label OCR."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .assist_label import AssistLabelRecognizer, PalletCandidate
from .detector import NoDetectionError, YOLODetector, YoloBox


@dataclass
class TargetSelectionResult:
    """Result of selecting a single target pallet-front candidate."""

    success: bool
    target_bbox: Optional[tuple[int, int, int, int]]
    target_label: Optional[str]
    matched_candidate: Optional[PalletCandidate]
    candidates: list[PalletCandidate]
    status: str
    warnings: list[str] = field(default_factory=list)


class TargetPalletSelector:
    """Select one target pallet-front bbox before RGB-D point-cloud processing."""

    VALID_FALLBACKS = {"error", "highest_confidence", "center_nearest", "first_valid_label"}

    def __init__(
        self,
        detector: YOLODetector,
        label_recognizer: AssistLabelRecognizer,
        target_label: Optional[str] = None,
        fallback_strategy: str = "error",
        min_detection_confidence: float = 0.25,
        label_match_mode: str = "exact",
        enable_assist_label: bool = True,
    ) -> None:
        self.detector = detector
        self.label_recognizer = label_recognizer
        self.target_label = AssistLabelRecognizer.normalize_label_text(target_label or "") if target_label else None
        self.fallback_strategy = str(fallback_strategy or "error")
        if self.fallback_strategy not in self.VALID_FALLBACKS:
            raise ValueError(f"fallback_strategy must be one of {sorted(self.VALID_FALLBACKS)}")
        self.min_detection_confidence = float(min_detection_confidence)
        self.label_match_mode = str(label_match_mode or "exact")
        if self.label_match_mode != "exact":
            raise ValueError("Only label_match_mode='exact' is currently supported")
        self.enable_assist_label = bool(enable_assist_label)

    def select_target(self, color_image: np.ndarray) -> TargetSelectionResult:
        """Detect all pallet-front candidates, OCR labels in-box, and select one."""

        if color_image is None or color_image.size == 0:
            raise ValueError("color_image is empty")
        try:
            yolo_boxes = self.detector.detect_all(color_image)
        except NoDetectionError:
            return TargetSelectionResult(False, None, self.target_label, None, [], "no_detection", [])

        yolo_boxes = [box for box in yolo_boxes if box.confidence >= self.min_detection_confidence]
        if not yolo_boxes:
            return TargetSelectionResult(False, None, self.target_label, None, [], "no_detection", [])

        candidates = self._build_candidates(color_image, yolo_boxes)
        warnings: list[str] = []
        if self.target_label:
            matches = [candidate for candidate in candidates if candidate.label_text == self.target_label]
            if not matches:
                return TargetSelectionResult(
                    False,
                    None,
                    self.target_label,
                    None,
                    candidates,
                    "target_label_not_found",
                    warnings,
                )
            matches.sort(key=lambda candidate: candidate.confidence, reverse=True)
            if len(matches) > 1:
                warnings.append(
                    f"Multiple candidates matched target_label={self.target_label}; selected highest confidence."
                )
            selected = matches[0]
            return TargetSelectionResult(
                True,
                selected.box_xyxy,
                self.target_label,
                selected,
                candidates,
                "matched",
                warnings,
            )

        selected = self._select_without_target_label(color_image, candidates, warnings)
        if selected is None:
            return TargetSelectionResult(
                False,
                None,
                None,
                None,
                candidates,
                "target_label_required",
                warnings,
            )
        return TargetSelectionResult(
            True,
            selected.box_xyxy,
            None,
            selected,
            candidates,
            self.fallback_strategy,
            warnings,
        )

    def _build_candidates(self, color_image: np.ndarray, boxes: list[YoloBox]) -> list[PalletCandidate]:
        candidates: list[PalletCandidate] = []
        for idx, box in enumerate(boxes):
            x1, y1, x2, y2 = box.xyxy
            box_crop = color_image[y1:y2, x1:x2]
            label_crop = None
            label_text = None
            ocr_confidence = None
            ocr_ms = 0.0
            if self.enable_assist_label:
                label_crop = self.label_recognizer.crop_label_from_box(box_crop)
                label_text, ocr_confidence, ocr_ms = self.label_recognizer.recognize_label(label_crop)
            candidates.append(
                PalletCandidate(
                    candidate_id=idx,
                    box_xyxy=box.xyxy,
                    class_name=box.class_name,
                    confidence=box.confidence,
                    label_text=label_text,
                    ocr_confidence=ocr_confidence,
                    ocr_ms=ocr_ms,
                    label_crop=label_crop,
                )
            )
        return candidates

    def _select_without_target_label(
        self,
        color_image: np.ndarray,
        candidates: list[PalletCandidate],
        warnings: list[str],
    ) -> Optional[PalletCandidate]:
        if self.fallback_strategy == "error":
            return None
        warnings.append(
            f"No target_label configured; using fallback_strategy={self.fallback_strategy}. "
            "This is not recommended for multi-pallet fork picking."
        )
        if self.fallback_strategy == "highest_confidence":
            return max(candidates, key=lambda candidate: candidate.confidence)
        if self.fallback_strategy == "first_valid_label":
            return next((candidate for candidate in candidates if candidate.label_text is not None), None)
        if self.fallback_strategy == "center_nearest":
            h, w = color_image.shape[:2]
            center = np.array([w * 0.5, h * 0.5], dtype=float)
            return min(candidates, key=lambda candidate: self._box_center_distance(candidate, center))
        return None

    @staticmethod
    def _box_center_distance(candidate: PalletCandidate, center: np.ndarray) -> float:
        x1, y1, x2, y2 = candidate.box_xyxy
        box_center = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5], dtype=float)
        return float(np.linalg.norm(box_center - center))
