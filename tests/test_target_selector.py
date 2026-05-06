from __future__ import annotations

import numpy as np

from pallet_pose_estimation.detector import YoloBox
from pallet_pose_estimation.target_selector import TargetPalletSelector


class FakeDetector:
    def __init__(self, boxes):
        self.boxes = boxes

    def detect_all(self, image):
        return self.boxes


class FakeRecognizer:
    def __init__(self, labels):
        self.labels = list(labels)
        self.idx = 0

    def crop_label_from_box(self, detection_box_image):
        return detection_box_image

    def recognize_label(self, label_image):
        label = self.labels[self.idx]
        self.idx += 1
        return label, 0.9 if label else None, 1.0


def _boxes():
    return [
        YoloBox((0, 0, 10, 10), "pallet_front", 0.8, 0),
        YoloBox((20, 0, 30, 10), "pallet_front", 0.9, 0),
    ]


def test_selects_matching_target_label() -> None:
    selector = TargetPalletSelector(FakeDetector(_boxes()), FakeRecognizer(["B203", "A102"]), target_label="A102")

    result = selector.select_target(np.zeros((20, 40, 3), dtype=np.uint8))

    assert result.success
    assert result.target_bbox == (20, 0, 30, 10)
    assert result.matched_candidate.label_text == "A102"


def test_target_label_not_found_returns_failure() -> None:
    selector = TargetPalletSelector(FakeDetector(_boxes()), FakeRecognizer(["B203", None]), target_label="A102")

    result = selector.select_target(np.zeros((20, 40, 3), dtype=np.uint8))

    assert not result.success
    assert result.status == "target_label_not_found"


def test_duplicate_label_selects_highest_confidence_with_warning() -> None:
    selector = TargetPalletSelector(FakeDetector(_boxes()), FakeRecognizer(["A102", "A102"]), target_label="A102")

    result = selector.select_target(np.zeros((20, 40, 3), dtype=np.uint8))

    assert result.success
    assert result.target_bbox == (20, 0, 30, 10)
    assert result.warnings


def test_fallback_highest_confidence_without_target_label() -> None:
    selector = TargetPalletSelector(
        FakeDetector(_boxes()),
        FakeRecognizer([None, None]),
        target_label=None,
        fallback_strategy="highest_confidence",
    )

    result = selector.select_target(np.zeros((20, 40, 3), dtype=np.uint8))

    assert result.success
    assert result.target_bbox == (20, 0, 30, 10)


def test_fallback_error_requires_target_label() -> None:
    selector = TargetPalletSelector(
        FakeDetector(_boxes()),
        FakeRecognizer([None, None]),
        target_label=None,
        fallback_strategy="error",
    )

    result = selector.select_target(np.zeros((20, 40, 3), dtype=np.uint8))

    assert not result.success
    assert result.status == "target_label_required"
