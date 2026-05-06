from __future__ import annotations

from pallet_pose_estimation.assist_label import AssistLabelRecognizer


def test_normalize_label_text() -> None:
    normalize = AssistLabelRecognizer.normalize_label_text

    assert normalize("A102") == "A102"
    assert normalize("a-102") == "A102"
    assert normalize("A1O2") == "A102"
    assert normalize("B 2O3") == "B203"
    assert normalize("1234") is None
    assert normalize("") is None
    assert normalize("abc") is None
