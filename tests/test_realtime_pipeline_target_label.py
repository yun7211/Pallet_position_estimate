from __future__ import annotations

from types import SimpleNamespace
import sys

import numpy as np

from pallet_pose_estimation.config import config_from_dict
from pallet_pose_estimation.target_selector import TargetSelectionResult
from pallet_pose_estimation.realtime_pipeline import PalletPoseRealtimePipeline


class FakePointCloud:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)


class FakeSelector:
    def __init__(self, result):
        self.result = result
        self.target_label = result.target_label

    def select_target(self, color_image):
        return self.result


class FakeCandidate:
    label_text = "A102"

    def to_dict(self):
        return {"label_text": "A102"}


def _cfg():
    return config_from_dict(
        {
            "yolo_model_path": "unused.pt",
            "yolo_class_name": "pallet_front",
            "confidence_threshold": 0.5,
            "depth_scale": 1000.0,
            "depth_min": 0.1,
            "depth_max": 5.0,
            "voxel_size": 0.03,
            "normal_radius_factor": 2.0,
            "fpfh_radius_factor": 5.0,
            "ransac_distance_threshold_factor": 1.5,
            "icp_max_correspondence_distance_factor": 1.5,
            "icp_method": "point_to_plane",
            "camera_intrinsics": {"fx": 1.0, "fy": 1.0, "cx": 0.0, "cy": 0.0},
            "extrinsic_T_B_C": np.eye(4).tolist(),
            "template_point_cloud_path": "unused.ply",
            "template_fork_center": [1.0, 0.0, 0.0],
            "template_insert_axis": [1.0, 0.0, 0.0],
            "target_selection": {
                "enable_assist_label": True,
                "enable_ocr": False,
                "target_label": "A102",
                "fallback_strategy": "error",
                "min_detection_confidence": 0.25,
            },
            "rgbd": {"require_aligned_depth_to_color": True},
        }
    )


def _patch_heavy_modules(monkeypatch, calls):
    fake_preprocessing = SimpleNamespace(
        numpy_to_o3d=lambda points: FakePointCloud(points),
        transform_point_cloud=lambda pcd, T: pcd,
        preprocess_point_cloud=lambda pcd, voxel_size: pcd,
        remove_ground_ransac=lambda pcd, distance_threshold: (pcd, None),
        filter_by_height_and_roi=lambda pcd, **kwargs: pcd,
    )

    def register_template_to_scene(template, scene, cfg):
        calls["registration"] += 1
        return SimpleNamespace(T_init=np.eye(4), T_final=np.eye(4), fitness=0.9, inlier_rmse=0.01, warnings=[])

    fake_registration = SimpleNamespace(register_template_to_scene=register_template_to_scene)
    monkeypatch.setitem(sys.modules, "pallet_pose_estimation.preprocessing", fake_preprocessing)
    monkeypatch.setitem(sys.modules, "pallet_pose_estimation.registration", fake_registration)


def test_pipeline_uses_selected_target_bbox_for_depth_roi(monkeypatch) -> None:
    calls = {"backproject_bbox": None, "registration": 0}
    _patch_heavy_modules(monkeypatch, calls)

    def fake_backproject(depth_roi, bbox, intrinsics, depth_scale, depth_min, depth_max):
        calls["backproject_bbox"] = tuple(bbox)
        assert depth_roi.shape == (2, 2)
        return np.ones((4, 3), dtype=float)

    monkeypatch.setattr("pallet_pose_estimation.realtime_pipeline.backproject_depth_to_points", fake_backproject)
    result = TargetSelectionResult(
        success=True,
        target_bbox=(2, 2, 4, 4),
        target_label="A102",
        matched_candidate=FakeCandidate(),
        candidates=[FakeCandidate()],
        status="matched",
        warnings=[],
    )
    pipeline = PalletPoseRealtimePipeline(
        _cfg(),
        detector=object(),
        template_pcd=FakePointCloud([[0, 0, 0]]),
        target_selector=FakeSelector(result),
    )

    output = pipeline.process_rgbd(
        np.zeros((5, 5, 3), dtype=np.uint8),
        np.full((5, 5), 1000, dtype=np.uint16),
        _cfg().camera_intrinsics,
    )

    assert output.success
    assert output.target_bbox == [2, 2, 4, 4]
    assert calls["backproject_bbox"] == (2, 2, 4, 4)
    assert calls["registration"] == 1
    assert output.camera_pose is not None


def test_pipeline_does_not_register_when_target_label_not_found(monkeypatch) -> None:
    calls = {"registration": 0}
    _patch_heavy_modules(monkeypatch, calls)
    result = TargetSelectionResult(
        success=False,
        target_bbox=None,
        target_label="A102",
        matched_candidate=None,
        candidates=[],
        status="target_label_not_found",
        warnings=[],
    )
    pipeline = PalletPoseRealtimePipeline(
        _cfg(),
        detector=object(),
        template_pcd=FakePointCloud([[0, 0, 0]]),
        target_selector=FakeSelector(result),
    )

    output = pipeline.process_rgbd(
        np.zeros((5, 5, 3), dtype=np.uint8),
        np.full((5, 5), 1000, dtype=np.uint16),
        _cfg().camera_intrinsics,
    )

    assert not output.success
    assert output.status_code == "target_label_not_found"
    assert calls["registration"] == 0
