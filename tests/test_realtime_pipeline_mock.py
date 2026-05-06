from __future__ import annotations

from types import SimpleNamespace
import sys

import numpy as np

from pallet_pose_estimation.config import config_from_dict
from pallet_pose_estimation.detector import YoloBox
from pallet_pose_estimation.realtime_pipeline import PalletPoseRealtimePipeline


class FakeDetector:
    def detect_all(self, image):
        return [YoloBox((0, 0, 3, 3), "pallet_front", 0.9, 0)]


class FakePointCloud:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)


def _base_config():
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
            "agv_forward_axis": "x",
            "agv_lateral_axis": "y",
            "target_selection": {
                "enable_assist_label": True,
                "enable_ocr": False,
                "target_label": "",
                "fallback_strategy": "highest_confidence",
                "min_detection_confidence": 0.25,
            },
            "rgbd": {"require_aligned_depth_to_color": True},
        }
    )


def test_realtime_pipeline_process_rgbd_with_mocked_geometry(monkeypatch) -> None:
    cfg = _base_config()

    _patch_geometry_modules(monkeypatch)
    pipeline = PalletPoseRealtimePipeline(cfg, detector=FakeDetector(), template_pcd=FakePointCloud([[0, 0, 0]]))
    color = np.zeros((3, 3, 3), dtype=np.uint8)
    depth = np.full((3, 3), 1000, dtype=np.uint16)

    result = pipeline.process_rgbd(color, depth, cfg.camera_intrinsics)

    assert result.success
    assert result.bbox == [0, 0, 3, 3]
    assert result.raw_roi_points is not None
    assert result.fork_pose is not None
    assert result.fork_pose.distance == 1.0


def _patch_geometry_modules(monkeypatch) -> None:
    fake_preprocessing = SimpleNamespace(
        numpy_to_o3d=lambda points: FakePointCloud(points),
        transform_point_cloud=lambda pcd, T: pcd,
        preprocess_point_cloud=lambda pcd, voxel_size: pcd,
        remove_ground_ransac=lambda pcd, distance_threshold: (pcd, None),
        filter_by_height_and_roi=lambda pcd, **kwargs: pcd,
    )
    fake_registration = SimpleNamespace(
        register_template_to_scene=lambda template, scene, cfg: SimpleNamespace(
            T_init=np.eye(4),
            T_final=np.eye(4),
            fitness=0.9,
            inlier_rmse=0.01,
            warnings=[],
        )
    )
    monkeypatch.setitem(sys.modules, "pallet_pose_estimation.preprocessing", fake_preprocessing)
    monkeypatch.setitem(sys.modules, "pallet_pose_estimation.registration", fake_registration)


def test_realtime_pipeline_reports_empty_roi_error(monkeypatch) -> None:
    cfg = _base_config()
    _patch_geometry_modules(monkeypatch)
    pipeline = PalletPoseRealtimePipeline(cfg, detector=FakeDetector(), template_pcd=FakePointCloud([[0, 0, 0]]))
    color = np.zeros((3, 3, 3), dtype=np.uint8)
    depth = np.zeros((3, 3), dtype=np.uint16)

    result = pipeline.process_rgbd(color, depth, cfg.camera_intrinsics)

    assert not result.success
    assert result.status_code == "empty_roi_cloud"
