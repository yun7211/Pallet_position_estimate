"""Configuration loading for pallet front pose estimation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics used for depth back-projection."""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class PalletPoseConfig:
    """Runtime configuration for the offline pallet-front pose pipeline."""

    yolo_model_path: str
    yolo_class_name: str
    confidence_threshold: float
    depth_scale: float
    depth_min: float
    depth_max: float
    voxel_size: float
    normal_radius_factor: float
    fpfh_radius_factor: float
    ransac_distance_threshold_factor: float
    icp_max_correspondence_distance_factor: float
    icp_method: str
    camera_intrinsics: CameraIntrinsics
    extrinsic_T_B_C: np.ndarray
    template_point_cloud_path: str
    template_fork_center: np.ndarray
    template_insert_axis: np.ndarray
    agv_forward_axis: str = "x"
    agv_lateral_axis: str = "y"
    min_registration_fitness: float = 0.25
    max_registration_rmse_factor: float = 3.0
    min_fork_distance: float = 0.1
    max_fork_distance: float = 8.0
    raw: dict[str, Any] = field(default_factory=dict, repr=False)


def _required(data: dict[str, Any], key: str) -> Any:
    if key not in data:
        raise KeyError(f"Missing required config key: {key}")
    return data[key]


def _as_matrix4x4(value: Any, key: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"{key} must be a 4x4 matrix, got shape {matrix.shape}")
    if not np.allclose(matrix[3], np.array([0.0, 0.0, 0.0, 1.0])):
        raise ValueError(f"{key} must be a homogeneous transform with last row [0, 0, 0, 1]")
    return matrix


def _as_vector3(value: Any, key: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{key} must be a 3-vector, got shape {vector.shape}")
    return vector


def _load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {config_path}")
    return data


def config_from_dict(data: dict[str, Any]) -> PalletPoseConfig:
    """Build a validated :class:`PalletPoseConfig` from a dictionary."""

    intrinsics_data = _required(data, "camera_intrinsics")
    if not isinstance(intrinsics_data, dict):
        raise ValueError("camera_intrinsics must be a mapping with fx, fy, cx, cy")

    intrinsics = CameraIntrinsics(
        fx=float(_required(intrinsics_data, "fx")),
        fy=float(_required(intrinsics_data, "fy")),
        cx=float(_required(intrinsics_data, "cx")),
        cy=float(_required(intrinsics_data, "cy")),
    )
    if intrinsics.fx <= 0 or intrinsics.fy <= 0:
        raise ValueError("camera intrinsics fx and fy must be positive")

    depth_scale = float(_required(data, "depth_scale"))
    voxel_size = float(_required(data, "voxel_size"))
    if depth_scale <= 0:
        raise ValueError("depth_scale must be positive")
    if voxel_size <= 0:
        raise ValueError("voxel_size must be positive")

    icp_method = str(_required(data, "icp_method")).lower()
    if icp_method not in {"gicp", "point_to_plane"}:
        raise ValueError("icp_method must be either 'gicp' or 'point_to_plane'")

    return PalletPoseConfig(
        yolo_model_path=str(_required(data, "yolo_model_path")),
        yolo_class_name=str(_required(data, "yolo_class_name")),
        confidence_threshold=float(_required(data, "confidence_threshold")),
        depth_scale=depth_scale,
        depth_min=float(_required(data, "depth_min")),
        depth_max=float(_required(data, "depth_max")),
        voxel_size=voxel_size,
        normal_radius_factor=float(_required(data, "normal_radius_factor")),
        fpfh_radius_factor=float(_required(data, "fpfh_radius_factor")),
        ransac_distance_threshold_factor=float(_required(data, "ransac_distance_threshold_factor")),
        icp_max_correspondence_distance_factor=float(_required(data, "icp_max_correspondence_distance_factor")),
        icp_method=icp_method,
        camera_intrinsics=intrinsics,
        extrinsic_T_B_C=_as_matrix4x4(_required(data, "extrinsic_T_B_C"), "extrinsic_T_B_C"),
        template_point_cloud_path=str(_required(data, "template_point_cloud_path")),
        template_fork_center=_as_vector3(_required(data, "template_fork_center"), "template_fork_center"),
        template_insert_axis=_as_vector3(_required(data, "template_insert_axis"), "template_insert_axis"),
        agv_forward_axis=str(data.get("agv_forward_axis", "x")),
        agv_lateral_axis=str(data.get("agv_lateral_axis", "y")),
        min_registration_fitness=float(data.get("min_registration_fitness", 0.25)),
        max_registration_rmse_factor=float(data.get("max_registration_rmse_factor", 3.0)),
        min_fork_distance=float(data.get("min_fork_distance", 0.1)),
        max_fork_distance=float(data.get("max_fork_distance", 8.0)),
        raw=dict(data),
    )


def load_config(path: str | Path) -> PalletPoseConfig:
    """Read and validate a YAML configuration file."""

    return config_from_dict(_load_yaml(path))
