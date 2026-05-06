"""Pose conversion utilities for pallet fork control outputs."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class ForkPoseResult:
    """Forkable-front pose result expressed in a Cartesian reference frame."""

    T_final: np.ndarray
    fork_center: np.ndarray
    insertion_direction: np.ndarray
    yaw_rad: float
    yaw_deg: float
    distance: float
    lateral_error: float
    height: float
    fitness: float = 0.0
    inlier_rmse: float = 0.0
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """Serialize the result to JSON-friendly Python types."""

        return {
            "T_final": self.T_final.tolist(),
            "fork_center": self.fork_center.tolist(),
            "insertion_direction": self.insertion_direction.tolist(),
            "yaw_rad": float(self.yaw_rad),
            "yaw_deg": float(self.yaw_deg),
            "distance": float(self.distance),
            "lateral_error": float(self.lateral_error),
            "height": float(self.height),
            "fitness": float(self.fitness),
            "inlier_rmse": float(self.inlier_rmse),
            "warnings": list(self.warnings),
        }


@dataclass
class CameraPoseResult:
    """Compact 4-DoF target pose expressed in the camera coordinate frame."""

    position: np.ndarray
    insertion_direction: np.ndarray
    yaw_rad: float
    yaw_deg: float

    @property
    def x(self) -> float:
        return float(self.position[0])

    @property
    def y(self) -> float:
        return float(self.position[1])

    @property
    def z(self) -> float:
        return float(self.position[2])

    def to_dict(self) -> dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_rad": float(self.yaw_rad),
            "yaw_deg": float(self.yaw_deg),
            "position": self.position.tolist(),
            "insertion_direction": self.insertion_direction.tolist(),
        }


def _as_transform(T: np.ndarray) -> np.ndarray:
    transform = np.asarray(T, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"T_final must be a 4x4 homogeneous matrix, got {transform.shape}")
    return transform


def transform_point(T: np.ndarray, point: np.ndarray | list[float]) -> np.ndarray:
    """Transform a 3D point by a homogeneous transform."""

    transform = _as_transform(T)
    p = np.asarray(point, dtype=np.float64)
    if p.shape != (3,):
        raise ValueError(f"point must be a 3-vector, got {p.shape}")
    homogeneous = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
    return (transform @ homogeneous)[:3]


def compute_insertion_direction(R: np.ndarray, template_insert_axis: np.ndarray | list[float]) -> np.ndarray:
    """Rotate and normalize the template insertion axis into the body frame."""

    rotation = np.asarray(R, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError(f"R must be a 3x3 rotation matrix, got {rotation.shape}")
    axis = np.asarray(template_insert_axis, dtype=np.float64)
    if axis.shape != (3,):
        raise ValueError(f"template_insert_axis must be a 3-vector, got {axis.shape}")
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("template_insert_axis must be non-zero")
    direction = rotation @ (axis / axis_norm)
    direction_norm = np.linalg.norm(direction)
    if direction_norm == 0:
        raise ValueError("computed insertion direction is zero")
    return direction / direction_norm


def compute_yaw_from_insert_direction(d_insert: np.ndarray | list[float]) -> float:
    """Compute body-frame yaw with x forward and y left."""

    direction = np.asarray(d_insert, dtype=np.float64)
    if direction.shape != (3,):
        raise ValueError(f"d_insert must be a 3-vector, got {direction.shape}")
    return float(np.arctan2(direction[1], direction[0]))


def compute_fork_pose(
    T_final: np.ndarray,
    template_fork_center: np.ndarray | list[float],
    template_insert_axis: np.ndarray | list[float],
    fitness: float = 0.0,
    inlier_rmse: float = 0.0,
    warnings: list[str] | None = None,
) -> ForkPoseResult:
    """Compute fork center, insertion direction, yaw, and AGV control errors."""

    transform = _as_transform(T_final)
    fork_center = transform_point(transform, template_fork_center)
    d_insert = compute_insertion_direction(transform[:3, :3], template_insert_axis)
    yaw_rad = compute_yaw_from_insert_direction(d_insert)
    return ForkPoseResult(
        T_final=transform,
        fork_center=fork_center,
        insertion_direction=d_insert,
        yaw_rad=yaw_rad,
        yaw_deg=float(np.degrees(yaw_rad)),
        distance=float(fork_center[0]),
        lateral_error=float(fork_center[1]),
        height=float(fork_center[2]),
        fitness=float(fitness),
        inlier_rmse=float(inlier_rmse),
        warnings=list(warnings or []),
    )


def compute_camera_pose(
    T_camera: np.ndarray,
    template_fork_center: np.ndarray | list[float],
    template_insert_axis: np.ndarray | list[float],
) -> CameraPoseResult:
    """Compute target x/y/z/yaw in the camera frame."""

    transform = _as_transform(T_camera)
    position = transform_point(transform, template_fork_center)
    insertion_direction = compute_insertion_direction(transform[:3, :3], template_insert_axis)
    yaw_rad = compute_yaw_from_insert_direction(insertion_direction)
    return CameraPoseResult(
        position=position,
        insertion_direction=insertion_direction,
        yaw_rad=yaw_rad,
        yaw_deg=float(np.degrees(yaw_rad)),
    )
