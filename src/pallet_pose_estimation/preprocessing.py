"""Point-cloud preprocessing for pallet-front ROI clouds."""

from __future__ import annotations

from typing import Optional

import numpy as np
import open3d as o3d


def numpy_to_o3d(points: np.ndarray) -> o3d.geometry.PointCloud:
    """Convert an ``Nx3`` NumPy array into an Open3D point cloud."""

    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"points must have shape Nx3, got {array.shape}")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(array)
    return pcd


def transform_point_cloud(pcd: o3d.geometry.PointCloud, T: np.ndarray) -> o3d.geometry.PointCloud:
    """Return a transformed copy of ``pcd`` using a 4x4 homogeneous matrix."""

    transform = np.asarray(T, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"T must be a 4x4 homogeneous transform, got {transform.shape}")
    transformed = o3d.geometry.PointCloud(pcd)
    transformed.transform(transform)
    return transformed


def _finite_point_cloud(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    points = np.asarray(pcd.points)
    if points.size == 0:
        return o3d.geometry.PointCloud(pcd)
    mask = np.all(np.isfinite(points), axis=1)
    return pcd.select_by_index(np.flatnonzero(mask).tolist())


def preprocess_point_cloud(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    nb_neighbors: int = 20,
    std_ratio: float = 2.0,
) -> o3d.geometry.PointCloud:
    """Apply conservative finite filtering, voxel downsampling, and outlier removal."""

    if voxel_size <= 0:
        raise ValueError("voxel_size must be positive")
    filtered = _finite_point_cloud(pcd)
    if len(filtered.points) == 0:
        return filtered
    down = filtered.voxel_down_sample(voxel_size)
    if len(down.points) >= max(3, nb_neighbors):
        down, _ = down.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return down


def remove_ground_ransac(
    pcd: o3d.geometry.PointCloud,
    distance_threshold: float,
    vertical_axis: np.ndarray = np.array([0.0, 0.0, 1.0]),
    normal_dot_threshold: float = 0.85,
) -> tuple[o3d.geometry.PointCloud, Optional[np.ndarray]]:
    """Remove a dominant horizontal ground plane when RANSAC finds one.

    The removal is deliberately conservative so fork-hole boundaries and front
    uprights are not stripped away with nearby floor returns.
    """

    if distance_threshold <= 0:
        raise ValueError("distance_threshold must be positive")
    if len(pcd.points) < 3:
        return o3d.geometry.PointCloud(pcd), None

    axis = np.asarray(vertical_axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("vertical_axis must be non-zero")
    axis /= norm

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000,
    )
    normal = np.asarray(plane_model[:3], dtype=np.float64)
    normal_norm = np.linalg.norm(normal)
    if normal_norm == 0:
        return o3d.geometry.PointCloud(pcd), np.asarray(plane_model, dtype=np.float64)
    normal /= normal_norm
    if abs(float(np.dot(normal, axis))) < normal_dot_threshold:
        return o3d.geometry.PointCloud(pcd), np.asarray(plane_model, dtype=np.float64)
    return pcd.select_by_index(inliers, invert=True), np.asarray(plane_model, dtype=np.float64)


def filter_by_height_and_roi(
    pcd: o3d.geometry.PointCloud,
    z_min: float | None = None,
    z_max: float | None = None,
    y_min: float | None = None,
    y_max: float | None = None,
) -> o3d.geometry.PointCloud:
    """Filter a body-frame ROI cloud by simple height and lateral bounds."""

    points = np.asarray(pcd.points)
    if points.size == 0:
        return o3d.geometry.PointCloud(pcd)
    mask = np.ones(len(points), dtype=bool)
    if z_min is not None:
        mask &= points[:, 2] >= float(z_min)
    if z_max is not None:
        mask &= points[:, 2] <= float(z_max)
    if y_min is not None:
        mask &= points[:, 1] >= float(y_min)
    if y_max is not None:
        mask &= points[:, 1] <= float(y_max)
    return pcd.select_by_index(np.flatnonzero(mask).tolist())
