"""RGB-D ROI cropping and depth back-projection utilities."""

from __future__ import annotations

import numpy as np

from .config import CameraIntrinsics


def _clip_bbox_to_image(bbox: list[float] | tuple[float, float, float, float], shape: tuple[int, ...]) -> tuple[int, int, int, int]:
    if len(bbox) != 4:
        raise ValueError(f"bbox must contain four values [u_min, v_min, u_max, v_max], got {bbox}")
    height, width = shape[:2]
    u_min = max(0, min(width, int(np.floor(bbox[0]))))
    v_min = max(0, min(height, int(np.floor(bbox[1]))))
    u_max = max(0, min(width, int(np.ceil(bbox[2]))))
    v_max = max(0, min(height, int(np.ceil(bbox[3]))))
    if u_max <= u_min or v_max <= v_min:
        raise ValueError(f"bbox {bbox} is outside the image or has non-positive area for image shape {shape}")
    return u_min, v_min, u_max, v_max


def crop_depth_by_bbox(depth: np.ndarray, bbox: list[float] | tuple[float, float, float, float]) -> np.ndarray:
    """Crop a depth image by an image-space bounding box."""

    if depth is None or depth.ndim != 2:
        raise ValueError("depth must be a non-empty HxW array")
    u_min, v_min, u_max, v_max = _clip_bbox_to_image(bbox, depth.shape)
    return depth[v_min:v_max, u_min:u_max]


def backproject_depth_to_points(
    depth_roi: np.ndarray,
    bbox: list[float] | tuple[float, float, float, float],
    intrinsics: CameraIntrinsics,
    depth_scale: float,
    depth_min: float,
    depth_max: float,
) -> np.ndarray:
    """Back-project an ROI depth image to camera-frame 3D points.

    ``depth_scale`` converts raw depth values to meters using ``Z = raw / depth_scale``.
    For meter-valued float depth images, set ``depth_scale`` to ``1.0``.
    Invalid, zero, NaN, too-near, and too-far depths are skipped.
    """

    if depth_roi is None or depth_roi.ndim != 2:
        raise ValueError("depth_roi must be a non-empty HxW array")
    if depth_scale <= 0:
        raise ValueError("depth_scale must be positive")
    if depth_min < 0 or depth_max <= depth_min:
        raise ValueError("depth_min/depth_max must define a valid positive range")

    u_min = int(np.floor(bbox[0]))
    v_min = int(np.floor(bbox[1]))
    rows, cols = np.indices(depth_roi.shape)
    u = cols.astype(np.float64) + float(u_min)
    v = rows.astype(np.float64) + float(v_min)

    z = depth_roi.astype(np.float64) / float(depth_scale)
    valid = np.isfinite(z) & (z > 0.0) & (z >= depth_min) & (z <= depth_max)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)

    z_valid = z[valid]
    x = (u[valid] - float(intrinsics.cx)) * z_valid / float(intrinsics.fx)
    y = (v[valid] - float(intrinsics.cy)) * z_valid / float(intrinsics.fy)
    return np.column_stack((x, y, z_valid)).astype(np.float64, copy=False)
