from __future__ import annotations

import numpy as np

from pallet_pose_estimation.config import CameraIntrinsics
from pallet_pose_estimation.rgbd import backproject_depth_to_points, crop_depth_by_bbox


def test_backproject_depth_to_points_uses_pinhole_formula_and_depth_scale() -> None:
    depth = np.array(
        [
            [1000, 2000],
            [0, 3000],
        ],
        dtype=np.uint16,
    )
    bbox = [10, 20, 12, 22]
    intrinsics = CameraIntrinsics(fx=2.0, fy=4.0, cx=10.0, cy=20.0)

    points = backproject_depth_to_points(
        depth,
        bbox,
        intrinsics,
        depth_scale=1000.0,
        depth_min=0.1,
        depth_max=3.0,
    )

    expected = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 2.0],
            [1.5, 0.75, 3.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_allclose(points, expected)


def test_crop_depth_by_bbox_clips_to_image_bounds() -> None:
    depth = np.arange(16, dtype=np.uint16).reshape(4, 4)

    cropped = crop_depth_by_bbox(depth, [-1, 1, 3, 10])

    np.testing.assert_array_equal(cropped, depth[1:4, 0:3])
