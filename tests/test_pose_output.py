from __future__ import annotations

import numpy as np

from pallet_pose_estimation.pose_output import (
    compute_camera_pose,
    compute_fork_pose,
    compute_insertion_direction,
    compute_yaw_from_insert_direction,
    transform_point,
)


def test_compute_fork_pose_identity_transform() -> None:
    T = np.eye(4)

    result = compute_fork_pose(T, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0], fitness=0.8, inlier_rmse=0.01)

    np.testing.assert_allclose(result.fork_center, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(result.insertion_direction, [1.0, 0.0, 0.0])
    assert result.yaw_rad == 0.0
    assert result.yaw_deg == 0.0
    assert result.distance == 1.0
    assert result.lateral_error == 2.0
    assert result.height == 3.0
    assert result.fitness == 0.8
    assert result.inlier_rmse == 0.01


def test_compute_fork_pose_rotates_insert_axis_and_yaw() -> None:
    theta = np.pi / 2.0
    T = np.array(
        [
            [np.cos(theta), -np.sin(theta), 0.0, 0.5],
            [np.sin(theta), np.cos(theta), 0.0, -0.2],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    result = compute_fork_pose(T, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])

    np.testing.assert_allclose(result.fork_center, [0.5, -0.2, 0.3])
    np.testing.assert_allclose(result.insertion_direction, [0.0, 1.0, 0.0], atol=1e-12)
    assert np.isclose(result.yaw_rad, np.pi / 2.0)
    assert np.isclose(result.yaw_deg, 90.0)
    assert result.distance == 0.5
    assert result.lateral_error == -0.2


def test_low_level_pose_helpers() -> None:
    T = np.eye(4)
    T[:3, 3] = [1.0, 2.0, 3.0]

    np.testing.assert_allclose(transform_point(T, [4.0, 5.0, 6.0]), [5.0, 7.0, 9.0])
    np.testing.assert_allclose(compute_insertion_direction(np.eye(3), [2.0, 0.0, 0.0]), [1.0, 0.0, 0.0])
    assert compute_yaw_from_insert_direction([0.0, 1.0, 0.0]) == np.pi / 2.0


def test_compute_camera_pose_returns_xyz_yaw() -> None:
    T = np.eye(4)
    T[:3, 3] = [0.4, 0.5, 0.6]

    result = compute_camera_pose(T, [0.1, 0.0, 0.0], [1.0, 0.0, 0.0])

    np.testing.assert_allclose(result.position, [0.5, 0.5, 0.6])
    assert np.isclose(result.x, 0.5)
    assert np.isclose(result.y, 0.5)
    assert np.isclose(result.z, 0.6)
    assert np.isclose(result.yaw_deg, 0.0)
