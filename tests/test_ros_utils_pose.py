from __future__ import annotations

import importlib.util

import numpy as np
import pytest

from pallet_pose_estimation.pose_output import ForkPoseResult
from pallet_pose_estimation.ros_utils import fork_pose_to_pose_stamped


pytestmark = pytest.mark.skipif(
    importlib.util.find_spec("geometry_msgs") is None,
    reason="ROS geometry_msgs is not available in this Python environment",
)


def test_fork_pose_to_pose_stamped_position_and_yaw_quaternion() -> None:
    pose = ForkPoseResult(
        T_final=np.eye(4),
        fork_center=np.array([1.25, -0.03, 0.18]),
        insertion_direction=np.array([0.0, 1.0, 0.0]),
        yaw_rad=np.pi / 2.0,
        yaw_deg=90.0,
        distance=1.25,
        lateral_error=-0.03,
        height=0.18,
        fitness=0.72,
        inlier_rmse=0.012,
    )

    msg = fork_pose_to_pose_stamped(pose, "base_link", None)

    assert msg.header.frame_id == "base_link"
    assert msg.pose.position.x == 1.25
    assert msg.pose.position.y == -0.03
    assert msg.pose.position.z == 0.18
    assert np.isclose(msg.pose.orientation.z, np.sin(np.pi / 4.0))
    assert np.isclose(msg.pose.orientation.w, np.cos(np.pi / 4.0))
