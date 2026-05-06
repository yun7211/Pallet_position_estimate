from __future__ import annotations

from types import SimpleNamespace

from pallet_pose_estimation.ros_utils import camera_info_to_intrinsics


def test_camera_info_to_intrinsics_extracts_k_matrix() -> None:
    msg = SimpleNamespace(K=[610.0, 0.0, 321.0, 0.0, 611.0, 242.0, 0.0, 0.0, 1.0])

    intrinsics = camera_info_to_intrinsics(msg)

    assert intrinsics.fx == 610.0
    assert intrinsics.fy == 611.0
    assert intrinsics.cx == 321.0
    assert intrinsics.cy == 242.0
