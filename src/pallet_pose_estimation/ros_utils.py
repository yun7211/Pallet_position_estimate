"""ROS1 conversion helpers for the pallet pose estimation pipeline."""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np

from .config import CameraIntrinsics
from .pose_output import ForkPoseResult


def ros_image_to_cv2(msg: Any, bridge: Any, desired_encoding: str) -> np.ndarray:
    """Convert a ROS sensor_msgs/Image to an OpenCV ndarray with cv_bridge."""

    try:
        return bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
    except Exception as exc:
        raise ValueError(f"Failed to convert ROS image with encoding '{desired_encoding}': {exc}") from exc


def camera_info_to_intrinsics(camera_info_msg: Any) -> CameraIntrinsics:
    """Convert sensor_msgs/CameraInfo.K to project CameraIntrinsics."""

    K = list(getattr(camera_info_msg, "K", []))
    if len(K) != 9:
        raise ValueError("CameraInfo.K must contain 9 values")
    fx, fy, cx, cy = float(K[0]), float(K[4]), float(K[2]), float(K[5])
    if fx <= 0 or fy <= 0:
        raise ValueError("CameraInfo contains invalid fx/fy; wait for valid CameraInfo or configure fallback_intrinsics")
    return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy)


def numpy_points_to_pointcloud2(points: np.ndarray, frame_id: str, stamp: Any) -> Any:
    """Convert Nx3 NumPy points to sensor_msgs/PointCloud2."""

    from sensor_msgs.msg import PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import Header

    array = np.asarray(points, dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"points must have shape Nx3, got {array.shape}")
    header = Header(stamp=stamp, frame_id=frame_id)
    if len(array) == 0:
        return PointCloud2(header=header)
    return point_cloud2.create_cloud_xyz32(header, array.tolist())


def yaw_to_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return quaternion x, y, z, w for a z-axis yaw in base_link."""

    half = float(yaw_rad) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def fork_pose_to_pose_stamped(fork_pose: ForkPoseResult, frame_id: str, stamp: Any) -> Any:
    """Convert ForkPoseResult to geometry_msgs/PoseStamped.

    The child pose origin is the fork center in the AGV body frame. Orientation
    is yaw-only around base_link z, matching the x-forward/y-left/z-up convention.
    """

    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(fork_pose.fork_center[0])
    msg.pose.position.y = float(fork_pose.fork_center[1])
    msg.pose.position.z = float(fork_pose.fork_center[2])
    qx, qy, qz, qw = yaw_to_quaternion(fork_pose.yaw_rad)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def fork_pose_to_json_msg(
    fork_pose: ForkPoseResult,
    warnings: list[str],
    processing_time_ms: float,
    success: bool = True,
    stamp: float | None = None,
    frame_id: str = "base_link",
    bbox: list[int] | None = None,
    target_label: str | None = None,
    selected_label: str | None = None,
    target_bbox: list[int] | None = None,
    selection_status: str | None = None,
    candidates: list[dict] | None = None,
    camera_pose: dict | None = None,
) -> Any:
    """Serialize pose result as std_msgs/String JSON."""

    from std_msgs.msg import String

    data = {
        "body_pose": fork_pose.to_dict(),
        "fitness": float(fork_pose.fitness),
        "inlier_rmse": float(fork_pose.inlier_rmse),
    }
    if camera_pose:
        data.update(
            {
                "x": camera_pose.get("x"),
                "y": camera_pose.get("y"),
                "z": camera_pose.get("z"),
                "yaw_rad": camera_pose.get("yaw_rad"),
                "yaw_deg": camera_pose.get("yaw_deg"),
                "camera_pose": dict(camera_pose),
            }
        )
    else:
        data.update(
            {
                "x": float(fork_pose.fork_center[0]),
                "y": float(fork_pose.fork_center[1]),
                "z": float(fork_pose.fork_center[2]),
                "yaw_rad": float(fork_pose.yaw_rad),
                "yaw_deg": float(fork_pose.yaw_deg),
                "camera_pose": {},
            }
        )
    data.update(
        {
            "success": bool(success),
            "stamp": stamp,
            "frame_id": frame_id,
            "bbox": bbox,
            "target_label": target_label,
            "selected_label": selected_label,
            "target_bbox": target_bbox,
            "selection_status": selection_status,
            "candidates": list(candidates or []),
            "camera_pose": dict(camera_pose or {}),
            "processing_time_ms": float(processing_time_ms),
            "warnings": list(warnings),
        }
    )
    msg = String()
    msg.data = json.dumps(data, ensure_ascii=False)
    return msg


def status_to_json_msg(success: bool, status_code: str, message: str, processing_time_ms: float = 0.0) -> Any:
    """Create a std_msgs/String status payload."""

    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps(
        {
            "success": bool(success),
            "status": status_code,
            "message": message,
            "processing_time_ms": float(processing_time_ms),
        },
        ensure_ascii=False,
    )
    return msg


def draw_ros_debug_image(
    color_image: np.ndarray,
    bbox: list[int] | None,
    fork_pose: ForkPoseResult | None,
    warnings: list[str],
) -> np.ndarray:
    """Draw detection and fork pose summary on a BGR debug image."""

    import cv2

    if color_image is None or color_image.size == 0:
        raise ValueError("color_image is empty")
    output = color_image.copy()
    if bbox is not None:
        u_min, v_min, u_max, v_max = [int(v) for v in bbox]
        cv2.rectangle(output, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)
        cv2.putText(output, "forkable_front", (u_min, max(20, v_min - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if fork_pose is not None:
        text = (
            f"center=({fork_pose.distance:.2f},{fork_pose.lateral_error:.2f},{fork_pose.height:.2f})m "
            f"yaw={fork_pose.yaw_deg:.1f} fit={fork_pose.fitness:.2f}"
        )
        cv2.putText(output, text, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if warnings:
        cv2.putText(output, warnings[0][:90], (16, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)
    return output
