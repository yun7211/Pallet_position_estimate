#!/usr/bin/env python3
"""ROS1 Noetic node for pallet forkable-front pose estimation."""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
from typing import Any

import numpy as np

from .config import CameraIntrinsics, load_config
from .realtime_pipeline import PalletPoseRealtimePipeline
from .ros_utils import (
    camera_info_to_intrinsics,
    draw_ros_debug_image,
    fork_pose_to_json_msg,
    fork_pose_to_pose_stamped,
    numpy_points_to_pointcloud2,
    ros_image_to_cv2,
    status_to_json_msg,
    yaw_to_quaternion,
)


def _import_ros():
    try:
        import rospy
        import message_filters
        import tf
        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise ImportError(
            "ROS1 Noetic Python packages are required. Source your ROS setup.bash and install "
            "cv_bridge, message_filters, tf, sensor_msgs, geometry_msgs, and std_msgs."
        ) from exc
    return rospy, message_filters, tf, CvBridge, CameraInfo, Image, String, PoseStamped, PointCloud2


class PalletPoseRos1Node:
    """Subscribe to aligned RealSense RGB-D topics and publish fork pose outputs."""

    def __init__(self) -> None:
        (
            self.rospy,
            self.message_filters,
            self.tf,
            CvBridge,
            CameraInfo,
            Image,
            String,
            PoseStamped,
            PointCloud2,
        ) = _import_ros()
        self.bridge = CvBridge()
        self.rospy.init_node("pallet_pose_ros1_node", anonymous=False)

        config_path = Path(self.rospy.get_param("~config_path", "config/ros1_noetic.yaml"))
        cfg = load_config(config_path)
        yolo_override = self.rospy.get_param("~yolo_model_path", "")
        template_override = self.rospy.get_param("~template_point_cloud_path", "")
        if yolo_override:
            cfg = replace(cfg, yolo_model_path=str(yolo_override))
        if template_override:
            cfg = replace(cfg, template_point_cloud_path=str(template_override))
        self.cfg = cfg

        raw_ros = cfg.raw.get("ros", {}) if isinstance(cfg.raw.get("ros", {}), dict) else {}
        self.color_topic = self.rospy.get_param("~color_topic", raw_ros.get("color_topic", "/camera/color/image_raw"))
        self.depth_topic = self.rospy.get_param(
            "~depth_topic",
            raw_ros.get("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
        )
        self.camera_info_topic = self.rospy.get_param(
            "~camera_info_topic",
            raw_ros.get("camera_info_topic", "/camera/color/camera_info"),
        )
        self.queue_size = int(self.rospy.get_param("~queue_size", raw_ros.get("queue_size", 5)))
        self.slop = float(self.rospy.get_param("~slop", raw_ros.get("slop", 0.08)))
        self.base_frame = self.rospy.get_param("~base_frame", raw_ros.get("base_frame", "base_link"))
        self.camera_frame = self.rospy.get_param(
            "~camera_frame",
            raw_ros.get("camera_frame", "camera_color_optical_frame"),
        )
        self.child_frame = self.rospy.get_param("~child_frame", raw_ros.get("child_frame", "pallet_fork_center"))
        self.publish_tf = bool(self.rospy.get_param("~publish_tf", raw_ros.get("publish_tf", True)))
        self.publish_debug_image = bool(
            self.rospy.get_param("~publish_debug_image", raw_ros.get("publish_debug_image", True))
        )
        self.publish_debug_cloud = bool(
            self.rospy.get_param("~publish_debug_cloud", raw_ros.get("publish_debug_cloud", True))
        )
        self.publish_candidates = bool(
            self.rospy.get_param("~publish_candidates", raw_ros.get("publish_candidates", True))
        )
        self.publish_selected_bbox = bool(
            self.rospy.get_param("~publish_selected_bbox", raw_ros.get("publish_selected_bbox", True))
        )
        self.target_label = self.rospy.get_param(
            "~target_label",
            cfg.raw.get("target_selection", {}).get("target_label", "") if isinstance(cfg.raw.get("target_selection", {}), dict) else "",
        )
        self.fallback_strategy = self.rospy.get_param(
            "~fallback_strategy",
            cfg.raw.get("target_selection", {}).get("fallback_strategy", "error")
            if isinstance(cfg.raw.get("target_selection", {}), dict)
            else "error",
        )
        enable_ocr = self.rospy.get_param(
            "~enable_ocr",
            cfg.raw.get("target_selection", {}).get("enable_ocr", True)
            if isinstance(cfg.raw.get("target_selection", {}), dict)
            else True,
        )
        self.use_camera_info_intrinsics = bool(
            self.rospy.get_param("~use_camera_info_intrinsics", raw_ros.get("use_camera_info_intrinsics", True))
        )
        self.fallback_intrinsics = self._read_fallback_intrinsics(raw_ros)
        process_every_n = int(
            self.rospy.get_param("~process_every_n_frames", raw_ros.get("process_every_n_frames", 1))
        )
        max_fps = float(self.rospy.get_param("~max_fps", cfg.raw.get("runtime", {}).get("max_fps", 5.0)))

        self.pipeline = PalletPoseRealtimePipeline(
            cfg,
            config_path=config_path,
            max_fps=max_fps,
            process_every_n_frames=process_every_n,
            target_label=self.target_label,
            fallback_strategy=self.fallback_strategy,
            enable_ocr=bool(enable_ocr),
        )
        self.tf_broadcaster = self.tf.TransformBroadcaster() if self.publish_tf else None
        self.busy = False

        self.pose_pub = self.rospy.Publisher("/pallet_pose/pose", PoseStamped, queue_size=1)
        self.result_pub = self.rospy.Publisher("/pallet_pose/result", String, queue_size=1)
        self.status_pub = self.rospy.Publisher("/pallet_pose/status", String, queue_size=1)
        self.debug_image_pub = self.rospy.Publisher("/pallet_pose/debug_image", Image, queue_size=1)
        self.roi_cloud_pub = self.rospy.Publisher("/pallet_pose/roi_cloud", PointCloud2, queue_size=1)
        self.target_bbox_pub = self.rospy.Publisher("/pallet_pose/target_bbox", String, queue_size=1)
        self.candidates_pub = self.rospy.Publisher("/pallet_pose/candidates", String, queue_size=1)

        color_sub = self.message_filters.Subscriber(self.color_topic, Image)
        depth_sub = self.message_filters.Subscriber(self.depth_topic, Image)
        info_sub = self.message_filters.Subscriber(self.camera_info_topic, CameraInfo)
        self.sync = self.message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=self.queue_size,
            slop=self.slop,
        )
        self.sync.registerCallback(self.callback)

        self.rospy.loginfo(
            "pallet_pose_ros1_node subscribed to color=%s depth=%s camera_info=%s",
            self.color_topic,
            self.depth_topic,
            self.camera_info_topic,
        )

    def spin(self) -> None:
        self.rospy.spin()

    def callback(self, color_msg: Any, depth_msg: Any, camera_info_msg: Any) -> None:
        if self.busy:
            self.rospy.logwarn_throttle(2.0, "Skipping RGB-D frame because previous pallet pose processing is still running")
            self.status_pub.publish(status_to_json_msg(False, "skipped", "previous frame is still processing"))
            return
        self.busy = True
        try:
            color = ros_image_to_cv2(color_msg, self.bridge, "bgr8")
            depth = ros_image_to_cv2(depth_msg, self.bridge, "passthrough")
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            intrinsics = self._select_intrinsics(camera_info_msg)
            stamp = color_msg.header.stamp
            stamp_float = stamp.to_sec() if hasattr(stamp, "to_sec") else None
            result = self.pipeline.process_rgbd(
                color,
                depth,
                intrinsics,
                timestamp=stamp_float,
                frame_id=self.camera_frame,
            )
            if result.skipped:
                self.rospy.logwarn_throttle(2.0, "Pallet pose frame skipped by rate limiter")
                return
            if not result.success or result.fork_pose is None:
                message = result.error_message or result.status_code
                self.status_pub.publish(status_to_json_msg(False, result.status_code, message, result.processing_time_ms))
                self._publish_selection_debug(result)
                self.rospy.logwarn_throttle(2.0, "Pallet pose failed: %s", message)
                self._publish_failure_debug(color, result, color_msg.header)
                return

            pose_msg = fork_pose_to_pose_stamped(result.fork_pose, self.base_frame, stamp)
            self.pose_pub.publish(pose_msg)
            self.result_pub.publish(
                fork_pose_to_json_msg(
                    result.fork_pose,
                    result.warnings,
                    result.processing_time_ms,
                    success=True,
                    stamp=stamp_float,
                    frame_id=self.base_frame,
                    bbox=result.bbox,
                    target_label=result.target_label,
                    selected_label=result.selected_label,
                    target_bbox=result.target_bbox,
                    selection_status=result.selection_status,
                    candidates=[candidate.to_dict() for candidate in result.candidates],
                    camera_pose=result.camera_pose.to_dict() if result.camera_pose is not None else None,
                )
            )
            self.status_pub.publish(status_to_json_msg(True, result.status_code, "ok", result.processing_time_ms))
            self._publish_selection_debug(result)
            if self.publish_debug_image:
                from .visualization import draw_target_selection

                debug_image = draw_target_selection(
                    color,
                    result.candidates,
                    target_bbox=result.target_bbox,
                    target_label=result.target_label,
                    selection_status=result.selection_status,
                    selected_label=result.selected_label,
                    camera_pose=result.camera_pose,
                    error_message=result.error_message,
                )
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))
            if self.publish_debug_cloud and result.filtered_scene_pcd is not None:
                points = np.asarray(result.filtered_scene_pcd.points)
                self.roi_cloud_pub.publish(numpy_points_to_pointcloud2(points, self.base_frame, stamp))
            if self.tf_broadcaster is not None:
                q = yaw_to_quaternion(result.fork_pose.yaw_rad)
                self.tf_broadcaster.sendTransform(
                    tuple(float(v) for v in result.fork_pose.fork_center),
                    q,
                    stamp,
                    self.child_frame,
                    self.base_frame,
                )
            self.rospy.loginfo_throttle(
                1.0,
                "Pallet pose: distance=%.3f lateral=%.3f height=%.3f yaw=%.2f fitness=%.3f",
                result.camera_pose.x if result.camera_pose is not None else 0.0,
                result.camera_pose.y if result.camera_pose is not None else 0.0,
                result.camera_pose.z if result.camera_pose is not None else 0.0,
                result.camera_pose.yaw_deg if result.camera_pose is not None else 0.0,
                result.fork_pose.fitness,
            )
        except Exception as exc:
            self.status_pub.publish(status_to_json_msg(False, "processing_failed", str(exc)))
            self.rospy.logwarn_throttle(2.0, "Pallet pose callback error: %s", exc)
        finally:
            self.busy = False

    def _select_intrinsics(self, camera_info_msg: Any) -> CameraIntrinsics:
        if self.use_camera_info_intrinsics:
            try:
                return camera_info_to_intrinsics(camera_info_msg)
            except Exception as exc:
                if self.fallback_intrinsics is None:
                    raise ValueError(f"CameraInfo is unavailable or invalid and fallback_intrinsics is not configured: {exc}") from exc
                self.rospy.logwarn_throttle(5.0, "Using fallback camera intrinsics because CameraInfo is invalid: %s", exc)
        if self.fallback_intrinsics is None:
            raise ValueError("use_camera_info_intrinsics is false but fallback_intrinsics is not configured")
        return self.fallback_intrinsics

    def _read_fallback_intrinsics(self, raw_ros: dict[str, Any]) -> CameraIntrinsics | None:
        data = raw_ros.get("fallback_intrinsics", None)
        if not isinstance(data, dict):
            return None
        try:
            return CameraIntrinsics(
                fx=float(data["fx"]),
                fy=float(data["fy"]),
                cx=float(data["cx"]),
                cy=float(data["cy"]),
            )
        except KeyError as exc:
            raise ValueError("ros.fallback_intrinsics must include fx, fy, cx, cy") from exc

    def _publish_failure_debug(self, color: np.ndarray, result: Any, header: Any) -> None:
        if self.publish_debug_image:
            try:
                from .visualization import draw_target_selection

                debug_image = draw_target_selection(
                    color,
                    result.candidates,
                    target_bbox=result.target_bbox,
                    target_label=result.target_label,
                    selection_status=result.selection_status or result.status_code,
                    selected_label=result.selected_label,
                    camera_pose=result.camera_pose,
                    error_message=result.error_message,
                )
            except Exception:
                debug_image = draw_ros_debug_image(color, result.bbox, None, [result.error_message or result.status_code])
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))

    def _publish_selection_debug(self, result: Any) -> None:
        from std_msgs.msg import String

        if self.publish_selected_bbox:
            msg = String()
            msg.data = json.dumps(
                {
                    "success": bool(result.success),
                    "target_label": result.target_label,
                    "selected_label": result.selected_label,
                    "target_bbox": result.target_bbox,
                    "selection_status": result.selection_status,
                    "camera_pose": result.camera_pose.to_dict() if result.camera_pose is not None else None,
                },
                ensure_ascii=False,
            )
            self.target_bbox_pub.publish(msg)
        if self.publish_candidates:
            msg = String()
            msg.data = json.dumps(
                {
                    "selection_status": result.selection_status,
                    "candidates": [candidate.to_dict() for candidate in result.candidates],
                },
                ensure_ascii=False,
            )
            self.candidates_pub.publish(msg)


def main() -> None:
    node = PalletPoseRos1Node()
    node.spin()


if __name__ == "__main__":
    main()
