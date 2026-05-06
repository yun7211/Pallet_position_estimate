"""Reusable RGB-D realtime pipeline for pallet forkable-front pose estimation."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from pathlib import Path
import time
from typing import Any, Optional

import numpy as np

from .assist_label import AssistLabelRecognizer, PalletCandidate
from .config import CameraIntrinsics, PalletPoseConfig
from .detector import YOLODetector
from .pose_output import CameraPoseResult, ForkPoseResult, compute_camera_pose, compute_fork_pose
from .rgbd import backproject_depth_to_points, crop_depth_by_bbox
from .target_selector import TargetPalletSelector

LOGGER = logging.getLogger(__name__)


@dataclass
class PipelineResult:
    """Result and debug artifacts from one RGB-D processing attempt."""

    success: bool
    error_message: Optional[str] = None
    bbox: Optional[list[int]] = None
    target_label: Optional[str] = None
    selected_label: Optional[str] = None
    target_bbox: Optional[list[int]] = None
    candidates: list[PalletCandidate] = field(default_factory=list)
    selection_status: str = ""
    camera_pose: Optional[CameraPoseResult] = None
    raw_roi_points: Optional[np.ndarray] = None
    raw_roi_camera_pcd: Any = None
    raw_roi_body_pcd: Any = None
    preprocessed_scene_pcd: Any = None
    ground_removed_scene_pcd: Any = None
    filtered_scene_pcd: Any = None
    template_pcd: Any = None
    T_init: Optional[np.ndarray] = None
    T_final: Optional[np.ndarray] = None
    T_camera: Optional[np.ndarray] = None
    fork_pose: Optional[ForkPoseResult] = None
    warnings: list[str] = field(default_factory=list)
    processing_time_ms: float = 0.0
    skipped: bool = False
    status_code: str = "ok"
    timestamp: Optional[float] = None
    frame_id: str = "camera_color_optical_frame"


def resolve_path(value: str, config_path: str | Path | None = None) -> Path:
    """Resolve a config-relative path without hardcoding project locations."""

    path = Path(value)
    if path.is_absolute() or path.exists():
        return path
    candidates: list[Path] = []
    if config_path is not None:
        cfg = Path(config_path)
        candidates.extend([cfg.parent / path, cfg.parent.parent / path])
    candidates.append(Path.cwd() / path)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[-1]


def resolve_model_path(value: str, config_path: str | Path | None = None) -> str:
    """Resolve a YOLO model path while still allowing Ultralytics model names."""

    path = Path(value)
    if path.is_absolute() or path.exists():
        return str(path)
    if config_path is not None:
        cfg = Path(config_path)
        for candidate in (cfg.parent / path, cfg.parent.parent / path):
            if candidate.exists():
                return str(candidate)
    return value


class PalletPoseRealtimePipeline:
    """Shared runtime pipeline used by offline CLI, RealSense direct mode, and ROS1.

    The color image is expected to be an OpenCV BGR image, matching the offline
    CLI path and Ultralytics' accepted ndarray input convention. Depth must be
    aligned to the color image before this class receives it; otherwise the YOLO
    bbox and depth ROI will refer to different pixel coordinates.
    """

    def __init__(
        self,
        cfg: PalletPoseConfig,
        config_path: str | Path | None = None,
        debug_output_dir: str | Path | None = None,
        detector: YOLODetector | None = None,
        template_pcd: Any = None,
        max_fps: float | None = None,
        process_every_n_frames: int | None = None,
        save_debug: bool | None = None,
        target_label: str | None = None,
        fallback_strategy: str | None = None,
        enable_ocr: bool | None = None,
        target_selector: TargetPalletSelector | None = None,
        save_label_crops: bool | None = None,
    ) -> None:
        self.cfg = cfg
        self.config_path = Path(config_path) if config_path is not None else None
        self.max_fps = self._read_float("runtime", "max_fps", default=max_fps)
        self.process_every_n_frames = int(
            self._read_nested("runtime", "process_every_n_frames", default=process_every_n_frames or 1)
        )
        if self.process_every_n_frames < 1:
            raise ValueError("runtime.process_every_n_frames must be >= 1")
        self.save_debug = bool(self._read_nested("runtime", "save_debug", default=bool(save_debug)))
        self.debug_output_dir = Path(debug_output_dir or self._read_nested("runtime", "debug_output_dir", default=""))
        self.save_label_crops = bool(
            self._read_nested("target_selection", "save_label_crops", default=bool(save_label_crops))
        )
        self._frame_count = 0
        self._last_process_time = 0.0
        self.require_aligned_depth_to_color = bool(
            self._read_nested("rgbd", "require_aligned_depth_to_color", default=True)
        )

        if self.cfg.extrinsic_T_B_C.shape != (4, 4):
            raise ValueError("extrinsic_T_B_C is required and must be 4x4 for body-frame output")

        self.detector = detector or YOLODetector(
            resolve_model_path(cfg.yolo_model_path, self.config_path),
            class_name=cfg.yolo_class_name,
            confidence_threshold=cfg.confidence_threshold,
        )
        target_cfg = self._target_selection_cfg()
        effective_target_label = target_label if target_label is not None else target_cfg.get("target_label")
        effective_fallback_strategy = fallback_strategy or target_cfg.get("fallback_strategy", "error")
        effective_enable_ocr = bool(target_cfg.get("enable_ocr", True) if enable_ocr is None else enable_ocr)
        enable_assist_label = bool(target_cfg.get("enable_assist_label", True))
        self.label_recognizer = AssistLabelRecognizer(
            pillar_width=int(target_cfg.get("pillar_width", 100)),
            pillar_height=int(target_cfg.get("pillar_height", 100)),
            hole_width=int(target_cfg.get("hole_width", 200)),
            ocr_languages=list(target_cfg.get("ocr_languages", ["en"])),
            use_gpu=bool(target_cfg.get("ocr_use_gpu", True)),
            enable_ocr=effective_enable_ocr,
        )
        self.target_selector = target_selector or TargetPalletSelector(
            detector=self.detector,
            label_recognizer=self.label_recognizer,
            target_label=effective_target_label,
            fallback_strategy=str(effective_fallback_strategy or "error"),
            min_detection_confidence=float(target_cfg.get("min_detection_confidence", cfg.confidence_threshold)),
            label_match_mode=str(target_cfg.get("label_match_mode", "exact")),
            enable_assist_label=enable_assist_label,
        )

        if template_pcd is None:
            from .registration import load_template_point_cloud

            template_pcd = load_template_point_cloud(resolve_path(cfg.template_point_cloud_path, self.config_path))
        self.template_pcd = template_pcd

    def process_rgbd(
        self,
        color_image: np.ndarray,
        depth_image: np.ndarray,
        intrinsics: CameraIntrinsics,
        timestamp: Optional[float] = None,
        frame_id: str = "camera_color_optical_frame",
    ) -> PipelineResult:
        """Process one aligned RGB-D pair and return a non-throwing result."""

        start = time.perf_counter()
        self._frame_count += 1
        if self._should_skip_frame(start):
            return PipelineResult(
                success=False,
                error_message="frame skipped by realtime rate limiter",
                warnings=["frame skipped by realtime rate limiter"],
                processing_time_ms=0.0,
                skipped=True,
                status_code="skipped",
                timestamp=timestamp,
                frame_id=frame_id,
                selection_status="skipped",
            )

        bbox: list[int] | None = None
        target_label = self.target_selector.target_label
        selected_label: str | None = None
        candidates: list[PalletCandidate] = []
        selection_status = ""
        raw_points: np.ndarray | None = None
        raw_roi_camera_pcd = None
        raw_roi_body_pcd = None
        preprocessed = None
        ground_removed = None
        filtered = None
        T_init = None
        T_final = None
        T_camera = None
        fork_pose = None
        camera_pose = None
        warnings: list[str] = []
        try:
            from .preprocessing import (
                filter_by_height_and_roi,
                numpy_to_o3d,
                preprocess_point_cloud,
                remove_ground_ransac,
                transform_point_cloud,
            )
            from .registration import register_template_to_scene

            if color_image is None or color_image.size == 0:
                raise ValueError("color image is empty")
            if depth_image is None or depth_image.ndim != 2:
                raise ValueError("aligned depth image must be a single-channel HxW array")
            if color_image.shape[:2] != depth_image.shape[:2]:
                raise ValueError(
                    "color image and aligned depth image shapes differ; use depth aligned to color"
                )
            if self.require_aligned_depth_to_color is not True:
                raise ValueError("rgbd.require_aligned_depth_to_color must be true for target bbox depth ROI")

            selection = self.target_selector.select_target(color_image)
            candidates = selection.candidates
            selection_status = selection.status
            if not selection.success or selection.target_bbox is None:
                elapsed_ms = (time.perf_counter() - start) * 1000.0
                return PipelineResult(
                    success=False,
                    error_message=selection.status,
                    bbox=None,
                    target_label=selection.target_label,
                    selected_label=None,
                    target_bbox=None,
                    candidates=candidates,
                    selection_status=selection.status,
                    warnings=selection.warnings,
                    processing_time_ms=elapsed_ms,
                    status_code=selection.status,
                    timestamp=timestamp,
                    frame_id=frame_id,
                )

            detected_bbox = selection.target_bbox
            bbox = [int(v) for v in detected_bbox]
            selected_label = selection.matched_candidate.label_text if selection.matched_candidate else None
            depth_roi = crop_depth_by_bbox(depth_image, detected_bbox)
            raw_points = backproject_depth_to_points(
                depth_roi,
                detected_bbox,
                intrinsics,
                self.cfg.depth_scale,
                self.cfg.depth_min,
                self.cfg.depth_max,
            )
            if len(raw_points) == 0:
                raise ValueError("empty_roi_cloud: no valid depth points inside pallet-front ROI")

            raw_roi_camera_pcd = numpy_to_o3d(raw_points)
            raw_roi_body_pcd = transform_point_cloud(raw_roi_camera_pcd, self.cfg.extrinsic_T_B_C)
            preprocessed = preprocess_point_cloud(raw_roi_body_pcd, self.cfg.voxel_size)
            ground_removed, _ = remove_ground_ransac(
                preprocessed,
                distance_threshold=self.cfg.voxel_size * self.cfg.ransac_distance_threshold_factor,
            )
            filtered = filter_by_height_and_roi(ground_removed, **self._optional_filter_kwargs())
            if len(filtered.points) == 0:
                raise ValueError("empty_roi_cloud: scene point cloud is empty after preprocessing")

            registration = register_template_to_scene(self.template_pcd, filtered, self.cfg)
            T_init = registration.T_init
            T_final = registration.T_final
            warnings.extend(registration.warnings)
            fork_pose = compute_fork_pose(
                registration.T_final,
                self.cfg.template_fork_center,
                self.cfg.template_insert_axis,
                fitness=registration.fitness,
                inlier_rmse=registration.inlier_rmse,
                warnings=registration.warnings,
            )
            T_camera = np.linalg.inv(self.cfg.extrinsic_T_B_C) @ registration.T_final
            camera_pose = compute_camera_pose(
                T_camera,
                self.cfg.template_fork_center,
                self.cfg.template_insert_axis,
            )
            self._last_process_time = start
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            result = PipelineResult(
                success=True,
                bbox=bbox,
                target_label=selection.target_label,
                selected_label=selected_label,
                target_bbox=bbox,
                candidates=candidates,
                selection_status=selection.status,
                camera_pose=camera_pose,
                raw_roi_points=raw_points,
                raw_roi_camera_pcd=raw_roi_camera_pcd,
                raw_roi_body_pcd=raw_roi_body_pcd,
                preprocessed_scene_pcd=preprocessed,
                ground_removed_scene_pcd=ground_removed,
                filtered_scene_pcd=filtered,
                template_pcd=self.template_pcd,
                T_init=T_init,
                T_final=T_final,
                T_camera=T_camera,
                fork_pose=fork_pose,
                warnings=warnings,
                processing_time_ms=elapsed_ms,
                status_code="ok" if not warnings else "registration_unreliable",
                timestamp=timestamp,
                frame_id=frame_id,
            )
            self._save_debug_outputs(color_image, result)
            return result
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            status = self._classify_error(exc)
            LOGGER.debug("Realtime pallet pose processing failed", exc_info=True)
            result = PipelineResult(
                success=False,
                error_message=str(exc),
                bbox=bbox,
                target_label=target_label,
                selected_label=selected_label,
                target_bbox=bbox,
                candidates=candidates,
                selection_status=selection_status,
                camera_pose=camera_pose,
                raw_roi_points=raw_points,
                raw_roi_camera_pcd=raw_roi_camera_pcd,
                raw_roi_body_pcd=raw_roi_body_pcd,
                preprocessed_scene_pcd=preprocessed,
                ground_removed_scene_pcd=ground_removed,
                filtered_scene_pcd=filtered,
                template_pcd=self.template_pcd,
                T_init=T_init,
                T_final=T_final,
                T_camera=T_camera,
                fork_pose=fork_pose,
                warnings=warnings,
                processing_time_ms=elapsed_ms,
                status_code=status,
                timestamp=timestamp,
                frame_id=frame_id,
            )
            self._save_debug_outputs(color_image, result)
            return result

    def _should_skip_frame(self, now: float) -> bool:
        if self.process_every_n_frames > 1 and (self._frame_count - 1) % self.process_every_n_frames != 0:
            return True
        if self.max_fps is not None and self.max_fps > 0:
            min_interval = 1.0 / self.max_fps
            if self._last_process_time > 0 and now - self._last_process_time < min_interval:
                return True
        return False

    def _optional_filter_kwargs(self) -> dict[str, float | None]:
        keys = ("z_min", "z_max", "y_min", "y_max")
        return {
            key: (float(self.cfg.raw[key]) if key in self.cfg.raw and self.cfg.raw[key] is not None else None)
            for key in keys
        }

    def _read_nested(self, group: str, key: str, default: Any = None) -> Any:
        section = self.cfg.raw.get(group, {})
        if isinstance(section, dict) and key in section:
            return section[key]
        return default

    def _read_float(self, group: str, key: str, default: float | None = None) -> float | None:
        value = self._read_nested(group, key, default=default)
        return None if value is None else float(value)

    def _target_selection_cfg(self) -> dict[str, Any]:
        section = self.cfg.raw.get("target_selection", {})
        return section if isinstance(section, dict) else {}

    def _save_debug_outputs(self, color_image: np.ndarray, result: PipelineResult) -> None:
        if not self.save_debug or color_image is None or color_image.size == 0:
            return
        import cv2

        from .preprocessing import transform_point_cloud
        from .visualization import (
            draw_target_selection,
            draw_yolo_candidates,
            save_debug_point_clouds,
            save_label_crop_images,
        )

        self.debug_output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.debug_output_dir / "yolo_candidates.png"), draw_yolo_candidates(color_image, result.candidates))
        cv2.imwrite(
            str(self.debug_output_dir / "target_selection.png"),
            draw_target_selection(
                color_image,
                result.candidates,
                target_bbox=result.target_bbox,
                target_label=result.target_label,
                selection_status=result.selection_status,
                selected_label=result.selected_label,
                camera_pose=result.camera_pose,
                error_message=result.error_message,
            ),
        )
        if self.save_label_crops:
            save_label_crop_images(self.debug_output_dir / "label_crops", result.candidates)
        if result.T_final is None or result.filtered_scene_pcd is None or result.raw_roi_body_pcd is None:
            return
        aligned_template = transform_point_cloud(self.template_pcd, result.T_final)
        save_debug_point_clouds(
            self.debug_output_dir,
            result.raw_roi_camera_pcd,
            result.raw_roi_body_pcd,
            result.preprocessed_scene_pcd,
            result.ground_removed_scene_pcd,
            result.filtered_scene_pcd,
            self.template_pcd,
            aligned_template,
        )

    @staticmethod
    def _classify_error(exc: Exception) -> str:
        message = str(exc).lower()
        name = exc.__class__.__name__.lower()
        if "nodetection" in name or "no detection" in message:
            return "no_detection"
        if "target_label_not_found" in message or "target_label_required" in message:
            return message
        if "empty_roi_cloud" in message or "no valid depth points" in message:
            return "empty_roi_cloud"
        if "global registration" in message or "ransac" in message or "fpfh" in message:
            return "global_registration_failed"
        if "registration" in message or "icp" in message or "fitness" in message:
            return "registration_unreliable"
        return "processing_failed"
