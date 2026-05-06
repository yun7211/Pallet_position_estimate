"""Direct Intel RealSense D435 RGB-D runtime entry point without ROS."""

from __future__ import annotations

import argparse
from dataclasses import replace
import logging
from pathlib import Path
import time

import numpy as np

from .config import CameraIntrinsics, load_config
from .realtime_pipeline import PalletPoseRealtimePipeline

LOGGER = logging.getLogger(__name__)


def _import_realsense():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise ImportError(
            "pyrealsense2 is required for RealSense direct mode. Install it with "
            "`pip install pyrealsense2` in a Python version supported by Intel RealSense."
        ) from exc
    return rs


def _read_runtime_bool(raw: dict, section: str, key: str, default: bool) -> bool:
    value = raw.get(section, {}).get(key, default) if isinstance(raw.get(section, {}), dict) else default
    return bool(value)


def _read_runtime_number(raw: dict, section: str, key: str, default: float | int) -> float:
    value = raw.get(section, {}).get(key, default) if isinstance(raw.get(section, {}), dict) else default
    return float(value)


def _intrinsics_from_rs(color_frame) -> CameraIntrinsics:
    profile = color_frame.profile.as_video_stream_profile()
    intr = profile.intrinsics
    return CameraIntrinsics(fx=float(intr.fx), fy=float(intr.fy), cx=float(intr.ppx), cy=float(intr.ppy))


def _configure_auto_exposure(rs, profile, enable: bool) -> None:
    device = profile.get_device()
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.enable_auto_exposure):
            sensor.set_option(rs.option.enable_auto_exposure, 1.0 if enable else 0.0)


def _depth_scale_from_sensor(rs, profile, cfg_raw: dict) -> float:
    depth_cfg = cfg_raw.get("realsense", {}).get("depth_scale", "auto")
    if isinstance(depth_cfg, str) and depth_cfg.lower() == "auto":
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_units_m = float(depth_sensor.get_depth_scale())
        if depth_units_m <= 0:
            raise RuntimeError("RealSense depth sensor reported a non-positive depth scale")
        return 1.0 / depth_units_m
    fixed_scale = float(depth_cfg)
    if fixed_scale <= 0:
        raise ValueError("realsense.depth_scale must be 'auto' or a positive raw-to-meter divisor")
    return fixed_scale


def _print_pose(result) -> None:
    camera_pose = result.camera_pose
    if camera_pose is None:
        print(f"[{result.status_code}] {result.error_message}")
        return
    print(
        "selected_label={} target_bbox={} camera_xyz=({:.3f}, {:.3f}, {:.3f}) yaw={:.2f} deg "
        "fitness={:.3f} rmse={:.4f} ms={:.1f}".format(
            result.selected_label,
            result.target_bbox,
            camera_pose.x,
            camera_pose.y,
            camera_pose.z,
            camera_pose.yaw_deg,
            result.fork_pose.fitness if result.fork_pose is not None else 0.0,
            result.fork_pose.inlier_rmse if result.fork_pose is not None else 0.0,
            result.processing_time_ms,
        )
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run pallet-front pose estimation directly from RealSense D435.")
    parser.add_argument("--config", required=True, help="Path to config/realsense.yaml")
    parser.add_argument("--show", action="store_true", help="Show live debug image window")
    parser.add_argument("--save-debug", default=None, help="Directory for debug images and point clouds")
    parser.add_argument("--target-label", default=None, help="Auxiliary target label, e.g. A102")
    parser.add_argument(
        "--fallback-strategy",
        default=None,
        choices=["error", "highest_confidence", "center_nearest", "first_valid_label"],
        help="Selection strategy when no target label is configured",
    )
    parser.add_argument("--disable-ocr", action="store_true", help="Disable EasyOCR and use configured fallback")
    parser.add_argument("--log-level", default="INFO")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    rs = _import_realsense()
    import cv2

    config_path = Path(args.config)
    cfg = load_config(config_path)
    raw = cfg.raw
    rs_cfg = raw.get("realsense", {}) if isinstance(raw.get("realsense", {}), dict) else {}
    width = int(rs_cfg.get("width", 640))
    height = int(rs_cfg.get("height", 480))
    fps = int(rs_cfg.get("fps", 30))
    align_depth = bool(rs_cfg.get("align_depth_to_color", True))
    auto_exposure = bool(rs_cfg.get("enable_auto_exposure", True))

    pipeline = rs.pipeline()
    config = rs.config()
    try:
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = pipeline.start(config)
    except Exception as exc:
        raise SystemExit(
            "Failed to start RealSense D435. Check that the camera is connected, "
            "not used by another process, and visible to librealsense."
        ) from exc

    try:
        _configure_auto_exposure(rs, profile, auto_exposure)
        depth_scale = _depth_scale_from_sensor(rs, profile, raw)
        cfg = replace(cfg, depth_scale=depth_scale)
        LOGGER.info("Using depth_scale divisor %.3f (raw depth / divisor = meters)", depth_scale)

        show_window = args.show or _read_runtime_bool(raw, "runtime", "show_window", False)
        save_debug = bool(args.save_debug) or _read_runtime_bool(raw, "runtime", "save_debug", False)
        debug_dir = args.save_debug or raw.get("runtime", {}).get("debug_output_dir", "outputs/realsense_debug")
        pose_pipeline = PalletPoseRealtimePipeline(
            cfg,
            config_path=config_path,
            debug_output_dir=debug_dir,
            save_debug=save_debug,
            max_fps=_read_runtime_number(raw, "runtime", "max_fps", 5.0),
            target_label=args.target_label,
            fallback_strategy=args.fallback_strategy,
            enable_ocr=False if args.disable_ocr else None,
        )
        align = rs.align(rs.stream.color) if align_depth else None
        if not align_depth:
            LOGGER.warning("Depth alignment is disabled; ROI back-projection can be invalid unless streams are registered.")

        while True:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames) if align is not None else frames
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                LOGGER.warning("Skipping frame because color or aligned depth frame is missing")
                continue

            # Color stream is configured as bgr8 so the ndarray is OpenCV BGR,
            # which is the convention used by the offline CLI and debug drawing.
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            intrinsics = _intrinsics_from_rs(color_frame)
            result = pose_pipeline.process_rgbd(
                color_image,
                depth_image,
                intrinsics,
                timestamp=time.time(),
                frame_id=str(raw.get("camera_frame", "camera_color_optical_frame")),
            )
            if not result.skipped:
                _print_pose(result)

            if show_window:
                from .visualization import draw_target_selection

                debug = draw_target_selection(
                    color_image,
                    result.candidates,
                    target_bbox=result.target_bbox,
                    target_label=result.target_label,
                    selection_status=result.selection_status,
                    selected_label=result.selected_label,
                    camera_pose=result.camera_pose,
                    error_message=result.error_message,
                )
                cv2.imshow("pallet_pose_realsense", debug)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        pipeline.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
