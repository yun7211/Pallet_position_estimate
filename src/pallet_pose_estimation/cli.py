"""Command-line entry point for offline pallet-front pose estimation."""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Any

from .config import load_config
from .realtime_pipeline import PalletPoseRealtimePipeline

LOGGER = logging.getLogger(__name__)


def _read_rgb(path: Path) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(f"RGB image not found or unreadable: {path}")
    return image


def _read_depth(path: Path) -> np.ndarray:
    import cv2

    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(f"Depth image not found or unreadable: {path}")
    if depth.ndim == 3:
        depth = depth[:, :, 0]
    return depth


def _optional_filter_kwargs(raw_cfg: dict[str, Any]) -> dict[str, float | None]:
    keys = ("z_min", "z_max", "y_min", "y_max")
    return {key: (float(raw_cfg[key]) if key in raw_cfg and raw_cfg[key] is not None else None) for key in keys}


def run_pipeline(args: argparse.Namespace) -> dict[str, Any]:
    """Execute the complete offline pipeline and return a JSON-ready result."""

    import cv2

    config_path = Path(args.config)
    cfg = load_config(config_path)
    rgb_path = Path(args.rgb)
    depth_path = Path(args.depth)
    output_path = Path(args.output)
    debug_dir = Path(args.save_dir) if args.save_dir else output_path.parent / "debug"

    rgb = _read_rgb(rgb_path)
    depth = _read_depth(depth_path)
    pipeline = PalletPoseRealtimePipeline(
        cfg,
        config_path=config_path,
        debug_output_dir=debug_dir,
        save_debug=args.debug_vis or bool(args.save_dir),
        target_label=args.target_label,
        fallback_strategy=args.fallback_strategy,
        enable_ocr=False if args.disable_ocr else None,
        save_label_crops=args.save_label_crops,
    )
    result = pipeline.process_rgbd(
        rgb,
        depth,
        cfg.camera_intrinsics,
        frame_id=str(cfg.raw.get("camera_frame", "camera_color_optical_frame")),
    )
    if not result.success or result.fork_pose is None:
        raise RuntimeError(result.error_message or result.status_code)

    output = {
        "success": True,
        "x": result.camera_pose.x if result.camera_pose is not None else None,
        "y": result.camera_pose.y if result.camera_pose is not None else None,
        "z": result.camera_pose.z if result.camera_pose is not None else None,
        "yaw_rad": result.camera_pose.yaw_rad if result.camera_pose is not None else None,
        "yaw_deg": result.camera_pose.yaw_deg if result.camera_pose is not None else None,
        "camera_pose": result.camera_pose.to_dict() if result.camera_pose is not None else None,
        "body_pose": result.fork_pose.to_dict(),
        "T_init": result.T_init.tolist() if result.T_init is not None else None,
        "T_final": result.T_final.tolist() if result.T_final is not None else None,
        "T_camera": result.T_camera.tolist() if result.T_camera is not None else None,
        "fitness": result.fork_pose.fitness,
        "inlier_rmse": result.fork_pose.inlier_rmse,
        "warnings": list(result.fork_pose.warnings),
    }
    output["bbox"] = result.bbox
    output["target_label"] = result.target_label
    output["selected_label"] = result.selected_label
    output["target_bbox"] = result.target_bbox
    output["selection_status"] = result.selection_status
    output["candidates"] = [candidate.to_dict() for candidate in result.candidates]
    output["processing_time_ms"] = result.processing_time_ms

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)

    if args.debug_vis and result.bbox is not None:
        debug_dir.mkdir(parents=True, exist_ok=True)
        from .visualization import draw_target_selection, visualize_registration

        cv2.imwrite(
            str(debug_dir / "result_overlay.png"),
            draw_target_selection(
                rgb,
                result.candidates,
                target_bbox=result.target_bbox,
                target_label=result.target_label,
                selection_status=result.selection_status,
                selected_label=result.selected_label,
                camera_pose=result.camera_pose,
            ),
        )
        if not args.no_window:
            visualize_registration(result.template_pcd, result.filtered_scene_pcd, result.T_init, result.T_final)

    return output


def build_arg_parser() -> argparse.ArgumentParser:
    """Build the CLI argument parser."""

    parser = argparse.ArgumentParser(
        description="Estimate the local forkable front-surface pose of a pallet from RGB-D data."
    )
    parser.add_argument("--config", required=True, help="Path to YAML config, e.g. config/default.yaml")
    parser.add_argument("--rgb", required=True, help="Path to RGB image")
    parser.add_argument("--depth", required=True, help="Path to depth image")
    parser.add_argument("--output", required=True, help="Path to output result.json")
    parser.add_argument("--target-label", default=None, help="Auxiliary label to bind the target pallet, e.g. A102")
    parser.add_argument(
        "--fallback-strategy",
        default=None,
        choices=["error", "highest_confidence", "center_nearest", "first_valid_label"],
        help="Selection strategy when --target-label is empty",
    )
    parser.add_argument("--disable-ocr", action="store_true", help="Disable EasyOCR and rely on fallback strategy")
    parser.add_argument("--save-label-crops", action="store_true", help="Reserved for saving OCR label crops in debug workflows")
    parser.add_argument("--save-dir", default=None, help="Directory for intermediate detection and point-cloud artifacts")
    parser.add_argument("--debug-vis", action="store_true", help="Save debug clouds and detection image")
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="With --debug-vis, skip the interactive Open3D visualization window",
    )
    parser.add_argument("--log-level", default="INFO", help="Python logging level")
    return parser


def main() -> None:
    """CLI main."""

    parser = build_arg_parser()
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    try:
        result = run_pipeline(args)
    except Exception as exc:
        raise SystemExit(f"pallet pose estimation failed: {exc}") from exc
    LOGGER.info("Wrote result to %s", args.output)
    if result.get("warnings"):
        LOGGER.warning("Registration warnings: %s", result["warnings"])


if __name__ == "__main__":
    main()
