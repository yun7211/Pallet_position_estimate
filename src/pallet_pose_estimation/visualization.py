"""Visualization helpers for detection and registration debugging."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from .assist_label import PalletCandidate
from .pose_output import CameraPoseResult


def draw_detection(image: np.ndarray, bbox: list[float] | tuple[float, float, float, float]) -> np.ndarray:
    """Draw a pallet-front detection box on a copy of an image."""

    if image is None or image.size == 0:
        raise ValueError("image is empty")
    output = image.copy()
    u_min, v_min, u_max, v_max = [int(round(v)) for v in bbox]
    cv2.rectangle(output, (u_min, v_min), (u_max, v_max), (0, 255, 0), 2)
    cv2.putText(
        output,
        "forkable_front",
        (u_min, max(0, v_min - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return output


def draw_target_selection(
    image: np.ndarray,
    candidates: list[PalletCandidate],
    target_bbox: list[int] | tuple[int, int, int, int] | None = None,
    target_label: str | None = None,
    selection_status: str | None = None,
    selected_label: str | None = None,
    camera_pose: CameraPoseResult | None = None,
    error_message: str | None = None,
) -> np.ndarray:
    """Draw all YOLO candidates, OCR labels, and selected target bbox."""

    if image is None or image.size == 0:
        raise ValueError("image is empty")
    output = image.copy()
    target_tuple = tuple(int(v) for v in target_bbox) if target_bbox is not None else None
    for candidate in candidates:
        x1, y1, x2, y2 = candidate.box_xyxy
        is_target = target_tuple == candidate.box_xyxy
        color = (0, 0, 255) if is_target else (0, 255, 0)
        thickness = 3 if is_target else 2
        cv2.rectangle(output, (x1, y1), (x2, y2), color, thickness)
        label = candidate.label_text if candidate.label_text is not None else "None"
        prefix = "TARGET: " if is_target else ""
        text = f"{prefix}{label} {candidate.confidence:.2f}"
        cv2.putText(output, text, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    status_text = (
        f"target={target_label or 'None'} selected={selected_label or 'None'} "
        f"status={selection_status or 'unknown'}"
    )
    status_color = (0, 0, 255) if selection_status in {"target_label_not_found", "target_label_required"} else (255, 255, 0)
    cv2.putText(output, status_text, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
    if camera_pose is not None:
        pose_text = (
            f"camera xyz=({camera_pose.x:.3f}, {camera_pose.y:.3f}, {camera_pose.z:.3f}) "
            f"yaw={camera_pose.yaw_deg:.2f}"
        )
        cv2.putText(output, pose_text, (16, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    elif error_message:
        cv2.putText(output, error_message[:100], (16, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return output


def draw_yolo_candidates(image: np.ndarray, candidates: list[PalletCandidate]) -> np.ndarray:
    """Draw raw YOLO candidates before target selection emphasis."""

    if image is None or image.size == 0:
        raise ValueError("image is empty")
    output = image.copy()
    for candidate in candidates:
        x1, y1, x2, y2 = candidate.box_xyxy
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            output,
            f"{candidate.class_name} {candidate.confidence:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )
    return output


def visualize_registration(
    template_pcd: o3d.geometry.PointCloud,
    scene_pcd: o3d.geometry.PointCloud,
    T_init: np.ndarray | None = None,
    T_final: np.ndarray | None = None,
) -> None:
    """Open an Open3D window showing scene and aligned template clouds."""

    geometries: list[o3d.geometry.Geometry] = []
    scene = o3d.geometry.PointCloud(scene_pcd)
    scene.paint_uniform_color([0.2, 0.7, 1.0])
    geometries.append(scene)

    if T_init is not None:
        initial = o3d.geometry.PointCloud(template_pcd)
        initial.transform(np.asarray(T_init, dtype=np.float64))
        initial.paint_uniform_color([1.0, 0.75, 0.1])
        geometries.append(initial)
    if T_final is not None:
        final = o3d.geometry.PointCloud(template_pcd)
        final.transform(np.asarray(T_final, dtype=np.float64))
        final.paint_uniform_color([1.0, 0.1, 0.1])
        geometries.append(final)
    if T_init is None and T_final is None:
        template = o3d.geometry.PointCloud(template_pcd)
        template.paint_uniform_color([1.0, 0.1, 0.1])
        geometries.append(template)

    o3d.visualization.draw_geometries(geometries)


def save_debug_point_clouds(
    output_dir: str | Path,
    raw_roi_camera_pcd: o3d.geometry.PointCloud,
    raw_roi_body_pcd: o3d.geometry.PointCloud,
    preprocessed_pcd: o3d.geometry.PointCloud | None,
    ground_removed_pcd: o3d.geometry.PointCloud | None,
    filtered_pcd: o3d.geometry.PointCloud,
    template_pcd: o3d.geometry.PointCloud,
    aligned_template_pcd: o3d.geometry.PointCloud,
) -> None:
    """Save debug point clouds for offline inspection."""

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out / "raw_roi_camera.ply"), raw_roi_camera_pcd)
    o3d.io.write_point_cloud(str(out / "raw_roi_body.ply"), raw_roi_body_pcd)
    if preprocessed_pcd is not None:
        o3d.io.write_point_cloud(str(out / "preprocessed_scene.ply"), preprocessed_pcd)
    if ground_removed_pcd is not None:
        o3d.io.write_point_cloud(str(out / "ground_removed_scene.ply"), ground_removed_pcd)
    o3d.io.write_point_cloud(str(out / "filtered_scene.ply"), filtered_pcd)
    o3d.io.write_point_cloud(str(out / "template_front.ply"), template_pcd)
    o3d.io.write_point_cloud(str(out / "aligned_template_final.ply"), aligned_template_pcd)


def save_label_crop_images(output_dir: str | Path, candidates: list[PalletCandidate]) -> None:
    """Save OCR label crops for each candidate when available."""

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)
    for candidate in candidates:
        if candidate.label_crop is None or candidate.label_crop.size == 0:
            continue
        label = candidate.label_text or "None"
        filename = out / f"candidate_{candidate.candidate_id}_{label}.png"
        cv2.imwrite(str(filename), candidate.label_crop)
