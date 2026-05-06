"""FPFH-RANSAC and ICP/GICP registration for pallet-front templates."""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from pathlib import Path

import numpy as np
import open3d as o3d

from .config import PalletPoseConfig

LOGGER = logging.getLogger(__name__)


@dataclass
class RegistrationOutput:
    """Template-to-scene registration result."""

    T_init: np.ndarray
    T_final: np.ndarray
    fitness: float
    inlier_rmse: float
    warnings: list[str] = field(default_factory=list)


def load_template_point_cloud(path: str | Path) -> o3d.geometry.PointCloud:
    """Load the local forkable-front template point cloud."""

    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"Template point cloud not found: {p}. Provide a front-surface template "
            "containing the forkable face, uprights, fork-hole boundaries, and beams."
        )
    pcd = o3d.io.read_point_cloud(str(p))
    if len(pcd.points) == 0:
        raise ValueError(f"Template point cloud is empty or unreadable: {p}")
    return pcd


def downsample_and_estimate_normals(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
    normal_radius: float,
) -> o3d.geometry.PointCloud:
    """Voxel-downsample a cloud and estimate normals."""

    if voxel_size <= 0 or normal_radius <= 0:
        raise ValueError("voxel_size and normal_radius must be positive")
    if len(pcd.points) == 0:
        raise ValueError("Cannot downsample an empty point cloud")
    pcd_down = pcd.voxel_down_sample(voxel_size)
    if len(pcd_down.points) == 0:
        raise ValueError("Voxel downsampling produced an empty point cloud")
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30)
    )
    return pcd_down


def compute_fpfh(pcd_down: o3d.geometry.PointCloud, radius: float) -> o3d.pipelines.registration.Feature:
    """Compute FPFH features for a downsampled point cloud."""

    if radius <= 0:
        raise ValueError("FPFH radius must be positive")
    if len(pcd_down.points) == 0:
        raise ValueError("Cannot compute FPFH for an empty point cloud")
    if not pcd_down.has_normals():
        raise ValueError("Point cloud must have normals before FPFH computation")
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100),
    )


def execute_global_registration_fpfh(
    source_down: o3d.geometry.PointCloud,
    target_down: o3d.geometry.PointCloud,
    source_fpfh: o3d.pipelines.registration.Feature,
    target_fpfh: o3d.pipelines.registration.Feature,
    voxel_size: float,
    distance_threshold_factor: float = 1.5,
) -> o3d.pipelines.registration.RegistrationResult:
    """Run FPFH feature matching plus RANSAC for coarse registration."""

    if len(source_down.points) < 4 or len(target_down.points) < 4:
        raise ValueError("Global registration needs at least 4 downsampled points in both clouds")
    distance_threshold = voxel_size * distance_threshold_factor
    estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint(False)
    checkers = [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold),
    ]
    criteria = o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    try:
        return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down,
            target_down,
            source_fpfh,
            target_fpfh,
            True,
            distance_threshold,
            estimation,
            4,
            checkers,
            criteria,
        )
    except TypeError:
        return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down,
            target_down,
            source_fpfh,
            target_fpfh,
            distance_threshold,
            estimation,
            4,
            checkers,
            criteria,
        )


def _copy_with_normals(pcd: o3d.geometry.PointCloud, radius: float) -> o3d.geometry.PointCloud:
    copied = o3d.geometry.PointCloud(pcd)
    if not copied.has_normals():
        copied.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
    return copied


def refine_registration(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    init_transform: np.ndarray,
    voxel_size: float,
    method: str = "gicp",
    max_correspondence_distance_factor: float = 1.5,
) -> o3d.pipelines.registration.RegistrationResult:
    """Refine coarse alignment with GICP when available, otherwise point-to-plane ICP."""

    init = np.asarray(init_transform, dtype=np.float64)
    if init.shape != (4, 4):
        raise ValueError(f"init_transform must be 4x4, got {init.shape}")
    max_correspondence_distance = voxel_size * max_correspondence_distance_factor
    normal_radius = voxel_size * 2.0
    source_for_icp = _copy_with_normals(source, normal_radius)
    target_for_icp = _copy_with_normals(target, normal_radius)

    method = method.lower()
    if method == "gicp":
        gicp = getattr(o3d.pipelines.registration, "registration_generalized_icp", None)
        estimation_cls = getattr(
            o3d.pipelines.registration,
            "TransformationEstimationForGeneralizedICP",
            None,
        )
        if gicp is not None and estimation_cls is not None:
            return gicp(
                source_for_icp,
                target_for_icp,
                max_correspondence_distance,
                init,
                estimation_cls(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80),
            )
        LOGGER.warning("Open3D generalized ICP is unavailable; falling back to point-to-plane ICP")
    elif method != "point_to_plane":
        raise ValueError("method must be either 'gicp' or 'point_to_plane'")

    return o3d.pipelines.registration.registration_icp(
        source_for_icp,
        target_for_icp,
        max_correspondence_distance,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80),
    )


def validate_registration_result(
    T_final: np.ndarray,
    fitness: float,
    inlier_rmse: float,
    cfg: PalletPoseConfig,
) -> list[str]:
    """Run geometric sanity checks for the final pallet-front pose."""

    warnings: list[str] = []
    transform = np.asarray(T_final, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError(f"T_final must be a 4x4 homogeneous matrix, got {transform.shape}")
    if not np.all(np.isfinite(transform)):
        raise ValueError("T_final contains NaN or infinite values")
    if not np.allclose(transform[3], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-6):
        raise ValueError("T_final must have homogeneous last row [0, 0, 0, 1]")

    axis = cfg.template_insert_axis.astype(np.float64)
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("template_insert_axis must be non-zero")
    d_insert = transform[:3, :3] @ (axis / axis_norm)
    d_norm = np.linalg.norm(d_insert)
    if d_norm == 0:
        warnings.append("Computed insertion direction has zero length")
    else:
        d_insert = d_insert / d_norm
        if d_insert[0] < -0.2:
            warnings.append(
                "Insertion direction points clearly opposite the AGV forward axis; check template axis or registration."
            )

    fork_center_h = np.r_[cfg.template_fork_center.astype(np.float64), 1.0]
    fork_center = (transform @ fork_center_h)[:3]
    if fork_center[0] < cfg.min_fork_distance or fork_center[0] > cfg.max_fork_distance:
        warnings.append(
            f"Fork center distance {fork_center[0]:.3f} m is outside "
            f"[{cfg.min_fork_distance:.3f}, {cfg.max_fork_distance:.3f}] m."
        )

    if fitness < cfg.min_registration_fitness:
        warnings.append(
            f"Registration fitness {fitness:.3f} is below configured threshold "
            f"{cfg.min_registration_fitness:.3f}."
        )
    rmse_threshold = cfg.voxel_size * cfg.max_registration_rmse_factor
    if inlier_rmse > rmse_threshold:
        warnings.append(
            f"Registration inlier RMSE {inlier_rmse:.4f} m exceeds threshold {rmse_threshold:.4f} m."
        )
    return warnings


def register_template_to_scene(
    template_pcd: o3d.geometry.PointCloud,
    scene_pcd: o3d.geometry.PointCloud,
    cfg: PalletPoseConfig,
) -> RegistrationOutput:
    """Register front-surface template ``P_s`` to scene ROI cloud ``P_t``."""

    if len(template_pcd.points) == 0:
        raise ValueError("template_pcd is empty")
    if len(scene_pcd.points) == 0:
        raise ValueError("scene_pcd is empty after ROI preprocessing")

    voxel_size = cfg.voxel_size
    normal_radius = voxel_size * cfg.normal_radius_factor
    fpfh_radius = voxel_size * cfg.fpfh_radius_factor

    source_down = downsample_and_estimate_normals(template_pcd, voxel_size, normal_radius)
    target_down = downsample_and_estimate_normals(scene_pcd, voxel_size, normal_radius)
    source_fpfh = compute_fpfh(source_down, fpfh_radius)
    target_fpfh = compute_fpfh(target_down, fpfh_radius)

    result_ransac = execute_global_registration_fpfh(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        voxel_size,
        distance_threshold_factor=cfg.ransac_distance_threshold_factor,
    )
    result_refined = refine_registration(
        template_pcd,
        scene_pcd,
        result_ransac.transformation,
        voxel_size,
        method=cfg.icp_method,
        max_correspondence_distance_factor=cfg.icp_max_correspondence_distance_factor,
    )
    warnings = validate_registration_result(
        result_refined.transformation,
        float(result_refined.fitness),
        float(result_refined.inlier_rmse),
        cfg,
    )
    return RegistrationOutput(
        T_init=np.asarray(result_ransac.transformation, dtype=np.float64),
        T_final=np.asarray(result_refined.transformation, dtype=np.float64),
        fitness=float(result_refined.fitness),
        inlier_rmse=float(result_refined.inlier_rmse),
        warnings=warnings,
    )
