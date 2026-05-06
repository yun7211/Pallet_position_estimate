# 模块说明

## config.py

读取 YAML 配置并转换为 `PalletPoseConfig`。核心对象：

- `CameraIntrinsics`
- `PalletPoseConfig`
- `load_config(path)`
- `config_from_dict(data)`

初始化阶段会检查内参、外参矩阵、模板轴、ICP 方法等基础参数。

## detector.py

封装 Ultralytics YOLO 检测。目标类别必须是 `pallet_front` 或 `forkable_front` 等可叉取前表面类别。

主要接口：

- `YOLODetector.detect_pallet_front(image) -> bbox`
- `NoDetectionError`

注意：本模块不把完整托盘外框作为默认目标。

## rgbd.py

负责深度 ROI 裁剪和反投影。

主要接口：

- `crop_depth_by_bbox(depth, bbox)`
- `backproject_depth_to_points(depth_roi, bbox, intrinsics, depth_scale, depth_min, depth_max)`

深度单位通过 `depth_scale` 控制：`Z = raw_depth / depth_scale`。

## preprocessing.py

负责点云基础处理。

主要接口：

- `numpy_to_o3d(points)`
- `transform_point_cloud(pcd, T)`
- `preprocess_point_cloud(pcd, voxel_size, nb_neighbors, std_ratio)`
- `remove_ground_ransac(...)`
- `filter_by_height_and_roi(...)`

点云在反投影后先位于相机坐标系，经 `T_B_C` 转换到 AGV 车体坐标系。

## registration.py

负责模板点云到现场 ROI 点云的配准。

主要接口：

- `load_template_point_cloud(path)`
- `downsample_and_estimate_normals(...)`
- `compute_fpfh(...)`
- `execute_global_registration_fpfh(...)`
- `refine_registration(...)`
- `register_template_to_scene(template_pcd, scene_pcd, cfg)`
- `validate_registration_result(...)`

默认 source 是 `template_front.ply`，target 是现场 ROI 点云。优先使用 Generalized ICP；Open3D 版本不支持时退回 point-to-plane ICP。

## pose_output.py

负责把 `T_final` 转换为 AGV 可用的叉取结果。

主要接口：

- `ForkPoseResult`
- `transform_point(T, point)`
- `compute_insertion_direction(R, template_insert_axis)`
- `compute_yaw_from_insert_direction(d_insert)`
- `compute_fork_pose(...)`

输出字段包括 `fork_center`、`insertion_direction`、`yaw_rad`、`yaw_deg`、`distance`、`lateral_error`、`height`、`fitness`、`inlier_rmse`。

## realtime_pipeline.py

实时和离线共用的编排层。它把检测、反投影、点云预处理、配准、位姿输出串联起来。

主要接口：

- `PalletPoseRealtimePipeline`
- `PipelineResult`

特性：

- 接收 BGR color image 和 aligned depth image。
- 捕获单帧处理中的异常，返回 `PipelineResult`，避免实时节点崩溃。
- 支持 `max_fps` 和 `process_every_n_frames` 控制处理频率。
- 支持保存 debug 图像和点云。

## cli.py

离线命令行入口。

典型用途：

```bash
python -m pallet_pose_estimation.cli --config config/default.yaml --rgb data/rgb.png --depth data/depth.png --output outputs/result.json
```

## realsense_node.py

RealSense D435 直连入口，不依赖 ROS。

职责：

- 启动 color/depth stream。
- 使用 `rs.align(rs.stream.color)` 对齐深度到彩色图。
- 从 RealSense intrinsics 转换为 `CameraIntrinsics`。
- 调用 `PalletPoseRealtimePipeline`。
- 终端打印 fork pose，并可显示 debug 窗口。

## ros_utils.py

ROS1 消息转换工具。

主要接口：

- `ros_image_to_cv2(...)`
- `camera_info_to_intrinsics(...)`
- `numpy_points_to_pointcloud2(...)`
- `fork_pose_to_pose_stamped(...)`
- `fork_pose_to_json_msg(...)`
- `draw_ros_debug_image(...)`

该模块集中处理 `cv_bridge`、`CameraInfo`、`PointCloud2`、`PoseStamped`、JSON `String`。

## ros1_node.py

ROS1 Noetic 节点入口。

职责：

- 使用 `message_filters.ApproximateTimeSynchronizer` 同步 RGB、aligned depth、CameraInfo。
- 调用 `PalletPoseRealtimePipeline`。
- 发布 `/pallet_pose/pose`、`/pallet_pose/result`、`/pallet_pose/status`、`/pallet_pose/debug_image`、`/pallet_pose/roi_cloud` 和 TF。

## visualization.py

离线调试可视化工具。

主要接口：

- `draw_detection(...)`
- `visualize_registration(...)`
- `save_debug_point_clouds(...)`
