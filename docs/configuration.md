# 配置说明

## 公共算法配置

三个 YAML 配置都包含以下核心项：

- `yolo_model_path`：YOLO 权重路径。
- `yolo_class_name`：检测类别，建议 `pallet_front` 或 `forkable_front`。
- `confidence_threshold`：YOLO 置信度阈值。
- `depth_scale`：深度原始值转米的除数。
- `depth_min` / `depth_max`：深度过滤范围，单位米。
- `voxel_size`：点云体素降采样尺度。
- `normal_radius_factor`：法向估计半径系数。
- `fpfh_radius_factor`：FPFH 半径系数。
- `ransac_distance_threshold_factor`：RANSAC 距离阈值系数。
- `icp_max_correspondence_distance_factor`：ICP 对应距离阈值系数。
- `icp_method`：`gicp` 或 `point_to_plane`。
- `camera_intrinsics`：离线或 fallback 使用的 `fx, fy, cx, cy`。
- `extrinsic_T_B_C`：camera frame 到 AGV body frame 的 4x4 外参。
- `template_point_cloud_path`：前表面局部模板点云。
- `template_fork_center`：模板坐标系下叉取中心。
- `template_insert_axis`：模板坐标系下插入方向，默认 `[0, 1, 0]`。

## config/default.yaml

用于离线 RGB-D 图片处理。

重点检查：

- `camera_intrinsics`
- `extrinsic_T_B_C`
- `template_point_cloud_path`
- `depth_scale`

## config/realsense.yaml

用于 pyrealsense2 直连。

RealSense 配置：

- `realsense.width`
- `realsense.height`
- `realsense.fps`
- `realsense.align_depth_to_color`
- `realsense.enable_auto_exposure`
- `realsense.depth_scale`

当 `realsense.depth_scale: "auto"` 时，程序从 RealSense depth sensor 读取 depth units。

运行配置：

- `runtime.max_fps`
- `runtime.process_every_n_frames`
- `runtime.show_window`
- `runtime.save_debug`
- `runtime.debug_output_dir`

## config/ros1_noetic.yaml

用于 ROS1 节点。

ROS 配置：

- `ros.color_topic`
- `ros.depth_topic`
- `ros.camera_info_topic`
- `ros.queue_size`
- `ros.slop`
- `ros.base_frame`
- `ros.camera_frame`
- `ros.child_frame`
- `ros.publish_tf`
- `ros.publish_debug_image`
- `ros.publish_debug_cloud`
- `ros.use_camera_info_intrinsics`
- `ros.fallback_intrinsics`

注意：RealSense ROS wrapper 的话题名称可能随 namespace 改变。先用 `rostopic list` 确认真实话题，再修改 YAML 或 launch 参数。
