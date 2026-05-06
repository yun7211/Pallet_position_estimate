# 故障排查

## no_detection

可能原因：

- YOLO 权重路径错误。
- 模型没有 `pallet_front` 或 `forkable_front` 类别。
- 检测阈值过高。
- 图像颜色通道或曝光异常。

处理：

- 检查 `yolo_model_path`。
- 检查 `yolo_class_name`。
- 降低 `confidence_threshold`。
- 保存 debug image 查看检测框。

## empty_roi_cloud

可能原因：

- 深度图没有与彩色图对齐。
- 检测框区域深度为空洞。
- `depth_scale` 设置错误。
- `depth_min/depth_max` 过滤过严。

处理：

- RealSense ROS wrapper 启动时设置 `align_depth:=true`。
- 确认订阅 `/camera/aligned_depth_to_color/image_raw`。
- 检查深度原始单位并修正 `depth_scale`。

## global_registration_failed

可能原因：

- ROI 点云太稀疏。
- 模板和真实托盘规格不一致。
- FPFH 在重复叉孔或横梁结构上误匹配。
- ROI 混入地面、货物、相邻托盘。

处理：

- 调整 `voxel_size`、FPFH 半径和 RANSAC 阈值。
- 改进 YOLO 检测框。
- 加强高度和横向 ROI 过滤。
- 使用更干净的 `template_front.ply`。

## registration_unreliable

可能原因：

- fitness 低。
- inlier RMSE 高。
- ICP 初值差导致局部最优。
- 外参 T_B_C 错误。

处理：

- 检查 debug 点云。
- 确认模板坐标系和 `template_insert_axis`。
- 降低处理速度，避免运动模糊或时间不同步。

## RealSense 问题

- 找不到设备：检查 USB、权限和占用进程。
- 帧率低：优先使用 USB 3.0。
- 深度和彩色错位：必须启用 depth-to-color alignment。
- 深度单位不对：使用 `realsense.depth_scale: "auto"` 或手动设置正确值。

## ROS1 问题

- 没有 CameraInfo：检查 `/camera/color/camera_info` 是否存在。
- 话题名不对：使用 `rostopic list` 查看实际名称。
- 节点卡顿：降低 `runtime.max_fps` 或增大 `process_every_n_frames`。
- RViz 看不到点云：检查 `/pallet_pose/roi_cloud` 的 frame_id 和 TF。
