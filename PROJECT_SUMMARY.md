# 项目总结

## 项目定位

`pallet_pose_estimation` 是一个面向 AGV 叉取任务的机器人视觉工程原型，用于估计**托盘可叉取前表面**的局部位姿。

项目不做完整托盘三维重建，也不把完整托盘外框作为默认目标。系统只关注叉车真正需要对准的前表面局部结构，包括前表面、立柱、叉孔边界和横梁。

## 核心能力

项目当前支持三类输入方式：

1. 离线 RGB-D 图像处理；
2. Intel RealSense D435 直连实时处理；
3. ROS1 Noetic 话题订阅模式。

同时支持“YOLO 多候选检测 + 辅助标签 OCR 目标绑定”：

- YOLO 检测所有 `pallet_front` / `forkable_front` 候选框；
- 在每个 YOLO 框内部裁剪辅助标签区域；
- 使用 EasyOCR 识别标签，例如 `A102`、`B203`；
- 根据人工指定的 `target_label` 选择目标托盘；
- 只把目标托盘检测框对应的 aligned depth ROI 送入点云处理；
- 如果目标标签未找到，默认不进入点云配准，避免误叉取其他托盘。

## 处理流程

完整流程如下：

```text
RGB image + aligned depth image
  -> YOLO 检测所有托盘可叉取前表面候选框
  -> 在每个检测框内部裁剪辅助标签区域
  -> OCR 识别并归一化标签
  -> 匹配 target_label，得到 target_bbox
  -> 使用 target_bbox 裁剪 aligned depth ROI
  -> 根据相机内参反投影为 ROI 点云
  -> 使用 T_B_C 转换到 AGV 车体坐标系
  -> 点云降采样、离群点滤波、RANSAC 去地面、ROI 过滤
  -> 加载托盘前表面局部模板点云
  -> FPFH 特征匹配 + RANSAC 粗配准
  -> GICP 或 point-to-plane ICP 精配准
  -> 输出叉取中心、插入方向、偏航角、距离和横向偏差
```

## 模块结构

主要 Python 模块位于 `src/pallet_pose_estimation/`：

- `config.py`：读取和校验 YAML 配置。
- `detector.py`：YOLO 检测 `pallet_front` / `forkable_front`。
- `assist_label.py`：辅助标签裁剪、OCR 和标签归一化。
- `target_selector.py`：根据 `target_label` 从多个 YOLO 候选框中选择目标托盘。
- `rgbd.py`：深度 ROI 裁剪和反投影。
- `preprocessing.py`：点云转换、降采样、离群点滤波、去地面和 ROI 过滤。
- `registration.py`：FPFH-RANSAC 粗配准和 GICP / ICP 精配准。
- `pose_output.py`：计算 AGV 可用的叉取位姿输出。
- `visualization.py`：检测框、目标选择和点云配准调试可视化。
- `realtime_pipeline.py`：离线、RealSense 和 ROS 共用的实时处理编排层。
- `cli.py`：离线 RGB-D 命令行入口。
- `realsense_node.py`：RealSense D435 直连入口。
- `ros_utils.py`：ROS1 消息转换工具。
- `ros1_node.py`：ROS1 Noetic 节点。

## 坐标系约定

默认坐标系约定：

- `base_link` / AGV 车体坐标系：x 前向，y 左向，z 上方；
- `camera_color_optical_frame`：RealSense 彩色相机光学坐标系；
- `T_B_C`：从相机坐标系到 AGV 车体坐标系的外参；
- `template_insert_axis`：模板坐标系下的插入方向，默认 `[0, 1, 0]`；
- `fork_center = T_final * template_fork_center`；
- `yaw = atan2(insertion_direction_y, insertion_direction_x)`。

## 输出结果

离线 `result.json` 或 ROS `/pallet_pose/result` 中包含：

- `target_label`
- `selected_label`
- `target_bbox`
- `candidates`
- `selection_status`
- `T_final`
- `fork_center`
- `insertion_direction`
- `yaw_rad`
- `yaw_deg`
- `distance`
- `lateral_error`
- `height`
- `fitness`
- `inlier_rmse`
- `warnings`

其中：

- `distance = fork_center[0]`
- `lateral_error = fork_center[1]`
- `height = fork_center[2]`
- `yaw` 为绕车体 z 轴的偏航角

这些量可直接用于 AGV 对准、横向修正、高度控制和低速插入规划。

## 运行方式

### 离线 RGB-D

```bash
python -m pallet_pose_estimation.cli ^
  --config config/default.yaml ^
  --rgb data/rgb.png ^
  --depth data/aligned_depth.png ^
  --target-label A102 ^
  --output outputs/result.json ^
  --debug-vis
```

### RealSense D435 直连

```bash
python -m pallet_pose_estimation.realsense_node ^
  --config config/realsense.yaml ^
  --target-label A102 ^
  --show
```

### ROS1 Noetic

启动 RealSense wrapper：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

启动本节点：

```bash
roslaunch pallet_pose_estimation pallet_pose_from_topics.launch \
  config_path:=/abs/path/to/config/ros1_noetic.yaml \
  target_label:=A102
```

## ROS 话题

ROS1 模式默认订阅：

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`

默认发布：

- `/pallet_pose/pose`
- `/pallet_pose/result`
- `/pallet_pose/status`
- `/pallet_pose/debug_image`
- `/pallet_pose/roi_cloud`
- `/pallet_pose/target_bbox`
- `/pallet_pose/candidates`
- `/tf`

## 关键配置

主要配置文件：

- `config/default.yaml`：离线 RGB-D 模式；
- `config/realsense.yaml`：RealSense D435 直连模式；
- `config/ros1_noetic.yaml`：ROS1 Noetic 模式。

重要配置项：

- `yolo_model_path`
- `yolo_class_name`
- `confidence_threshold`
- `depth_scale`
- `camera_intrinsics`
- `extrinsic_T_B_C`
- `template_point_cloud_path`
- `template_fork_center`
- `template_insert_axis`
- `target_selection.target_label`
- `target_selection.fallback_strategy`
- `rgbd.require_aligned_depth_to_color`

默认 `fallback_strategy` 为 `error`，即没有匹配到目标标签时不继续点云配准。

## 用户需要提供的数据

真实运行前仍需要补充：

- 训练好的 `pallet_front` / `forkable_front` YOLO 权重；
- 带辅助标签的托盘图像或实时相机画面；
- 目标标签 `target_label`，例如 `A102`；
- 与 RGB 对齐的 aligned depth；
- RealSense D435 或等价 RGB-D 相机；
- 准确相机内参或 ROS `CameraInfo`；
- 相机到 AGV 车体坐标系的外参 `T_B_C`；
- 前表面局部模板点云 `template_front.ply`。

## 当前验证状态

当前工程已包含基础单元测试，覆盖：

- RGB-D 反投影公式；
- 叉取位姿输出；
- 辅助标签归一化；
- 目标托盘选择策略；
- realtime pipeline 中目标框到深度 ROI 的调用路径；
- ROS 工具函数的轻量测试。

最近一次验证结果：

```text
16 passed, 1 skipped
```

其中 skipped 项为当前非 ROS 环境下自动跳过的 ROS 消息测试。

## 常见风险

- YOLO 检测框偏移或检测成完整托盘外框；
- 辅助标签太小、模糊、反光或被遮挡；
- RGB 与深度未对齐；
- 深度空洞导致 ROI 点云稀疏；
- ROI 混入地面、货物、背景或相邻托盘；
- FPFH 在重复结构上误匹配；
- ICP 初值差导致局部最优；
- 模板点云与真实托盘规格不一致；
- 相机外参 `T_B_C` 不准确。
