# Pallet Pose Estimation

基于 YOLO、辅助标签 OCR、RGB-D 点云和 FPFH-GICP 的托盘可叉取前表面位姿估计工程。

项目的目标是：

- 输入形式支持 `图片` 和 `实时视频流`
- 从多个托盘候选中，通过辅助标签 ID 锁定目标托盘
- 只对目标托盘检测框对应的 `aligned depth ROI` 做点云处理
- 输出托盘相对于`相机坐标系`的四个核心参数：
  `x`, `y`, `z`, `yaw`
- 在 RGB 画面上显示目标托盘检测框和辅助标签 ID
- 把中间结果保存到一个文件夹，便于排查整个链路

## 两种直接可运行的接口

## 1. 图片接口

输入：

- `RGB` 图片
- 与 RGB 对齐的 `aligned depth` 图片

运行：

```bash
python -m pallet_pose_estimation.cli ^
  --config config/default.yaml ^
  --rgb data/rgb.png ^
  --depth data/aligned_depth.png ^
  --target-label A102 ^
  --output outputs/result.json ^
  --save-dir outputs/debug_run ^
  --debug-vis
```

如果不想使用 OCR，而是单托盘兜底运行：

```bash
python -m pallet_pose_estimation.cli ^
  --config config/default.yaml ^
  --rgb data/rgb.png ^
  --depth data/aligned_depth.png ^
  --fallback-strategy highest_confidence ^
  --disable-ocr ^
  --output outputs/result.json ^
  --save-dir outputs/debug_run
```

## 2. 实时视频流接口

当前实时视频流接口基于 `Intel RealSense D435` 直连，输入是实时 RGB-D 视频流。

运行：

```bash
python -m pallet_pose_estimation.realsense_node ^
  --config config/realsense.yaml ^
  --target-label A102 ^
  --show ^
  --save-debug outputs/realsense_debug
```

说明：

- 该接口会使用 `rs.align(rs.stream.color)` 将 depth 对齐到 color
- 只把目标托盘检测框对应的 depth ROI 送入点云处理
- 窗口里会显示所有候选框、辅助标签 ID 和最终目标框

## 处理流程

```text
RGB image + aligned depth image
  -> YOLO 检测所有 pallet_front / forkable_front 候选框
  -> 在每个 YOLO 框内部裁剪辅助标签区域
  -> OCR 识别辅助标签
  -> 与预先指定的 target_label 匹配
  -> 得到唯一 target_bbox
  -> 用 target_bbox 裁剪 aligned depth ROI
  -> 反投影为目标托盘点云
  -> 点云预处理
  -> FPFH-RANSAC + GICP / ICP
  -> 输出相机坐标系下的 x / y / z / yaw
```

## 输出结果

图片接口会输出 `result.json`，实时流接口会在终端打印结果。

当前核心输出已经改成相机坐标系优先，包括：

- `x`
- `y`
- `z`
- `yaw_rad`
- `yaw_deg`
- `camera_pose`

同时保留目标选择和配准相关信息：

- `target_label`
- `selected_label`
- `target_bbox`
- `candidates`
- `selection_status`
- `fitness`
- `inlier_rmse`
- `warnings`

## 中间结果保存

当启用 `--save-dir` 或 `--save-debug` 时，会保存中间产物，方便逐步排查：

- `yolo_candidates.png`
  只显示 YOLO 初步检测结果
- `target_selection.png`
  显示候选框、辅助标签 ID、目标框和相机系输出
- `label_crops/`
  各候选框内部裁剪出的辅助标签图
- `raw_roi_camera.ply`
  目标检测框对应的原始相机系点云
- `raw_roi_body.ply`
  变换到车体系后的原始点云
- `preprocessed_scene.ply`
  预处理后的点云
- `ground_removed_scene.ply`
  去地面后的点云
- `filtered_scene.ply`
  经过 ROI 和高度过滤后的最终场景点云
- `template_front.ply`
  模板点云
- `aligned_template_final.ply`
  最终配准后的模板点云

## 目标托盘选择

辅助标签只负责目标选择，不替代 YOLO 检测，也不参与最终位姿精配准。

选择逻辑：

- 如果指定了 `target_label`，只有 OCR 标签匹配的托盘才会进入点云处理
- 如果找不到目标标签，默认返回错误，不继续配准
- 如果不指定 `target_label`，可以通过 `fallback_strategy` 决定是否使用最高置信度托盘

推荐：

- 多托盘真实叉取场景：`fallback_strategy=error`
- 单托盘调试场景：`fallback_strategy=highest_confidence`

## 关键要求

- 深度图必须是 `aligned_depth_to_color`
- 辅助标签裁剪必须严格在 YOLO 检测框内部
- 最终输出要求的是相机坐标系，不是 AGV 车体系
- 如果辅助标签匹配失败，默认不能静默切换到其他托盘

## 配置文件

- `config/default.yaml`：图片接口
- `config/realsense.yaml`：RealSense 实时视频流接口
- `config/ros1_noetic.yaml`：ROS1 Noetic 话题接口

## 测试

```bash
python -B -m pytest -p no:cacheprovider
```

当前测试覆盖：

- 辅助标签归一化
- 目标托盘选择
- RGB-D 反投影
- 实时 pipeline 中 target_bbox 到 depth ROI 的调用路径
- 位姿输出

## 依赖

建议使用 Python 3.10 到 3.12。

主要依赖：

- `ultralytics`
- `opencv-python`
- `numpy`
- `open3d`
- `pyyaml`
- `easyocr`
- `pyrealsense2`（实时视频流接口）

## 真实运行前还需要提供

- 训练好的 `pallet_front` / `forkable_front` YOLO 权重
- 带辅助标签的托盘 RGB 图像或实时画面
- 目标辅助标签 ID，例如 `A102`
- 与 RGB 对齐的 `aligned depth`
- 相机内参
- 相机到车体外参 `T_B_C`
- 前表面局部模板点云 `template_front.ply`
