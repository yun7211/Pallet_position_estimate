# 辅助标签目标托盘选择

## 为什么需要辅助标签

多个托盘并排时，YOLO 可以检测出多个 `pallet_front` / `forkable_front` 候选框，但无法知道 AGV 当前应该叉取哪个托盘。辅助标签 OCR 用于把人工指定的 `target_label`，例如 `A102`，绑定到目标托盘检测框。

辅助标签只负责目标选择，不替代 YOLO 检测，也不参与最终点云配准。

## 输入

- RGB image。
- 与 RGB 对齐的 aligned depth image。
- YOLO pallet_front / forkable_front 模型。
- `target_label`，例如 `A102`。
- 相机内参。
- 相机到 AGV 车体坐标系外参 `T_B_C`。
- 前表面局部模板点云 `template_front.ply`。

## 流程

```text
YOLO pallet_front detection
  -> for each YOLO box, crop label inside the box
  -> EasyOCR recognition
  -> normalize to letter+digits, e.g. A102
  -> match target_label
  -> selected target_bbox
  -> crop aligned depth ROI by target_bbox
  -> backproject target ROI cloud
  -> preprocessing
  -> FPFH-RANSAC + GICP/ICP
  -> fork pose output
```

## 选择策略

有 `target_label` 时：

- 唯一匹配：选择该候选框。
- 多个匹配：选择 YOLO confidence 最高者，并给 warning。
- 无匹配：返回 `target_label_not_found`，不进入点云配准。

无 `target_label` 时：

- `error`：返回 `target_label_required`。
- `highest_confidence`：选择 YOLO 置信度最高框。
- `center_nearest`：选择距离图像中心最近框。
- `first_valid_label`：选择第一个 OCR 有效标签框。

默认策略是 `error`，避免多托盘场景误叉取。

## 运行示例

离线：

```bash
python -m pallet_pose_estimation.cli \
  --config config/default.yaml \
  --rgb data/rgb.png \
  --depth data/aligned_depth.png \
  --target-label A102 \
  --output outputs/result.json
```

RealSense：

```bash
python -m pallet_pose_estimation.realsense_node \
  --config config/realsense.yaml \
  --target-label A102 \
  --show
```

ROS1：

```bash
roslaunch pallet_pose_estimation pallet_pose_from_topics.launch target_label:=A102
```

## 注意事项

- 深度图必须是 `aligned_depth_to_color`。
- OCR 标签必须位于 YOLO 检测框内部。
- 标签太小、模糊、反光或遮挡时，OCR 可能失败。
- 标签识别失败时，默认不进行点云配准。
- 辅助标签只是目标选择，最终叉取仍依赖 RGB-D ROI 点云和局部模板配准。
