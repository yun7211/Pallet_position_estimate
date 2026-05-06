# 数据准备

## YOLO 权重

YOLO 模型必须能检测托盘可叉取前表面，类别名建议：

- `pallet_front`
- `forkable_front`

标注训练数据时不要把完整托盘外轮廓作为默认目标，否则 ROI 会混入地面、货物、背景和侧面结构。

## RGB-D 图像

离线模式要求：

- RGB 图像和深度图来自同一时刻。
- 深度图必须与 RGB 图像对齐。
- 深度图可以是 `uint16` 毫米深度，也可以是米单位浮点深度。
- 根据深度单位正确设置 `depth_scale`。

## RealSense D435

直连模式会启用：

- color stream：`bgr8`
- depth stream：`z16`
- depth alignment：`rs.align(rs.stream.color)`

ROS 模式建议使用：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

并订阅 `/camera/aligned_depth_to_color/image_raw`。

## 相机内参

离线模式使用 YAML 中的：

```yaml
camera_intrinsics:
  fx: 600.0
  fy: 600.0
  cx: 320.0
  cy: 240.0
```

ROS 模式优先使用 `CameraInfo.K`。

## 外参 T_B_C

`extrinsic_T_B_C` 是从相机坐标系到 AGV 车体坐标系的 4x4 齐次矩阵。

输出的 `fork_center`、`distance`、`lateral_error`、`height` 都依赖该外参。如果外参错误，位姿输出会系统性偏移。

## template_front.ply

模板只应包含：

- 托盘可叉取前表面；
- 左右立柱；
- 叉孔边界；
- 前横梁；
- 与叉取定位相关的局部几何边缘。

不要使用完整托盘模型。
