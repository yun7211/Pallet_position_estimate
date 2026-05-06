# 运行指南

## 1. 安装

普通 Python 环境：

```bash
cd pallet_pose_estimation
pip install -r requirements.txt
```

完整 Open3D / RealSense 流程建议使用 Python 3.10、3.11 或 3.12。

## 2. 离线 RGB-D 模式

准备：

- RGB 图像。
- aligned depth 图像。
- YOLO 权重。
- `template_front.ply`。
- 相机内参和 `T_B_C`。

运行：

```bash
python -m pallet_pose_estimation.cli ^
  --config config/default.yaml ^
  --rgb data/rgb.png ^
  --depth data/depth.png ^
  --output outputs/result.json ^
  --debug-vis ^
  --no-window
```

## 3. RealSense D435 直连模式

运行：

```bash
python -m pallet_pose_estimation.realsense_node --config config/realsense.yaml --show
```

脚本方式：

```bash
python scripts/run_realsense_direct.py --config config/realsense.yaml --show
```

保存调试数据：

```bash
python -m pallet_pose_estimation.realsense_node --config config/realsense.yaml --save-debug outputs/realsense_debug
```

直连模式不发布 ROS topic，只用于本地实时验证。

## 4. ROS1 Noetic 模式

先启动 RealSense ROS wrapper：

```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

只订阅已有 RGB-D topic：

```bash
roslaunch pallet_pose_estimation pallet_pose_from_topics.launch \
  config_path:=/abs/path/to/config/ros1_noetic.yaml
```

启动 RealSense wrapper 和本节点：

```bash
roslaunch pallet_pose_estimation pallet_pose_realsense.launch \
  config_path:=/abs/path/to/config/ros1_noetic.yaml \
  align_depth:=true
```

直接运行节点：

```bash
rosrun pallet_pose_estimation ros1_node.py _config_path:=/abs/path/to/config/ros1_noetic.yaml
```

## 5. ROS 话题

订阅默认话题：

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`

发布默认话题：

- `/pallet_pose/pose`
- `/pallet_pose/result`
- `/pallet_pose/status`
- `/pallet_pose/debug_image`
- `/pallet_pose/roi_cloud`
- `/tf`

查看输出：

```bash
rostopic echo /pallet_pose/result
rostopic echo /pallet_pose/status
rviz
```

## 6. rosbag

录制：

```bash
rosbag record \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info
```

回放：

```bash
rosbag play xxx.bag --clock
```
