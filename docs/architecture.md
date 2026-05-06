# 工程架构

## 目标

本工程用于 AGV 叉取任务中的托盘可叉取前表面位姿估计。系统只估计局部前表面位姿，不构建完整托盘三维模型。

## 分层结构

```text
输入层
  离线 RGB-D 文件
  RealSense D435 直连
  ROS1 RGB / aligned depth / CameraInfo 话题

感知层
  detector.py         YOLO 检测 pallet_front / forkable_front
  rgbd.py             aligned depth ROI 反投影

点云处理层
  preprocessing.py    坐标变换、降采样、离群点滤波、去地面、ROI 过滤
  registration.py     FPFH-RANSAC 粗配准，GICP/ICP 精配准

结果层
  pose_output.py      fork_center、insertion_direction、yaw、distance 等输出
  visualization.py    OpenCV/Open3D 调试可视化

编排层
  realtime_pipeline.py 复用离线和实时处理流程
  cli.py               离线命令行入口
  realsense_node.py    pyrealsense2 直连入口
  ros1_node.py         ROS1 Noetic 节点入口
  ros_utils.py         ROS 消息转换
```

## 核心数据流

```text
color image + aligned depth
  -> YOLO 检测可叉取前表面 bbox
  -> 裁剪 depth ROI
  -> 相机内参反投影为 camera frame 点云
  -> T_B_C 转换到 base_link / AGV body frame
  -> 点云预处理与去地面
  -> 加载 template_front.ply
  -> FPFH + RANSAC 得到 T_init
  -> GICP 或 point-to-plane ICP 得到 T_final
  -> 输出 fork_center / yaw / distance / lateral_error / height
```

## 坐标系约定

- `base_link` / AGV 车体坐标系：x 前向，y 左向，z 上方。
- `camera_color_optical_frame`：RealSense 彩色相机光学坐标系。
- `T_B_C`：从 camera frame 到 base frame 的外参。
- 模板坐标系默认 `template_insert_axis = [0, 1, 0]`。
- `fork_center = T_final * template_fork_center`。
- `insertion_direction = R_final * template_insert_axis`。
- `yaw = atan2(insertion_direction_y, insertion_direction_x)`。

## 模块依赖原则

- 算法模块不依赖 ROS。
- `realtime_pipeline.py` 复用算法模块，统一异常处理和调试输出。
- `realsense_node.py` 只负责相机采集和调用 pipeline。
- `ros1_node.py` 只负责 ROS 订阅、发布、TF 和调用 pipeline。
- ROS 工具函数集中在 `ros_utils.py`，不污染离线算法模块。
