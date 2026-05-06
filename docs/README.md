# 项目文档索引

本目录用于说明 `pallet_pose_estimation` 的模块化结构、运行方式、配置项和扩展边界。

- [工程架构](architecture.md)：整体分层、数据流、坐标系和模块依赖。
- [模块说明](modules.md)：逐个 Python 模块说明职责、输入输出和复用方式。
- [运行指南](usage.md)：离线 RGB-D、RealSense D435 直连、ROS1 Noetic 三种模式。
- [配置说明](configuration.md)：`default.yaml`、`realsense.yaml`、`ros1_noetic.yaml` 的关键参数。
- [数据准备](data_preparation.md)：YOLO 权重、RGB-D 图像、CameraInfo、外参和模板点云准备。
- [辅助标签目标选择](target_selection.md)：YOLO 多候选框、框内 OCR、target_label 匹配和 fallback 策略。
- [故障排查](troubleshooting.md)：检测、深度、点云、配准、RealSense 和 ROS 常见问题。

项目目标始终是估计**托盘可叉取前表面**局部位姿，不做完整托盘三维重建。
