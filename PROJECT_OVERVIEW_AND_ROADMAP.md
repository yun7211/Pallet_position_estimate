# 项目全景说明与发展规划

## 1. 项目定位

`pallet_pose_estimation` 是一个面向叉取场景的机器人视觉工程原型，目标不是做完整托盘三维重建，而是估计**托盘可叉取前表面**的局部位姿，并进一步输出可直接用于控制的目标信息。

当前项目已经支持两类主输入形式：

1. 图片输入  
   输入为 `RGB 图像 + 与 RGB 对齐的 aligned depth 图像`
2. 实时视频流输入  
   当前以 `Intel RealSense D435` 直连 RGB-D 视频流为主

项目输出聚焦于**托盘相对于相机坐标系**的四个核心参数：

- `x`
- `y`
- `z`
- `yaw`

同时在 RGB 画面中显示：

- 目标托盘检测框
- 辅助标签 ID
- 目标选择状态

并将中间调试结果保存到指定文件夹，便于排查每个阶段的问题。

---

## 2. 项目解决的问题

在多托盘并排或场景干扰较多时，仅靠 YOLO 检测往往只能得到多个候选托盘框，无法稳定知道“哪一个才是当前要叉取的托盘”。  
因此本项目在 YOLO 检测之前后之间增加了一层**辅助标签绑定的目标托盘选择机制**：

1. YOLO 负责检测所有候选托盘前表面
2. 在每个 YOLO 检测框内部裁剪辅助标签区域
3. 使用 OCR 识别标签，例如 `A102`
4. 将 OCR 结果与人工提前指定的 `target_label` 进行匹配
5. 只有匹配到的候选框才进入深度 ROI、点云处理和配准

也就是说：

- YOLO 负责“找候选托盘”
- OCR 负责“从多个候选中锁定目标托盘”
- 点云与模板配准负责“精确估计最终叉取位姿”

---

## 3. 当前项目目录结构

```text
pallet_pose_estimation/
  README.md
  PROJECT_SUMMARY.md
  PROJECT_OVERVIEW_AND_ROADMAP.md
  requirements.txt
  pyproject.toml
  setup.py
  package.xml
  CMakeLists.txt

  config/
    default.yaml
    realsense.yaml
    ros1_noetic.yaml

  docs/
    README.md
    architecture.md
    configuration.md
    data_preparation.md
    modules.md
    target_selection.md
    troubleshooting.md
    usage.md

  src/
    pallet_pose_estimation/
      __init__.py
      config.py
      detector.py
      assist_label.py
      target_selector.py
      rgbd.py
      preprocessing.py
      registration.py
      pose_output.py
      visualization.py
      realtime_pipeline.py
      cli.py
      realsense_node.py
      ros_utils.py
      ros1_node.py

  launch/
    pallet_pose_from_topics.launch
    pallet_pose_realsense.launch

  ros/
    CMakeLists.txt
    package.xml
    launch/
      pallet_pose_from_topics.launch
      pallet_pose_realsense.launch

  scripts/
    run_realsense_direct.py
    run_ros1_node.py
    ros1_node.py

  tests/
    conftest.py
    test_assist_label.py
    test_pose_output.py
    test_realtime_pipeline_mock.py
    test_realtime_pipeline_target_label.py
    test_rgbd.py
    test_ros_utils_intrinsics.py
    test_ros_utils_pose.py
    test_target_selector.py
```

---

## 4. 当前实现的整体流程

### 4.1 图片 / 视频流统一主流程

```text
color_image + aligned_depth_image
  -> YOLO 检测所有 pallet_front / forkable_front 候选框
  -> 在每个检测框内部裁剪辅助标签区域
  -> OCR 识别标签
  -> 根据 target_label 选择目标托盘 target_bbox
  -> 用 target_bbox 裁剪 aligned depth ROI
  -> 反投影为目标点云
  -> 点云预处理
  -> 去地面 / 去背景 / 去无关结构
  -> 使用局部模板做 FPFH-RANSAC 粗配准
  -> 使用 GICP 或 point-to-plane ICP 精配准
  -> 输出相机坐标系下的 x / y / z / yaw
```

### 4.2 输入要求

当前核心输入要求是：

- RGB 图像
- 与 RGB 严格对齐的深度图 `aligned_depth_to_color`
- 相机内参
- 可选的辅助标签 `target_label`
- 托盘前表面局部模板点云 `template_front.ply`

如果深度图没有和 RGB 图像对齐，当前工程默认会拒绝继续执行，因为检测框与深度 ROI 会发生错位。

---

## 5. 当前已经实现的功能

### 5.1 YOLO 检测

模块：`detector.py`

已实现：

- 加载 `ultralytics.YOLO` 模型
- 支持从 `numpy` 图像直接推理
- 支持检测所有候选框 `detect_all`
- 支持兼容旧接口 `detect_pallet_front`
- 按类别过滤 `pallet_front` / `forkable_front`
- 按置信度过滤
- 检测框边界裁剪到图像范围内

### 5.2 辅助标签 OCR

模块：`assist_label.py`

已实现：

- 定义 `PalletCandidate`
- 在 YOLO 检测框内部裁剪标签区域
- 使用 EasyOCR 识别标签
- 标签归一化为 `字母 + 数字` 格式
- 易混字符映射，例如：
  - `O/Q/D -> 0`
  - `I/L -> 1`
  - `Z -> 2`
  - `S -> 5`
  - `B -> 8`
  - `G -> 6`
- OCR 异常保护，不让主流程崩溃
- 支持禁用 OCR

### 5.3 目标托盘选择

模块：`target_selector.py`

已实现：

- 从 YOLO 多候选框中选择目标托盘
- 支持显式指定 `target_label`
- 支持以下回退策略：
  - `error`
  - `highest_confidence`
  - `center_nearest`
  - `first_valid_label`
- 目标标签唯一匹配时返回唯一目标框
- 多个候选同时匹配时按 YOLO 置信度最高者选择，并给 warning
- 找不到目标标签时返回 `target_label_not_found`
- 保留所有候选框和 OCR 信息，便于调试与 ROS 输出

### 5.4 RGB-D 处理

模块：`rgbd.py`

已实现：

- 按检测框裁剪深度 ROI
- 按内参反投影为三维点云
- 支持 `uint16` 深度输入
- 支持 `depth_scale`
- 自动过滤：
  - `0`
  - `NaN`
  - 过近深度
  - 过远深度

### 5.5 点云预处理

模块：`preprocessing.py`

已实现：

- `numpy` 点集转 Open3D 点云
- 点云刚体变换
- 体素降采样
- 统计离群点滤波
- RANSAC 去地面
- 按高度和 ROI 限制过滤点云

### 5.6 点云配准

模块：`registration.py`

已实现：

- 模板点云加载
- 降采样与法向估计
- FPFH 特征提取
- 基于特征匹配 + RANSAC 的全局粗配准
- GICP 精配准
- Open3D 不支持 GICP 时自动 fallback 到 point-to-plane ICP
- 基本几何合理性校验
- 输出：
  - `T_init`
  - `T_final`
  - `fitness`
  - `inlier_rmse`

### 5.7 位姿输出

模块：`pose_output.py`

已实现两类输出：

1. **车体坐标系输出**
   - `fork_center`
   - `distance`
   - `lateral_error`
   - `height`
   - `yaw`

2. **相机坐标系输出**
   - `x`
   - `y`
   - `z`
   - `yaw`

当前对外主输出已经切换为**相机坐标系四参数**，更符合你现在提出的接口要求。

### 5.8 可视化与中间结果保存

模块：`visualization.py`

已实现：

- 绘制 YOLO 候选框
- 绘制目标选择结果
- 绘制 OCR 标签
- 在 RGB 图像上标出目标框与状态信息
- 保存标签裁剪图
- 保存点云处理各阶段结果

### 5.9 统一实时编排层

模块：`realtime_pipeline.py`

已实现：

- 离线 CLI、RealSense、ROS1 共用同一套处理逻辑
- 支持输入：
  - `color_image`
  - `depth_image`
  - `intrinsics`
  - `timestamp`
  - `frame_id`
- 内部统一处理：
  - 目标选择
  - 深度 ROI
  - 反投影
  - 点云预处理
  - 模板配准
  - 位姿输出
- 支持异常收敛为结构化 `PipelineResult`
- 支持保存调试结果
- 支持处理频率限制，避免实时模式卡死

### 5.10 图片接口

模块：`cli.py`

当前支持：

- 输入单张 RGB 图像与 aligned depth 图像
- 指定 `target_label`
- 指定 `fallback_strategy`
- 启用或禁用 OCR
- 保存中间结果目录
- 输出 `result.json`

### 5.11 RealSense 实时视频流接口

模块：`realsense_node.py`

当前支持：

- 直接通过 `pyrealsense2` 读取 RealSense D435
- 使用 `rs.align(rs.stream.color)` 对齐深度到彩色图
- 支持实时显示窗口
- 支持指定 `target_label`
- 支持保存调试图像与点云
- 在终端打印相机坐标系下的：
  - `x`
  - `y`
  - `z`
  - `yaw`

### 5.12 ROS1 Noetic 接口

模块：`ros1_node.py`、`ros_utils.py`

当前支持：

- 订阅：
  - `color image`
  - `aligned depth image`
  - `CameraInfo`
- 同步 RGB / 深度 / 内参
- 复用统一 pipeline
- 发布：
  - `/pallet_pose/result`
  - `/pallet_pose/status`
  - `/pallet_pose/pose`
  - `/pallet_pose/debug_image`
  - `/pallet_pose/roi_cloud`
  - `/pallet_pose/target_bbox`
  - `/pallet_pose/candidates`
- 支持通过 ROS 参数指定 `target_label`

---

## 6. 当前开放的两类主接口

### 6.1 图片格式接口

适用场景：

- 离线验证
- 单帧调试
- 算法参数对比
- 中间结果分析

运行示例：

```bash
python -m pallet_pose_estimation.cli \
  --config config/default.yaml \
  --rgb data/rgb.png \
  --depth data/aligned_depth.png \
  --target-label A102 \
  --output outputs/result.json \
  --save-dir outputs/debug_run \
  --debug-vis
```

### 6.2 实时视频流接口

适用场景：

- RealSense D435 现场联调
- 实时目标托盘选择验证
- 多托盘动态切换验证

运行示例：

```bash
python -m pallet_pose_estimation.realsense_node \
  --config config/realsense.yaml \
  --target-label A102 \
  --show \
  --save-debug outputs/realsense_debug
```

---

## 7. 当前输出定义

当前对外主输出为**相机坐标系下**的四个参数：

- `x`：目标托盘叉取中心在相机坐标系下的 x 坐标
- `y`：目标托盘叉取中心在相机坐标系下的 y 坐标
- `z`：目标托盘叉取中心在相机坐标系下的 z 坐标
- `yaw`：托盘插入方向在相机坐标系下的偏航角

同时还会输出：

- `target_label`
- `selected_label`
- `target_bbox`
- `selection_status`
- `candidates`
- `fitness`
- `inlier_rmse`
- `warnings`

---

## 8. 当前中间结果保存内容

当启用 `--save-dir` 或 `--save-debug` 时，会保存以下文件：

- `yolo_candidates.png`  
  YOLO 候选托盘框结果

- `target_selection.png`  
  显示候选框、OCR 标签、最终目标框、相机系输出和状态

- `label_crops/`  
  每个候选框内部裁剪得到的标签图

- `raw_roi_camera.ply`  
  相机坐标系下的原始目标 ROI 点云

- `raw_roi_body.ply`  
  变换到车体坐标系后的原始 ROI 点云

- `preprocessed_scene.ply`  
  预处理后的场景点云

- `ground_removed_scene.ply`  
  去地面后的场景点云

- `filtered_scene.ply`  
  最终用于配准的目标场景点云

- `template_front.ply`  
  托盘前表面局部模板点云

- `aligned_template_final.ply`  
  精配准完成后的模板点云

---

## 9. 项目依赖

### 9.1 Python 依赖

来自 `requirements.txt`：

- `numpy>=1.23`
- `opencv-python>=4.7`
- `open3d>=0.17`
- `ultralytics>=8.0`
- `pyyaml>=6.0`
- `scipy>=1.10`
- `pytest>=7.0`
- `pyrealsense2`
- `easyocr`

说明：

- `open3d`、`pyrealsense2`、`easyocr` 在部分 Python 版本下会有兼容限制
- 建议使用 Python `3.10` 到 `3.12`
- 当前 `requirements.txt` 已针对 Python `3.13` 做了条件限制，避免直接安装失败

### 9.2 ROS1 相关系统依赖

如果使用 ROS1 Noetic 模式，还需要：

- `ros-noetic-cv-bridge`
- `ros-noetic-image-transport`
- `ros-noetic-message-filters`
- `ros-noetic-tf`
- `ros-noetic-tf2-ros`
- `ros-noetic-sensor-msgs`
- `ros-noetic-geometry-msgs`
- `ros-noetic-std-msgs`
- `ros-noetic-realsense2-camera`

---

## 10. 当前配置文件

### 10.1 `config/default.yaml`

用途：

- 图片模式
- 离线 RGB-D 测试

### 10.2 `config/realsense.yaml`

用途：

- RealSense D435 直连模式
- 实时视频流验证

### 10.3 `config/ros1_noetic.yaml`

用途：

- ROS1 Noetic 模式
- 话题名称、同步参数和发布控制

### 10.4 当前重点配置项

- `yolo_model_path`
- `yolo_class_name`
- `confidence_threshold`
- `camera_intrinsics`
- `depth_scale`
- `depth_min`
- `depth_max`
- `extrinsic_T_B_C`
- `template_point_cloud_path`
- `template_fork_center`
- `template_insert_axis`
- `target_selection.target_label`
- `target_selection.enable_ocr`
- `target_selection.fallback_strategy`
- `target_selection.ocr_languages`
- `target_selection.ocr_use_gpu`
- `rgbd.require_aligned_depth_to_color`

---

## 11. 当前测试与验证状态

目前工程已经具备基础自动化测试，覆盖以下内容：

- `RGB-D` 反投影公式
- 位姿输出计算
- 辅助标签归一化
- 目标托盘选择逻辑
- `target_bbox -> depth ROI -> point cloud` 主路径
- 实时 pipeline 的 mock 调用
- ROS 工具函数的轻量验证

最近一轮验证结果：

```text
17 passed, 1 skipped
```

其中：

- `passed` 表示纯 Python 层面的核心逻辑已通过
- `skipped` 通常是当前环境没有 ROS 消息包时自动跳过的测试

---

## 12. 当前项目的优势

1. **主链路已经闭合**  
   从目标托盘选择到点云配准再到位姿输出，已经能形成完整链路。

2. **两种主接口可直接使用**  
   一个面向图片调试，一个面向 RealSense 实时视频流。

3. **支持提前指定辅助标签 ID**  
   这点非常适合多托盘并排场景。

4. **中间结果保存较完整**  
   出现问题时可以比较快定位在检测、OCR、深度、点云还是配准阶段。

5. **实时与离线共用同一套 pipeline**  
   不容易出现“离线能跑、实时不能复用”的两套逻辑分裂问题。

---

## 13. 当前已知限制

1. 项目目前仍是工程原型，不是完整量产版本。
2. OCR 对标签尺寸、反光、模糊和遮挡比较敏感。
3. 若场景深度空洞较多，目标 ROI 点云会很稀疏。
4. FPFH 在重复结构场景下可能出现误匹配。
5. ICP/GICP 仍可能受粗配准初值影响。
6. 不同托盘规格若和模板差异过大，配准会变差。
7. 当前 RealSense 实时模式主要验证单相机输入，尚未扩展多相机融合。
8. 当前主输出已经是相机系四参数，但部分车体系输出仍保留在内部和 JSON 中，用于兼容。

---

## 14. 当前还需要用户提供的真实数据

如果要真正跑通现场结果，仍需要用户提供：

1. 训练好的 `pallet_front` / `forkable_front` YOLO 权重
2. 带辅助标签的托盘图像或视频流
3. 提前指定的 `target_label`，例如 `A102`
4. 与 RGB 对齐的 `aligned depth`
5. 相机内参
6. 如需要车体系输出，则提供准确的 `T_B_C`
7. 托盘前表面局部模板点云 `template_front.ply`
8. 如果使用实时模式，则提供 `RealSense D435`
9. 如果使用 ROS 模式，则提供真实话题名称与 ROS 环境

---

## 15. 后续发展规划

下面是比较务实、按优先级可推进的后续规划。

### 15.1 第一阶段：把现有原型变成稳定可验收版本

目标：

- 用真实权重和真实 RGB-D 数据跑通整条链路
- 固定模板、配置和深度尺度
- 输出可重复的相机系 `x / y / z / yaw`

建议工作：

1. 补充一组标准离线测试数据
2. 为 `result.json` 定义更稳定的字段约定
3. 增加几组真实失败案例回归测试
4. 完成 RealSense 现场数据留存与回放

### 15.2 第二阶段：增强目标选择稳定性

目标：

- 提高多托盘场景下的目标绑定稳定性

建议工作：

1. 增加 OCR 结果缓存
2. 增加多帧投票机制
3. 增加标签区域自适应裁剪
4. 增加标签清晰度与可读性评分
5. 增加“未读到标签时保持上一目标”的可选策略

### 15.3 第三阶段：增强点云配准鲁棒性

目标：

- 提高复杂光照、深度空洞、重复结构下的成功率

建议工作：

1. 优化 ROI 过滤规则
2. 引入更稳健的法向筛选
3. 为不同托盘规格维护多模板库
4. 增加模板自动选择
5. 增加配准结果置信度评分
6. 增加粗配准失败后的二次尝试策略

### 15.4 第四阶段：增强实时性能

目标：

- 让实时模式更适合持续运行

建议工作：

1. YOLO、OCR、点云配准解耦成异步工作流
2. 引入处理队列和跳帧机制
3. 对 OCR 和模板点云预处理做缓存
4. 提高 `max_fps` 下的稳定性
5. 对中间图像保存做异步写盘

### 15.5 第五阶段：增强 ROS 与控制系统集成

目标：

- 让输出更容易直接进入 AGV 控制栈

建议工作：

1. 增加标准化 `PoseStamped` 和自定义状态码规范
2. 增加 TF 树一致性检查
3. 增加 rosbag 离线回放验收脚本
4. 增加 RViz 调试配置
5. 增加与导航、叉齿控制模块的接口定义

### 15.6 第六阶段：面向工程交付

目标：

- 从算法原型走向可部署系统

建议工作：

1. 完善日志分级和运行监控
2. 增加配置模板与版本管理
3. 补充更多自动化测试
4. 增加现场标定工具
5. 增加安装脚本和一键运行脚本
6. 增加异常追踪与运行报告输出

---

## 16. 推荐的下一步工作顺序

如果接下来要继续推进，建议按下面顺序做：

1. 用一组真实图片先跑通图片接口
2. 确认 `target_label -> target_bbox -> aligned depth ROI` 没有偏差
3. 用同一组数据检查点云预处理与配准质量
4. 跑通 RealSense 实时模式
5. 再接入 ROS1 Noetic
6. 最后做多场景、多托盘和长时间稳定性测试

---

## 17. 一句话总结

当前项目已经不是单纯的算法脚本集合，而是一个具备**目标托盘选择、RGB-D 点云处理、模板配准、相机系四参数输出、调试可视化和双接口运行方式**的工程化原型。  
它已经适合进入“真实数据打磨 + 现场联调 + 稳定性增强”的下一阶段。
