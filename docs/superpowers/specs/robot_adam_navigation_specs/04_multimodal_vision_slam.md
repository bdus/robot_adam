# 特征固化与多模态视觉感知栈详细规格说明书 (SPEC 04_Vision_Stack)

> **版本**: v1.0 | **日期**: 2026-06-28 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/04_multimodal_vision_slam.md`
> **范围**：ORB-SLAM3 双目惯导集成与 `.bin` 词袋静态索引 + DROID-SLAM 共享内存（Shared Memory）隔离封装外壳 + `grid_projector` 稠密网格投影器 + Level 2 终极多源合流 EKF 断流保护。
> **关联总设计**：[`docs/spec/01_robot_adam_navigation_architecture.md`](../../../spec/01_robot_adam_navigation_architecture.md) — 本系列宏观架构纲领。
> **前置依赖**：`SPEC 01`（宏观架构），`SPEC 02` 与 `SPEC 03` 稳定运行（2D、3D 雷达和底座 EKF 已全部就绪）。Level 1 仿真底盘已挂载双目相机与高频物理 IMU。
> **总工期预估**：4 天 | **原子交付单元数**：3

---

## 1. 本期范围与边界 (Scope & Boundary)

### 1.1 本期开发目标

引入多模态视觉感知作为系统的高级位置冗余源。打通 **"双目惯导强回环 VIO + 端到端神经稠密光流跟踪 + 3D 稠密网格压缩降维投影 + Level 2 级多源定位合流"**。系统需具备在极端物理致盲场景（如突入全黑暗室、摄像头被恶意完全遮挡）下的**无感断流续命机制**，确保整车运控不减速、位姿不跃迁。

### 1.2 本期严格不包含

- 顶层控制中枢 Lifecycle 状态机逻辑管理、大模型语义 VLN 层（SPEC 05）。

---

## 2. 最小原子交付单元拆解 (Sub-Agent Units)

### 📦 Unit 1: ORB-SLAM3 词袋回环与视觉里程计 (VIO)

**功能描述**：编译并封装 ORB-SLAM3。使用双目 + 惯导（Stereo-Inertial）模式。静态词袋文件（`ORBvoc.bin`）必须通过 `adam_assets` 提供的 Python 路由进行动态加载。

**数据流接口**：

- **输入**：双目图像话题 `/camera/left/image_raw`、`/camera/right/image_raw`，以及高频物理 IMU `/camera/imu`（≥200Hz）。
- **输出**：绝对位姿话题 `/orbslam3_pose`（`geometry_msgs/msg/PoseWithCovarianceStamped`）。**其参数 `publish_tf` 强制关闭（设为 false）**。

**相机标定资产注入**：

- 在 `adam_assets/share/camera_calibration/` 下放置仿真相机内参文件 `sim_camera_intrinsics.yaml` 与真车占位文件 `real_camera_intrinsics.yaml`。
- 标定文件包含：`camera_matrix`、`distortion_coefficients`、`rectification_matrix`、`projection_matrix`，格式遵循 ROS2 `camera_info_manager` 规范。

**关键参数**：
```yaml
# ORB-SLAM3 双目惯导模式
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8

# 双目匹配
Camera.stereo: true
Camera.stereo.b: 0.075               # 双目基线 7.5cm

# IMU 参数
IMU.Frequency: 200.0
IMU.GyroNoiseSigma: 0.001
IMU.AccNoiseSigma: 0.01

# 回环检测
LoopClosing.enable: true
LoopClosing.minScore: 0.4
```

---

### 📦 Unit 2: DROID-SLAM 神经网络稠密跟踪与 Python 共享内存隔离外壳

**功能描述**：部署端到端深度学习 SLAM DROID-SLAM。由于该算法基于 PyTorch 且存在高频 CUDA 密集求导，为防止 Python 全局解释器锁（GIL）阻塞 ROS2 核心的回调线程（Callback Queue），**必须设计物理进程隔离与通信外壳**。

**隔离架构设计**：

- **后端进程（Python 独立进程）**：全速运行 DROID-SLAM 神经网络。将接收到的相机图像写入系统**共享内存（Shared Memory / IPC）**。
- **前端节点（C++ ROS2 Wrapper）**：常驻 ROS2 空间，负责消费相机话题，以零拷贝（Zero-Copy）方式将图像塞入共享内存。同时，从共享内存中读取 Python 后端吐出的最新三维稠密几何网格（Dense Mesh）与相机 Pose。
- **输出**：发布神经里程计话题 `/droid_slam_odom` 与 3D 稠密网格 `/droid_slam_mesh`。

**架构拓扑**：
```
相机话题 → ROS2 Wrapper (droid_slam_node.py)
                          │
                    共享内存 (Shared Memory)
                          │
           独立推理进程 (TensorRT / PyTorch)
                ┌─────────────────┐
                │ DROID-SLAM 内核  │
                │ (Dense BA 迭代)  │
                └─────────────────┘
                          │
                    共享内存
                          │
                  ↓ /droid_slam_odom
                  ↓ /droid_slam_mesh
```

**关键参数**：
```yaml
# DROID-SLAM 推理参数
droid_slam:
  ros__parameters:
    model_path: "$(find adam_assets)/share/maps_vision/droid.pth"
    image_width: 640
    image_height: 480
    max_frames: 12              # 滑动窗口帧数
    inference_iterations: 32    # Dense BA 迭代次数
```

---

### 📦 Unit 3: grid_projector 稠密网格投影器与 Level 2 终极多源断流合流

**功能描述**：编写 `grid_projector` 节点，将三维稠密网格中在底盘通过高度范围内的三维点，降维压缩投影为 2D 占用像素图注入 Nav2 Costmap。同时，在 `adam_localization` 层的全局 EKF 中实现多源融合防致盲。

**Level 2 终极多源合流规格**：

- 扩展并升级 `global_ekf_node`。输入源扩展为：`[底层常驻轮速+IMU EKF (Local)]`、`[3D FAST-LIO2 (Lidar)]`、`[ORB-SLAM3 (Vision)]`、`[DROID-SLAM (Neural)]`。
- **断流续命硬核逻辑**：Global EKF 建立多通道卡方检验（Chi-Square Test）马氏距离门限过滤器。当视觉源（ORB-SLAM3/DROID-SLAM）由于环境光照突变导致 Tracking Lost 或协方差对角线元素跃迁（>1.0）时，全局 EKF 必须在单帧回调周期内（≤10 毫秒）将该视觉源的观测权重归零（拒绝合流），位置基准完全由常驻的 3D 雷达或底层 Local EKF 承托。

**多源断流续命退化降级状态机**：
```
正常状态：global_ekf_node 融合 [FAST-LIO2 + ORB-SLAM3 + DROID-SLAM + 轮速计]
    │
    ├── ORB-SLAM3 协方差爆炸 (低纹理) → 剔除视觉，保留 [FAST-LIO2 + DROID-SLAM + 轮速计]
    │
    ├── DROID-SLAM 协方差爆炸 (全黑)   → 剔除神经视觉，保留 [FAST-LIO2 + 轮速计]
    │
    ├── FAST-LIO2 协方差爆炸 (长走廊)   → 剔除激光，保留 [轮速计 + IMU]（最保守模式）
    │
    └── 所有外部源丢失                  → 纯轮速计 + IMU 航位推算（不死）
```

---

## 3. Bottom-Up 路线图与单点测试验收方案 (Gate Criteria)

开发代理或工程师必须无条件遵循以下调试顺序，闭环通过白纸黑字的测试用例。

```
[步骤 1: 视觉回环与重定位锁定验收]
                 │
                 ▼
[步骤 2: 神经 SLAM 共享内存低延迟与 GIL 隔离验收]
                 │
                 ▼
[步骤 3: 极端物理致盲 —— 黑暗室断流续命终极合流验收]
```

---

### 🛠️ 步骤 1：ORB-SLAM3 词袋回环与重定位锁定测试

**操作方法**：一键拉起 ORB-SLAM3 双目惯导链路。控制小车在 Gazebo 复杂多房间场景中自由行驶，产生一条长达 30 米且包含多次交叉重叠的轨迹（构造累计误差）。小车绕行一圈回到起点（触发全局回环检测）。随后，在小车静止状态下，通过仿真器**强行遮挡相机 5 秒后放开**（测试拍照即重定位能力）。

**验收方案与白盒标准（Gate 1）**：

1. **回环阶跃验证**：当小车回到起点，ORB-SLAM3 内部触发 DBoW 词袋匹配成功瞬间，在 `/orbslam3_pose` 的累积误差必须发生瞬间阶跃拉平，后端图优化完成误差分摊。
2. **拍照重定位时效**：放开相机遮挡后，算法必须在 **500 毫秒内**基于当前视场特征点与历史关键帧词袋对齐，瞬间重新锁死位姿跟踪（Tracking OK），无位姿发散。
3. **通关铁律**：整个测试过程中，`/orbslam3_pose` 的发布频率必须稳定在 **≥20Hz**。

---

### 🛠️ 步骤 2：DROID-SLAM 共享内存通信低延迟与 GIL 隔离验收

**操作方法**：一键拉起神经网络 SLAM。控制小车全速前行，并发布高频耗时任务（如故意向 ROS2 核心回调队列中注入大量的假高频广播话题）。

**验收方案与白盒标准（Gate 2）**：

1. **低延迟验证**：测量前端 C++ 节点接收到相机图像，到 Python 后端处理完毕并将位姿写回共享内存的**端到端延迟，必须 ≤45 毫秒**。
2. **GIL 锁隔离终极测试**：使用 `top` 或 `htop` 监控。
3. **通关铁律**：即使 Python 后端因为深度学习复杂后端优化导致单帧求导卡顿、甚至因显存溢出而假死（SEGFAULT），**前端的 C++ ROS2 Wrapper 节点必须表现出完全的物理免疫。ROS2 核心线程的 CPU 占用率曲线不能有任何波动，节点绝不允许发生线程死锁或被 Python 进程拖垮闪退**。

---

### 🛠️ 步骤 3：极端物理致盲 —— 暗室多源断流续命终极合流测试

**操作方法**：控制小车以 1.0m/s 的高线速度向目标点全速前行。在小车运行到行进路线正中间时，通过仿真器控制环境光照，**在 0.1 秒内将全场环境光照调为 0（即瞬间强行关灯、进入全黑暗室环境）**，致使双目相机画面完全变黑。

**验收方案与白盒标准（Gate 3）**：

1. **数据断流观测**：在关灯瞬间，通过 `ros2 topic echo` 观测，ORB-SLAM3 与 DROID-SLAM 必须由于视觉特征点归零而瞬间断流、抛出 Lost 日志或协方差直接飙升至无穷大。
2. **马氏距离门限验证**：检查 `adam_localization` 全局 EKF 的内部日志，滤波器必须在 **10 毫秒内（单帧周期）** 触发卡方检验拒绝，强行切断视觉输入流。
3. **★ 全栈终极通关铁律 ★**：在全黑暗室中，由于有 Level 2 常驻的 3D FAST-LIO2 雷达里程计与底层轮速 EKF 的强力筑基托底，**小车模型在 Rviz 中的全局位姿绝不允许发生任何超过 3 厘米的跃迁。小车的底盘运控绝不允许发生任何急刹、减速、顿挫或撞墙行为，必须保持原有的 1.0m/s 线速度，四平八稳地盲开穿过黑暗区，实现多模态具身状态估计的极致鲁棒性**。

---

## 4. 依赖安装

```bash
# ORB-SLAM3 ROS2
git clone https://github.com/thien94/ORB_SLAM3_ROS2.git

# DROID-SLAM（需 GPU）
git clone https://github.com/princeton-vl/DROID-SLAM.git
# 下载预训练权重
wget https://dl.fbaipublicfiles.com/droid-slam/droid.pth

# 词袋文件（ORB-SLAM3）
# 需从 ORB-SLAM3 官方仓库下载 ORBvoc.txt / ORBvoc.bin
```

---

## 5. 交付标准总览

| 单元 | 产出物 | 通关标准 | 工期 |
|------|-------|---------|------|
| Unit 1 | ORB-SLAM3 VIO 节点 + 相机标定资产 | 回环修正阶跃，遮挡恢复 ≤500ms，≥20Hz | 1.5d |
| Unit 2 | DROID-SLAM 共享内存外壳 | 端到端延迟 ≤45ms，GIL 完全隔离，Python 崩溃不影响 ROS2 | 1.5d |
| Unit 3 | grid_projector + 多源 EKF 终极合流 | 关灯 10ms 内切断视觉，位姿跃迁 <3cm，1.0m/s 不减速盲开 | 1d |
