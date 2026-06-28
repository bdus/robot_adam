# 特征固化与多模态视觉感知栈详细规格说明书 (SPEC 04_Vision_Stack)

> 版本: v1.0 | 日期: 2026-06-28 | 状态: Working
> 范围：ORB-SLAM3 传统 VSLAM + DROID-SLAM 神经网络稠密 SLAM + grid_projector 网格投影 + 多源 EKF 终极融合
> 前置依赖：SPEC 02（2D 全栈）、SPEC 03（3D 激光栈）
> 总工期预估：~4 天 | 原子单元数：4

---

## 1. 范围与边界

### 1.1 本期目标

引入传统特征点 VIO（ORB-SLAM3）的强回环能力与神经网络 SLAM（DROID-SLAM）的极端鲁棒性，通过多源 EKF 终极合流解决单一传感器在低纹理、暗光场景下的退化问题，实现"黑夜无感运控"。

### 1.2 本期不包含

- 中枢 Lifecycle 状态机收官（SPEC 05）
- VLN 大模型决策与语义拓扑（SPEC 05）

### 1.3 输入依赖

| 依赖 | 来源 | 状态 |
|------|------|------|
| 双目/RGBD 相机话题 | robot_description | ✅ 已完成 |
| 相机内参标定文件 | SPEC 02 Unit 1 → adam_assets | ✅ 资产包已就绪 |
| `local_ekf_node` + `global_ekf_node` | SPEC 02 Unit 2 | 需先完成 |
| FAST-LIO2 `/fast_lio_odom` | SPEC 03 Unit 1 | 需先完成 |
| `adam_assets/maps_vision/` 词袋文件 | ORB-SLAM3 离线生成 | 需下载预训练权重 |

---

## 2. 原子单元拆解

本期拆解为 4 个最小可交付单元。

### 📦 Unit 1: 相机标定资产注入

**功能描述**：将仿真环境和真车相机的内参文件标准化写入 `adam_assets/camera_calibration/`，确保 ORB-SLAM3、DROID-SLAM 等视觉节点可通过 `get_asset_path()` 统一调阅。

**输入依赖**：无（标定数据可离线生成或取自 Gazebo 仿真参数）。

**产出物**：
- `adam_assets/camera_calibration/sim_camera_intrinsics.yaml`：仿真相机内参
- `adam_assets/camera_calibration/real_camera_intrinsics.yaml`：真车相机内参（占位）

**标定文件格式**：
```yaml
# sim_camera_intrinsics.yaml
camera_name: "sim_stereo"
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [530.0, 0.0, 320.0, 0.0, 530.0, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [530.0, 0.0, 320.0, 0.0, 0.0, 530.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

**通关标准**：`get_asset_path('camera_calibration', 'sim_camera_intrinsics.yaml')` 返回有效路径。

---

### 📦 Unit 2: ORB-SLAM3 词袋回环与 VIO 里程计

**功能描述**：加载 `adam_assets` 中的 `.bin` 特征点词袋文件，运行双目惯导模式（Stereo-Inertial），输出具备强回环修正、拍照即重定位能力的绝对视觉位姿。

**输入依赖**：
- 双目图像话题（`/left/image_raw`, `/right/image_raw`）
- `/imu/data`
- `adam_assets/camera_calibration/sim_camera_intrinsics.yaml`
- `adam_assets/maps_vision/orb_slam3_vocabulary.bin`（预训练词袋）

**产出物**：
- `config/orbslam3_stereo_inertial.yaml`：ORB-SLAM3 配置
- `launch/orbslam3_vio.launch.py`：启动脚本
- 话题输出：`/orbslam3_pose`（`geometry_msgs/msg/PoseWithCovarianceStamped`，含协方差）

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

**通关标准**：
1. 小车在场景中绕行一圈产生回环
2. 回环发生瞬间，ORB-SLAM3 触发词袋检测，累积误差瞬间拉平
3. `/orbslam3_pose` 产生修正阶跃，Rviz 中地图轨迹被拉回正确位置

---

### 📦 Unit 3: DROID-SLAM 神经网络稠密跟踪外壳

**功能描述**：利用宿主机 GPU 算力运行端到端神经网络 SLAM，利用稠密束调整（Dense BA）对抗剧烈运动模糊和低纹理大白墙。

**输入依赖**：RGB 图像话题（`/camera/image_raw`）

**架构约束（硬性规格）**：
- ⚠ DROID-SLAM 官方基于 PyTorch，Python GIL 锁会导致 ROS2 回调阻塞
- **必须**采用独立 C++ 推理进程（TensorRT 加速版）或独立 Python 进程，通过**共享内存（Shared Memory）**与 ROS2 Wrapper 通信
- **禁止**在 ROS2 回调函数中直接执行神经网络推理

**产出物**：
- `config/droid_slam_params.yaml`：推理参数（模型权重路径、图像尺寸、迭代次数）
- `droid_slam_node.py`：ROS2 Wrapper（共享内存通信层）
- 话题输出：`/droid_slam_odom`（`nav_msgs/msg/Odometry`）

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
```

**通关标准**：
1. GPU 跑通稠密光流跟踪，帧率 ≥10Hz
2. 剧烈晃动相机，DROID-SLAM 不丢失跟踪
3. 独立推理进程崩溃时，ROS2 Wrapper 不受影响（发布空协方差）

---

### 📦 Unit 4: grid_projector 稠密网格投影与多源 EKF 终极合流

**功能描述**：
- 将 DROID-SLAM 产出的三维稠密几何网格压缩降维，变为 2D Costmap 占用像素
- 在 `adam_localization` 中配置终极 `global_ekf_node`，融合 FAST-LIO2、ORB-SLAM3、DROID-SLAM 及底层轮速计

**输入依赖**：
- `/fast_lio_odom`（SPEC 03 Unit 1）
- `/orbslam3_pose`（SPEC 04 Unit 2）
- `/droid_slam_odom`（SPEC 04 Unit 3）
- 底层 `local_ekf_node` 输出的基础里程计

**产出物**：
- `grid_projector.py`：稠密网格 → 2D 占用栅格投影节点
- `config/ekf_global_ultimate.yaml`：多源终极融合 EKF 配置

**多源断流续命逻辑（退化降级状态机）**：
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

**通关标准**：
1. `grid_projector` 输出的占用栅格正确反映 3D 障碍物位置
2. 仿真中**强行关灯**，ORB-SLAM3 / DROID-SLAM 协方差爆炸
3. **铁律**：关灯瞬间，小车完全不减速、不撞墙，靠激光和轮速计稳健穿过暗室

---

## 3. 严格 Bottom-Up 构建与调试顺序

```
步骤 1: Unit 1 相机标定资产注入 (0.5天)
    ↓
步骤 2: Unit 2 ORB-SLAM3 VIO + 回环 (1.5天)
    ↓
步骤 3: Unit 3 DROID-SLAM 推理外壳 (1.5天)
    ↓
步骤 4: Unit 4 grid_projector + 终极 EKF 融合 (0.5天)
```

### 🛠️ 步骤 1：标定资产注入 — 0.5 天

**操作**：将仿真相机内参写入 `adam_assets/camera_calibration/`，编译后验证路径可访问。

### 🛠️ 步骤 2：ORB-SLAM3 VIO 闭环 — 1.5 天

**操作**：
1. 拉取 ORB-SLAM3 ROS2 封装
2. 下载预训练词袋文件，存入 `adam_assets/maps_vision/`
3. 启动双目图像 + IMU，在仿真中绕行

**通关验证**：
- Rviz 中观察回环检测修正轨迹

### 🛠️ 步骤 3：DROID-SLAM 推理外壳 — 1.5 天

**操作**：
1. 搭建共享内存通信层
2. 加载预训练权重
3. 测试剧烈运动下的跟踪稳定性

### 🛠️ 步骤 4：终极合流 — 0.5 天

**操作**：
1. 编写 `grid_projector.py`
2. 配置 `ekf_global_ultimate.yaml`
3. 关灯测试降级

**通关验证**：
- 关灯后小车不撞墙

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

| 单元 | 产出 | 通关标准 | 工期 |
|------|------|---------|------|
| Unit 1 | 相机标定资产 | `get_asset_path()` 可检索 | 0.5d |
| Unit 2 | ORB-SLAM3 VIO | 回环检测修正累积误差 | 1.5d |
| Unit 3 | DROID-SLAM 推理壳 | ≥10Hz，剧烈抖动不丢 | 1.5d |
| Unit 4 | grid_projector + EKF | 关灯不撞墙，降级无感 | 0.5d |
