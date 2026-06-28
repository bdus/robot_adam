# 3D 空间几何与时空立体避障详细规格说明书 (SPEC 03_3D_Spatial)

> **版本**: v1.0 | **日期**: 2026-06-28 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/03_3d_spatial_and_stvl.md`
> **范围**：FAST-LIO2 编译与参数对齐 + Level 2 全局 EKF 退化保护机制 + `pointcloud_to_laserscan` 高度切片 + Nav2 STVL (时空体素层) 动态残影消除。
> **关联总设计**：[`docs/spec/01_robot_adam_navigation_architecture.md`](../../../spec/01_robot_adam_navigation_architecture.md) — 本系列宏观架构纲领。
> **前置依赖**：`SPEC 01`（宏观架构），`SPEC 02`（2D 激光与常驻 EKF 已通）。Level 1 仿真底盘已挂载固态雷达 Livox Mid360。
> **总工期预估**：3 天 | **原子交付单元数**：3

---

## 1. 本期范围与边界 (Scope & Boundary)

### 1.1 本期开发目标

将机器人感知维度由 2D 栅格升维至 3D 空间几何。利用固态雷达 Mid360 跑通 **"3D 雷达惯导紧耦合状态估计 → 空间高度切片投影伪激光 → 三维时空体素悬空避障 → 动态行人残影自动衰减"** 的完整流水线，并建立严密的里程计物理退化降级保护。

### 1.2 本期严格不包含

- 传统特征点视觉 VIO（ORB-SLAM3）与神经网络稠密 SLAM（SPEC 04）。
- 顶层控制中枢 Lifecycle 状态机逻辑管理、大模型具身语义层（SPEC 05）。

---

## 2. 最小原子交付单元拆解 (Sub-Agent Units)

### 📦 Unit 1: FAST-LIO2 紧耦合状态估计调优与 Odom 退化保护

**功能描述**：源码编译社区 ROS2 Humble 移植版 FAST-LIO2。消费 Mid360 的点云与内置高频 IMU，输出高频 3D 里程计。在 Level 2 的全局 EKF 中建立基于协方差的退化保护。

**数据流拓扑与输入输出**：

- **输入话题**：`/livox/lidar`（定制 `LivoxCustomMsg` 格式或标准点云）与 `/livox/imu`（≥200Hz）。
- **输出话题**：`/fast_lio_odom`（`nav_msgs/msg/Odometry`）。

**Level 2 终极 EKF 退化分流硬性规格**：

- `local_ekf_node` 继续常驻融合轮速计与物理 IMU，发布稳定的 `odom -> base_link` TF。**重要**：`odom -> base_link` 是 Level 2 的输出，不是 FAST-LIO2 的输出。FAST-LIO2 在数学上计算了"雷达相对于里程计起点的相对位姿"，但**禁止让它广播 `odom -> base_link` TF**。因为 SLAM 算法计算量大、遇到空旷/剧烈晃动会丢帧降频，若 Nav2 MPPI 直接吃 SLAM 的 TF 会引发急刹失控。正确分工：FAST-LIO2 = 纯 Topic 输出（`/fast_lio_odom`），`local_ekf_node` = 独裁广播器，唯一发布 `odom -> base_link` TF，确保运控 50Hz+ 绝对不断流。
- `global_ekf_node` 接收 `/fast_lio_odom` 作为绝对位姿观测源，发布 `map -> odom` TF。

**降级硬性逻辑**：写一个轻量级监控节点 `odom_health_monitor`。实时订阅 `/fast_lio_odom`，提取其 `pose.covariance` 矩阵对角线元素（特别是 X, Y, Z 轴方差）。当方差均 ≤0.05 时，Global EKF 100% 信任 Fast-LIO2；一旦方差突变（≥0.5 或收到非数 NaN），监控节点通过服务或动态参数降低 Global EKF 中该位姿源的权重，将整车运控降级切换至底层常驻的轮速里程计，防止整车漂移撞墙。

**关键参数**：
```yaml
# FAST-LIO2 参数
lio:
  ros__parameters:
    # 点云预处理
    point_filter_num: 4               # 降采样步长
    filter_size_surf: 0.5             # 面特征体素滤波尺寸
    filter_size_map: 0.5              # 地图体素滤波尺寸

    # 传感器外参
    extrinsic_est_en: false           # 是否在线估计外参（false=使用给定值）
    lidar_to_imu_init: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    # 噪声协方差（IKFoM 初始值）
    imu_gyro_cov: 0.001
    imu_acc_cov: 0.01
    lidar_point_cov: 0.001
```

---

### 📦 Unit 2: pointcloud_to_laserscan 空间高度切片投影器

**功能描述**：防止 3D 全局寻路对全局 3D 点云进行实时 A* 搜索而导致算力崩盘。将 Mid360 稠密点云在底盘上方特定垂直高度区间进行水平截取，降维投影为 2D 伪激光，作为 Nav2 全局路径规划的安全边界。

**切片硬性规格**：

- **输入**：`/livox/lidar_pc2`（标准 `sensor_msgs/msg/PointCloud2`）。
- **输出**：`/scan_3d_projected`（`sensor_msgs/msg/LaserScan`）。
- **参数对齐**：
  - `min_height = 0.10`（底盘上方 10cm，避开地面噪声）
  - `max_height = 0.40`（底盘上方 40cm，覆盖小车自身通过高度）
  - `range_max = 30.0`
  - `angle_increment: 0.00872665`（~0.5°）
  - `angle_min: -3.14159` / `angle_max: 3.14159`（360° 全向）

---

### 📦 Unit 3: Nav2 Costmap STVL (时空体素插件) 集成

**功能描述**：解决传统 2D 栅格障碍物层无法表达悬空物体（如倾斜桌腿、桌面上悬空的显示器），以及动态行人走过留下"永久残影导致小车死锁"的痛点。

**架构配置 (`nav2_stvl_config.yaml`)**：
在 Nav2 的 `local_costmap` 和 `global_costmap` 的 `plugins` 中，卸载旧的 `obstacle_layer`，挂载 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`。

**核心参数锁死**：
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        publish_voxel_map: true

        # 体素空间分辨率
        voxel_size: 0.05              # 5cm 体素
        voxels_size_x: 40             # 2m 宽度
        voxels_size_y: 40             # 2m 长度
        voxels_size_z: 20             # 1m 高度

        # 时空衰减参数（核心）
        voxel_decay: 1.0              # 体素 1.0 秒自动消散
        decay_model: 1                # 0=瞬时, 1=线性, 2=指数
        time_scale: 1.0

        # 传感器源
        observation_sources: "lidar"
        lidar:
          topic: /livox/lidar
          data_type: "PointCloud2"
          expected_update_rate: 10.0
          min_obstacle_height: 0.05
          max_obstacle_height: 1.0
```

---

## 3. Bottom-Up 路线图与单点测试验收方案 (Gate Criteria)

测试必须在仿真/真实物理环境中严格按照以下 3 步推进，并闭环通过白纸黑字的测试用例。

```
[步骤 1: 3D LIO 调通 + 协方差退化测试]
                 │
                 ▼
[步骤 2: 空间切片降维 + 悬空几何提取验收]
                 │
                 ▼
[步骤 3: STVL 挂载 + 动态残影一秒消散验收]
```

---

### 🛠️ 步骤 1：3D LIO 状态估计调通与几何退化降级保护测试

**操作方法**：一键拉起 FAST-LIO2。控制小车在 Gazebo 场景中高速前行、做 360° 剧烈原地自旋。随后，控制小车进入**极端退化场景：长达 30 米、没有任何几何起伏的纯大白墙走廊**，或用脚本强行将 `/livox/lidar` 点云话题断流。

**验收方案与白盒标准（Gate 1）**：

1. **高频验证**：使用 `ros2 topic hz /fast_lio_odom`，其输出频率必须稳定在 **≥100Hz** 且无丢包。
2. **剧烈运动验证**：小车在剧烈自旋时，Rviz 中的点云地图绝不允许出现明显的断层或分层（即当前帧点云与历史地图重合度 >95%）。
3. **退化降级终极测试**：当进入大白墙走廊或点云断流瞬间，观察终端日志，监控节点必须在 **50 毫秒内**识别出协方差发散（方差 >0.5），Global EKF 必须顺利完成降级保护。
4. **通关铁律**：重定位和 TF 树（`map -> odom`）在此断流期间**绝对不允许发生超过 5 厘米的瞬间阶跃，底盘不闪退，整车靠数轮子格子继续保持平滑前行**。

---

### 🛠️ 步骤 2：空间切片降维与悬空几何提取验收

**操作方法**：在仿真场景中，人为放置几个特殊障碍物：**一张普通的四腿桌子（桌腿很细），以及一块悬空在底盘上方 25cm 处的横板（地面完全悬空）**。启动 `pointcloud_to_laserscan` 节点，在 Rviz 中订阅 `/scan_3d_projected` 话题。

**验收方案与白盒标准（Gate 2）**：

1. **细小特征提取验证**：小车静止在桌子旁，投影出的伪激光 `/scan_3d_projected` 必须能精准呈现 4 个离散的、直径与桌腿物理尺寸相符的伪雷达特征点。
2. **悬空障碍物捕获验证**：传统 2D 雷达由于光束高度固定，会完全无视悬空横板从而直接钻入。此时，投影伪激光必须在横板对应区域呈现连续的激光弧线。
3. **通关铁律**：Rviz 中观察 `/scan_3d_projected`，在小车行走过程中，**绝不允许出现因为地面不平、车身颠簸把"地面"误识别为障碍物的噪点点云**（若有，说明 `min_height` 设矮了，必须调大到避开地噪）。

---

### 🛠️ 步骤 3：STVL 时空体素集成与动态行人残影消散测试

**操作方法**：拉起配置了 STVL 的 Nav2 全栈导航。在 Rviz 中开启 3D Costmap 渲染。在小车前方下发一个 10 米外的目标点。在小车开始全速前行的路线正前方，控制一个仿真动态行人（Actor）从左向右横穿通过，随后让行人迅速离开小车视野。

**验收方案与白盒标准（Gate 3）**：

1. **立体防撞验证**：当小车开向上述步骤 2 放置的悬空横板时，Nav2 的本地代价图（Local Costmap）必须立刻被三维体素填充为红色高代价区，Smac 规划器重新寻路绕行，底盘绝不撞击悬空物体。
2. **动态残影自动消散终极验证**：当行人在小车正前方走过时，3D 体素层会实时在局部代价图上留下行人的移动轨迹。
3. **通关铁律**：当行人完全离开小车视野的瞬间（以行人的最后一帧点云消失开始计时），代价图上由于行人走过留下的红色高风险"残影体素"，**必须在 1.0 秒内（容差 ±0.1 秒）像烟雾一样完全自动老化消散，代价图恢复纯净的空地状态**。Nav2 MPPI 控制器绝不允许因为历史残影未消散而在空无一人的空地上发生"紧急刹车顿挫"或"原地死锁行为"。

---

## 4. 依赖安装

```bash
# pointcloud_to_laserscan（apt 可用）
sudo apt install ros-humble-pointcloud-to-laserscan

# FAST-LIO2（需源码编译 ROS2 Humble 社区分支）
git clone https://github.com/EmarUn/fast_lio_rviz  # 或等效稳定 ROS2 移植
# 或 git clone https://github.com/lifegpc/FAST_LIO -b ros2

# STVL（需源码编译）
git clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git
```

---

## 5. 交付标准总览

| 单元 | 产出物 | 通关标准 | 工期 |
|------|-------|---------|------|
| Unit 1 | FAST-LIO2 里程计 + odom_health_monitor | ≥100Hz，退化协方差 50ms 内识别降级，TF 阶跃 <5cm | 1d |
| Unit 2 | pointcloud_to_laserscan 切片 | 桌腿离散特征 + 悬空横板连续弧线，地面噪点零容忍 | 0.5d |
| Unit 3 | STVL 时空避障 | 体素 1.0s ±0.1s 消散，动态行人无残影死锁 | 1.5d |
