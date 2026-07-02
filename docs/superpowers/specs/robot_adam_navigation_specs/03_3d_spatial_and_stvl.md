# 3D 空间几何与时空立体避障详细规格说明书 (SPEC 03_3D_Spatial)

> **版本**: v1.1 | **日期**: 2026-06-28 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/03_3d_spatial_and_stvl.md`
> **范围**：FAST-LIO2 编译与参数对齐 + Level 2 全局 EKF 退化保护机制 + `pointcloud_to_laserscan` 高度切片（仅用于Global Planner） + Nav2 STVL (时空体素层) 用于Local Costmap 的动态残影消除。
> **关联总设计**：[`docs/spec/01_robot_adam_navigation_architecture.md`](../../../spec/01_robot_adam_navigation_architecture.md) — 本系列宏观架构纲领。
> **前置依赖**：`SPEC 01`（宏观架构），`SPEC 02`（2D 激光与常驻 EKF 已通）。Level 1 仿真底盘已挂载固态雷达 Livox Mid360。
> **基准测试变体**：`mid360_2wd`。该变体挂载了 Livox Mid360 激光雷达和 IMU，满足 3D 空间感知的所有需求。`laser_*` 系列变体不含 3D 雷达，不在本 SPEC 的测试范围内。
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

**功能描述**：源码编译 FAST-LIO2（bdus/FASTLIO2_ROS2 fork），消费 Mid360 的点云（CustomMsg 格式）与内置高频 IMU，输出高频 3D 里程计（含 IESKF 协方差矩阵）。在 Level 2 的全局 EKF 中建立基于协方差的退化保护。**FAST-LIO2 仅输出纯 Topic（`/slam_pose/fast_lio`），不广播任何 TF**，由 Level 2 `local_ekf_node` 独裁维护 `odom -> base_link` TF。

**数据流拓扑与输入输出**：

- **输入话题**：`/livox/lidar`（定制 `LivoxCustomMsg` 格式或标准点云）与 `/livox/imu`（≥200Hz）。
- **输出话题**：`/slam_pose/fast_lio`（`nav_msgs/msg/Odometry`）。

**Level 2 终极 EKF 退化分流硬性规格**：

- `local_ekf_node` 继续常驻融合轮速计与物理 IMU，发布稳定的 `odom -> base_link` TF。**重要**：`odom -> base_link` 是 Level 2 的输出，不是 FAST-LIO2 的输出。FAST-LIO2 在数学上计算了"雷达相对于里程计起点的相对位姿"，但**禁止让它广播 `odom -> base_link` TF**。因为 SLAM 算法计算量大、遇到空旷/剧烈晃动会丢帧降频，若 Nav2 MPPI 直接吃 SLAM 的 TF 会引发急刹失控。正确分工：FAST-LIO2 = 纯 Topic 输出（`/slam_pose/fast_lio`），`local_ekf_node` = 独裁广播器，唯一发布 `odom -> base_link` TF，确保运控 50Hz+ 绝对不断流。
- `global_ekf_node` 接收 `/slam_pose/fast_lio` 作为绝对位姿观测源，发布 `map -> odom` TF。

**`ekf_global.yaml` 扩展变更要点（SPEC 02 → SPEC 03）**：

在 SPEC 02 的 `global_ekf_node` 配置基础上，新增 FAST-LIO2 位姿源为 `odom1`：

```yaml
# ekf_global.yaml 变更（SPEC 03 新增部分）
ekf_filter_node:
  ros__parameters:
    odom1: /slam_pose/fast_lio
    odom1_config: [true, true, false,    # x, y, z 位置（观测X和Y，不观测Z）
                   false, false, true,   # roll, pitch, yaw（仅观测Yaw）
                   false, false, false,  # vx, vy, vz
                   false, false, false]  # vroll, vpitch, vyaw
    odom1_differential: false              # 绝对观测模式，不做差分
    odom1_queue_size: 10
```

> **注意**：SPEC 02 中的 `odom0` 仍对应 `/slam_pose/cartographer`（2D 位姿源）。当 SPEC 03 更新后，`odom0` 与 `odom1` 同时接入，Global EKF 融合两者的绝对位姿观测。`odom_health_monitor` 负责按协方差实时调节各源权重。

**降级硬性逻辑**：写一个轻量级监控节点 `odom_health_monitor`。实时订阅 `/slam_pose/fast_lio`，提取其 `pose.covariance` 矩阵对角线元素（特别是 X, Y, Z 轴方差）。当方差均 ≤0.05 时，Global EKF 100% 信任 Fast-LIO2；一旦方差突变（≥0.5 或收到非数 NaN），监控节点通过服务或动态参数降低 Global EKF 中该位姿源的权重，将整车运控降级切换至底层常驻的轮速里程计，防止整车漂移撞墙。

**关键参数（对应 `fastlio2/config/lio.yaml`）**：

```yaml
# FAST-LIO2 参数 — 与 FASTLIO2_ROS2 (fork: bdus/FASTLIO2_ROS2) 的 lio.yaml 格式对齐
# 注意：FAST-LIO2 使用独立 YAML 文件通过 config_path 参数传入，非 ROS2 parameter 风格

# 话题绑定（与 Livox driver multi_topic=0 模式对齐）
imu_topic: /livox/imu
lidar_topic: /livox/lidar
body_frame: body
world_frame: lidar

# 点云预处理
lidar_filter_num: 6                   # 降采样步长（每隔 N 点取 1）
lidar_min_range: 0.5                  # 最小有效距离（米）
lidar_max_range: 30.0                 # 最大有效距离（米）
scan_resolution: 0.15                 # 扫描体素滤波分辨率
map_resolution: 0.3                   # 地图 IKDTree 体素分辨率

# 空间管理
cube_len: 300                         # 局部地图立方体边长（米）
det_range: 60                         # 探测范围（米）
move_thresh: 1.5                      # 地图移动阈值

# IMU 噪声协方差（IKFoM 初始值，越大概率越信任）
na: 0.01                              # 加速度随机游走
ng: 0.01                              # 陀螺仪随机游走
nba: 0.0001                           # 加速度偏置随机游走
nbg: 0.0001                           # 陀螺仪偏置随机游走

# 初始化
imu_init_num: 20                      # IMU 初始化用帧数
near_search_num: 5                    # 最近邻搜索数
ieskf_max_iter: 5                     # IESKF 最大迭代次数
gravity_align: true                   # 是否利用重力方向对齐初始姿态

# 外参（LiDAR → IMU）
esti_il: false                        # 是否在线估计外参（false=使用给定值）
r_il: [1.0, 0.0, 0.0,                # 旋转矩阵（3x3）
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0]
t_il: [-0.011, -0.02329, 0.04412]    # 平移向量（米）

# 观测
lidar_cov_inv: 1000.0                 # 激光点观测信息矩阵（协方差逆）
```

> **注意**：IMU 加速度在 `lio_node.cpp:125` 被乘以 10.0 因子（适配 Livox IMU 输出单位），`lio_node.cpp:67` 的 timer 周期为 10ms（目标 ≥100Hz 输出）。

---

### 📦 Unit 2: pointcloud_to_laserscan 空间高度切片投影器

**功能描述**：防止 3D 全局寻路对全局 3D 点云进行实时 A* 搜索而导致算力崩盘。将 Mid360 稠密点云在底盘上方特定垂直高度区间进行水平截取，降维投影为 2D 伪激光，作为 Nav2 全局路径规划的安全边界。

**为什么需要 pointcloud_to_laserscan 与 STVL 共存**：
虽然 STVL 3D 时空体素网格能够提供局部避障的时空残影记忆，但在工业落地场景中，Global Costmap 依然需要 pointcloud_to_laserscan 作为轻量级实时输入，原因有三：

1. **解决"全局规划器看不到实时动态障碍物"的算力灾难**：
   Global Costmap 需要实时感知动态障碍物（如行人、移动的箱子）以避免长期路线被堵死。直接使用原始 3D 点云会导致算力崩盘，而 pointcloud_to_laserscan 通过高度切片产生的轻量级伪激光话题（数据量相当于传统 2D 雷达），能够以极低算力高频更新 Global Costmap，确保动态障碍物能被及时纳入全局规划。

2. **STVL 的时空衰减机制不适合用于全局导航**：
   STVL 的核心优势在于其时空残影消散机制（ voxel_decay: 1.0 秒自动消失），这对于局部避障是理想的，但灾难性地不适合全局规划。全局规划器需要障碍物在静态地图上长期存在，除非明确观测到消失否则不能自行清除。若将 STVL 用于 Global Costmap，机器人转头时 10 米外的障碍物会因时空衰减而消失，导致全局规划器误判路径畅通而规划撞墙轨迹。

3. **真车颠簸与地面"噪点陷阱"（工业落地的最痛点）**：
   真车行驶中底盘俯仰抖动会导致雷达束频繁扫到地面，若未过滤则地面噪点会被误认为障碍物。height slice (min_height=0.10, max_height=0.40) 作为物理防火墙，可过滤掉低于 10cm 的地面噪点（如电线、厚地毯）和因车身俯仰带来的虚假障碍，确保投影出的伪激光仅包含真正需要规避的硬核障碍物。

**切片硬性规格**：
**重要说明**：高度切片范围 (min_height=0.10m, max_height=0.40m) 仅用于 **全局规划器（Global Planner）**。在局部代价图（Local Costmap）中，必须保留全量或更大范围的 3D STVL 体素网格输入，依靠 MPPI 控制器在前向 56 步的时空推演中实施强行刹车或绕行。

- **输入**：`/livox/lidar`（`sensor_msgs/msg/PointCloud2`，需 `xfer_format = kPointCloud2Msg`）。
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

**架构设计**：
- **全局代价图（Global Costmap）**：使用传统的 `obstacle_layer`，输入源为 `pointcloud_to_laserscan` 生成的 `/scan_3d_projected` 话题。这是为了确保全局规划器能够实时感知动态障碍物而不受时空衰减影响。
- **局部代价图（Local Costmap）**：卸载旧的 `obstacle_layer`，挂载 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`，以获得时空体素的动态残影消散能力，专门用于处理悬空物体等 2D 雷达盲区。

**架构配置 (`nav2_stvl_config.yaml`)**：
在 Nav2 的 `local_costmap` 的 `plugins` 中，卸载旧的 `obstacle_layer`，挂载 `spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`。
（Global Costmap 保持使用标准 `obstacle_layer`，输入为 `/scan_3d_projected`）

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

1. **高频验证**：使用 `ros2 topic hz /slam_pose/fast_lio`，其输出频率必须稳定在 **≥100Hz** 且无丢包。
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
3. **通关铁律**：当行人完全离开小车视野的瞬间，代价图上由于行人走过留下的红色高风险"残影体素"，**必须在 1.0 秒内（容差 ±0.1 秒）像烟雾一样完全自动老化消散，代价图恢复纯净的空地状态**。Nav2 MPPI 控制器绝不允许因为历史残影未消散而在空无一人的空地上发生"紧急刹车顿挫"或"原地死锁行为"。

   **消散计时量化方法**：
   - **计时起点**：以 STVL 层最后收到行人点云的传感器时间戳为准（即 observation_sources 的 expected_update_rate 超时时刻），而非人眼主观判断。
   - **消散完毕判定**：订阅 STVL 发布的体素地图话题，统计行人轨迹对应区域的存活体素计数。当计数归零时即为消散完毕时刻。
   - **自动化思路**：编写轻量级验收脚本，订阅体素话题记录消散曲线，自动判定是否满足 1.0s±0.1s 约束，避免人眼观察 Rviz 的主观误差。

---

## 4. 依赖安装

```bash
# pointcloud_to_laserscan（apt 可用）
sudo apt install ros-humble-pointcloud-to-laserscan

# FAST-LIO2（使用 bdus/FASTLIO2_ROS2 fork，含回环、重定位、一致性优化）
git clone https://github.com/bdus/FASTLIO2_ROS2.git
cd FASTLIO2_ROS2
# 初始化子模组（含 Sophus 1.22.10）
git submodule update --init --recursive
colcon build --packages-select fastlio2 hba localizer pgo interface

# Sophus（若未通过子模组获取，可自行编译）
# 参考：git clone https://github.com/strasdat/Sophus.git && cd Sophus && git checkout 1.22.10 && mkdir build && cd build && cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make && sudo make install

# 其他编译依赖（需自行满足）：
#   PCL >= 1.8, Eigen >= 3.3.4, yaml-cpp, GTSAM（via apt or source）
#   livox_ros_driver2（请参考其官方仓库完成 SDK 与 ROS2 包编译）

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
