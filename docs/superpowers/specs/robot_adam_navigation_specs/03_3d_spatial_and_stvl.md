# 基础设施与 3D 空间几何栈详细规格说明书 (SPEC 03_3D_Spatial)

> 版本: v1.0 | 日期: 2026-06-28 | 状态: Working
> 范围：FAST-LIO2 3D 雷达里程计 + pointcloud_to_laserscan 点云降维 + STVL 时空体素避障
> 前置依赖：SPEC 02（2D 全栈已通，adam_assets 已就绪）
> 总工期预估：~3 天 | 原子单元数：3

---

## 1. 范围与边界

### 1.1 本期目标

升级物理常驻雷达从 2D 切换为 3D 固态雷达 Mid360，源码编译 FAST-LIO2 输出高频 3D 里程计，引入 STVL 时空体素层解决动态障碍物"幽灵残影"问题，实现立体避障闭环。

### 1.2 本期不包含

- 视觉 SLAM ORB-SLAM3 / DROID-SLAM（SPEC 04）
- 中枢 Lifecycle 状态机、VLN 大模型决策（SPEC 05）

### 1.3 输入依赖

| 依赖 | 来源 | 状态 |
|------|------|------|
| Gazebo 仿真 + Mid360 雷达 `/livox/lidar` | robot_description | ✅ 已完成 |
| IMU `/imu/data` | robot_description | ✅ 已完成 |
| `adam_assets` 资产包 | SPEC 02 Unit 1 | 需先完成 |
| `local_ekf_node`（轮速+IMU）| SPEC 02 Unit 2 | 需先完成 |
| `global_ekf_node`（map->odom） | SPEC 02 Unit 2 | 需先完成 |

---

## 2. 原子单元拆解

本期拆解为 3 个最小可交付单元，每个单元可由 Sub-Agent 在 0.5~1 天内独立开发。

### 📦 Unit 1: FAST-LIO2 ROS2 Humble 适配与高频里程计

**功能描述**：对接物理/仿真 Mid360 雷达点云与内置 IMU，运行流形迭代卡尔曼滤波（IKFoM），输出 >100Hz 的高精 3D 里程计。

**输入依赖**：
- `/livox/lidar`：Mid360 点云话题
- `/imu/data`：IMU 数据话题

**产出物**：
- `config/fast_lio_mid360.yaml`：FAST-LIO2 参数配置（雷达内参、IMU 外参、初始噪声协方差）
- `launch/fast_lio_odometry.launch.py`：启动脚本
- 话题输出：`/fast_lio_odom`（`nav_msgs/msg/Odometry`，含协方差）

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

**Odom 降级保护规格**：
- **正常状态**：FAST-LIO2 正常运行时，其位姿协方差极小，`global_ekf_node` 赋予高融合权重
- **退化场景**：机器人进入长走廊/低几何特征环境，FAST-LIO2 位姿协方差（Covariance 矩阵对角线）剧烈发散
- **自动降级**：`global_ekf_node` 持续监听协方差，超限时毫秒级降低 FAST-LIO2 权重，向底层轮速计 + IMU EKF 倾斜，确保底盘不因 3D 里程计发散而瞬移

**通关标准**：
1. 启动 Mid360 雷达 + FAST-LIO2，`/fast_lio_odom` 稳定输出频率 ≥100Hz
2. 车子做剧烈颠簸/旋转运动时，位姿依然平滑无阶跃
3. 人为遮挡雷达（退化场景），观察协方差发散后底盘运控不失控

---

### 📦 Unit 2: pointcloud_to_laserscan 空间切片投影

**功能描述**：避免三维全局寻路消耗极端算力，将 3D 点云在机器人底盘上方指定高度空间进行水平切片，降维投影为 2D 伪激光话题，供全局规划器使用。

**输入依赖**：`/livox/lidar`（Mid360 稠密点云）

**产出物**：
- `config/pointcloud_to_laserscan.yaml`：切片参数
- `launch/cloud_to_scan.launch.py`：启动脚本
- 话题输出：`/scan_3d_projected`（`sensor_msgs/msg/LaserScan`）

**关键参数**：
```yaml
pointcloud_to_laserscan:
  ros__parameters:
    # 切片高度范围（机器人底盘上方 10cm ~ 40cm）
    min_height: 0.10
    max_height: 0.40

    # 射线角度分辨率
    angle_increment: 0.00872665       # ~0.5°

    # 有效距离
    range_min: 0.2
    range_max: 10.0

    # 扫描角度范围（360° 全向）
    angle_min: -3.14159
    angle_max: 3.14159
```

**通关标准**：
1. 在 Rviz 中观察 `/scan_3d_projected`
2. **铁律**：必须能完美把 3D 的凳子腿、悬空桌角、悬空屏幕投影成一圈 2D 激光点
3. 地面和天花板被切片过滤掉，不产生虚假障碍物

---

### 📦 Unit 3: Nav2 Costmap STVL 时空体素插件集成

**功能描述**：解决传统 2D 栅格地图无法表达悬空障碍物（桌角、悬空屏幕）以及动态行人走过留下"永久残影"的问题。引入 OpenVDB 三维体素管理，配置体素自动时间衰减。

**输入依赖**：
- `/scan_3d_projected`（来自 Unit 2，全局规划安全边界）
- `/livox/lidar`（原始 3D 点云，局部避障输入）

**产出物**：
- `config/nav2_3d_params.yaml`：含 STVL 插件的 Nav2 配置
- `launch/navigation_3d.launch.py`：3D Nav2 启动脚本

**关键参数**：
```yaml
# STVL 时空体素层配置
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
        voxel_decay: 1.0              # ⚡ 体素 1.0 秒自动消散
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

**通关标准**：
1. 在仿真中放一张桌子，小车经过时 STVL 正确识别悬空桌角
2. 人在小车前方来回走动，人走过去后 Rviz 中的 3D 体素在 **1.0 秒内** 像烟雾一样自动消散
3. Nav2 MPPI 控制器不因幽灵残影而发生"空地卡死"

---

## 3. 严格 Bottom-Up 构建与调试顺序

```
步骤 1: Unit 1 FAST-LIO2 3D 里程计 (1天)
    ↓
步骤 2: Unit 2 点云切片降维 (0.5天)
    ↓
步骤 3: Unit 3 STVL 时空避障闭环 (1.5天)
```

### 🛠️ 步骤 1：FAST-LIO2 编译与调优 — 1 天

**操作**：
1. 拉取社区 ROS2 Humble 移植分支（⚠ 官方已停更）
2. 编译并配置 Mid360 参数
3. 启动 FAST-LIO2 观察 `/fast_lio_odom`

**通关验证**：
```bash
ros2 topic hz /fast_lio_odom   # ≥100Hz
# 人为晃动小车，检查位姿平滑度
```

### 🛠️ 步骤 2：点云切片降维 — 0.5 天

**操作**：
1. 安装 `pointcloud_to_laserscan`
2. 配置切片高度参数
3. 启动观察伪激光话题

**通关验证**：
- Rviz 中 `/scan_3d_projected` 正确显示桌面、桌腿的轮廓
- 地面点被过滤干净

### 🛠️ 步骤 3：STVL 集成与调优 — 1.5 天

**操作**：
1. 源码编译 STVL 插件
2. 在 Nav2 局部代价图中挂载 `spatio_temporal_voxel_layer`
3. 配置 `voxel_decay` 为 1.0 秒
4. 在仿真中测试动态障碍物消散

**通关验证**：
```bash
# 人走过 -> 体素 1s 内消散
ros2 topic echo /local_costmap/voxel_markers | head -20
```

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

| 单元 | 产出 | 通关标准 | 工期 |
|------|------|---------|------|
| Unit 1 | FAST-LIO2 里程计 | ≥100Hz，退化协方差自动降级 | 1d |
| Unit 2 | 点云切片投影 | 悬空障碍物正确降维，地面过滤 | 0.5d |
| Unit 3 | STVL 时空避障 | 体素 1s 消散，无幽灵残影卡死 | 1.5d |
