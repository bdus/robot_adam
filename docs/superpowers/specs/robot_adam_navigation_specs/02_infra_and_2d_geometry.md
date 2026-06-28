# 基础设施与 2D 几何导航栈详细规格说明书 (SPEC 02_Infra_2D)

> 版本: v1.0 | 日期: 2026-06-28 | 状态: Working
> 范围：adam_assets 资产包骨架 + 基础 EKF 里程计 + Cartographer 2D 建图/定位 + explore_lite 自主探索 + Nav2 运控
> 前置依赖：SPEC 01（宏观架构），robot_description（6 种底盘变体已完成）
> 总工期预估：~4 天 | 原子单元数：6

---

## 1. 范围与边界

### 1.1 本期目标

在 Gazebo 仿真环境中，彻底跑通"常驻底盘 → 自动探索建图 → 地图序列化落盘至 adam_assets → 二次开机一帧重定位 → Nav2 运控"的扫地机核心闭环。

### 1.2 本期不包含

以下内容属于后续 SPEC，本期**不实现**：
- 3D 激光雷达 FAST-LIO2 与 STVL 时空避障（SPEC 03）
- 视觉 SLAM ORB-SLAM3 / DROID-SLAM（SPEC 04）
- 中枢 Lifecycle 状态机、VLN 大模型决策（SPEC 05）

### 1.3 输入依赖

| 依赖 | 来源 | 状态 |
|------|------|------|
| Gazebo 仿真环境 + 2D 激光雷达 `/scan` | robot_description | ✅ 已完成 |
| 底盘 `/odom`（轮速里程计） | robot_description / Gazebo | ✅ 已完成 |
| IMU `/imu/data` | robot_description | ✅ 已完成 |
| `bigH.world` 仿真场景 | robot_description | ✅ 已完成 |
| 2WD/4WD/Omni 差速底盘控制 | robot_description | ✅ 已完成 |

---

## 2. 原子单元拆解

本期拆解为 6 个最小可交付单元（Unit），每个单元可由 Sub-Agent 在 0.5~1 天内独立开发。

### 📦 Unit 1: adam_assets 静态资源仓库

**功能描述**：建立全局非结构化资产的动态索引路由，解决各个 SLAM 算法生成地图后路径硬编码问题。

**输入依赖**：无。

**产出物**：
- 标准 ROS2 静态资产包，包含 `camera_calibration/`, `maps_2d/`, `maps_3d/` 子目录
- `CMakeLists.txt`：利用 `install(DIRECTORY ...)` 将资产安装至 ROS2 `share` 空间
- `package.xml`：声明为标准的 `ament_cmake` 包
- Python 资产解析器工具函数 `get_asset_path(sub_folder, file_name)`

**目录结构**：
```
adam_assets/
├── camera_calibration/
│   ├── sim_camera_intrinsics.yaml      # 仿真相机内参（占位）
│   └── real_camera_intrinsics.yaml     # 真实相机内参（占位）
├── maps_2d/
│   └── .gitkeep                        # 初始为空，建图后填充
├── maps_3d/
│   └── .gitkeep
├── maps_vision/
│   └── .gitkeep
├── maps_semantic/
│   └── .gitkeep
├── CMakeLists.txt                      # install(DIRECTORY ...) 分发资产
└── package.xml
```

**通关标准**：
1. 执行 `colcon build --packages-select adam_assets` 编译通过
2. 在任意 Python 终端执行：
   ```python
   from ament_index_python.packages import get_package_share_directory
   path = get_package_share_directory('adam_assets')
   print(path)  # 必须输出 install/adam_assets/share 的绝对路径
   ```
3. 该路径下包含所有子目录和 `.gitkeep` 文件

---

### 📦 Unit 2: 基础里程计底座 EKF

**功能描述**：融合底盘轮速里程计 `/odom_raw` 与高频 IMU `/imu/data`，消除物理层打滑产生的随机误差，发布平滑的 `odom -> base_link` TF 树。

**输入依赖**：`robot_description` 发布的 `/odom_raw`（原始轮速里程计，≥50Hz 编码器解算，包含 `Twist` 速度和累计 `Pose`）与 `/imu/data`（高频 IMU 话题）。

**数据流动拓扑**：
```
底盘 MCU 编码器 → /odom_raw (Twist+Pose, ≥50Hz)
                              ↓
IMU 传感器       → /imu/data (angular_velocity + linear_acceleration)
                              ↓
                    local_ekf_node (robot_localization)
                              ↓
              发布唯一 odom → base_link TF (≥50Hz, 绝对连续)
```

**参数约束（硬性规格）**：
- `ekf_local.yaml` 中，轮速计配置 `odom0_config`：融合 `Pose (x, y)` 和 `Twist (angular_z)`，共 3 自由度
- IMU 配置 `imu0_config`：融合 `angular_velocity (z)` 和 `linear_acceleration (x, y)`，提供高频姿态阻尼
- **必须关闭任何 SLAM 节点的直接 TF 广播**：Cartographer 的 `provide_odom_frame` 必须设为 `false`
- `local_ekf_node` 常驻且**唯一广播** `odom -> base_link` TF
- `differential`：`false`（输出绝对连续位姿，不依赖差分模式）

**退化协方差保护逻辑**：
- 在本期（2D 阶段），Cartographer 纯定位输出的 `/cartographer_pose` 通过 Pose Relay 适配器以**显式高协方差**喂给第二层 `global_ekf_node`
- `global_ekf_node` 负责发布唯一的 `map -> odom` TF
- 当 Cartographer 定位协方差超限时，`global_ekf_node` 自动向底层轮速计 + IMU EKF 权重倾斜，确保即使定位漂移也不会导致底盘瞬移

**产出物**：
- `config/ekf_local.yaml`：首层 `robot_localization` EKF 节点配置
- `config/ekf_global.yaml`：第二层全局 EKF 节点配置（融合 Cartographer 位姿）
- `launch/ekf_localization.launch.py`：启动脚本
- 关键参数：
  - `odom0`：`/odom_raw`（轮速里程计话题）
  - `imu0`：`/imu/data`（IMU 话题）
  - `differential`：`false`
  - `odom0_config`：`[true, true, false, false, false, false, false, false, false, false, false, true, false, false, false]`
  - `imu0_config`：`[false, false, false, false, false, true, false, false, false, false, false, false, false, false, false]`

**通关标准**：
1. 在 Gazebo 中人为让底盘原地打滑/剧烈冲撞
2. 观察 Rviz 中 `odom` 坐标系
3. **铁律**：`odom -> base_link` TF 坐标变换绝对连续，在 `tf_monitor` 中频率 ≥50Hz，且绝无阶跃

---

### 📦 Unit 3: Cartographer 2D 建图与 explore_lite 自主探索

**功能描述**：驱动 2D 激光雷达进行局部图优化（Submap），同时挂载边界前沿（Frontier）算法让小车在仿真环境中自主盲探建图。

**输入依赖**：`/scan`（激光雷达），`odom -> base_link`（来自 Unit 2）。

**产出物**：
- `config/cartographer_2d.lua`：建图模式配置（激光关联、运动学窗约束、闭环检测频率）
- `config/explore_lite.yaml`：边界探索参数（最小边界像素长度、信息增益权重）
- `launch/cartographer_mapping.launch.py`：一键启动建图 + 盲探

**cartographer_2d.lua 关键参数**：
```lua
-- 轨迹构建器
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.6
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- 全局位姿估计器（闭环检测）
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
```

**explore_lite.yaml 关键参数**：
```yaml
explore_lite:
  ros__parameters:
    exploration_radius: 3.0
    min_frontier_size: 10
    potential_scale: 1e-4
    gain_scale: 1.0
    transform_tolerance: 0.1
```

**通关标准**：
1. 启动仿真环境，加载 `cartographer_mapping.launch.py`
2. 小车自动提取未建图区域边界（Frontier），自主生成 `/cmd_vel` 驱动底盘
3. Rviz 中逐渐吐出局部子图（Submaps），无大范围重影或几何扭曲

---

### 📦 Unit 4: 异步中转地图归档机制

**功能描述**：解决节点沙盒无权限直接修改 `install/share` 资产路径的问题。建立临时中转区 → 拷贝至源码目录 → 触发 `colcon build` 的完整归档流水线。

**输入依赖**：Cartographer 提供的 `/write_state` 服务（写入 `.pbstream`）。

**产出物**：
- `scripts/archive_map.py`：自动化归档脚本

**归档流程**：
```
触发 /save_current_map 服务
        │
        ▼
调用 Cartographer /write_state 服务 → 写入 /tmp/adam_maps/{timestamp}/
        │
        ▼
脚本拷贝文件至 src/3.navigation_ai/adam_assets/maps_2d/
        │
        ▼
后台静默执行 colcon build --packages-select adam_assets
        │
        ▼
验证 install/share/adam_assets/maps_2d/ 下文件就绪
```

**脚本接口**：
```python
def archive_map(temp_path: str, target_subdir: str, filename: str) -> bool:
    """
    将临时文件归档至 adam_assets 并刷新 install/share 路径
    :param temp_path: /tmp/adam_maps/ 下的源文件路径
    :param target_subdir: maps_2d / maps_3d 等
    :param filename: 目标文件名
    :return: True 表示归档成功
    """
```

**通关标准**：
1. 自主探图完成后，触发归档脚本
2. 检查 `/tmp/adam_maps/` 是否生成带时间戳的 `.yaml`、`.pgm`、`.pbstream`
3. 后台 `colcon build` 自动执行完毕
4. `src/3.navigation_ai/adam_assets/maps_2d/` 下可见完整地图文件

---

### 📦 Unit 5: Cartographer 纯定位与一帧重定位

**功能描述**：小车二次开机时，拒绝人工给定初始位姿（2D Pose Estimate），利用全局分支定界（Branch-and-Bound）扫描匹配，实现开机瞬间定位自动锁死。

**输入依赖**：`adam_assets/maps_2d/` 中的静态地图与 `.pbstream`（来自 Unit 4）。

**产出物**：
- `config/cartographer_localization.lua`：纯定位模式配置
- `launch/cartographer_localization.launch.py`：启动脚本

**关键参数差异（与建图模式对比）**：
```lua
-- 纯定位模式
TRAJECTORY_BUILDER_2D.pure_localization = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil

-- 全局重定位参数（分支定界）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4

-- 加载已保存的 pbstream
-- 通过命令行参数：-load_state_filename <pbstream_path>
```

**通关标准**：
1. 关闭所有节点，将小车在 Gazebo 仿真中**随机传送到地图任意未知角落**（模拟劫持开机）
2. 启动纯定位 Launch，加载上一步生成的地图资产
3. **铁律**：不给任何 2D Pose Estimate，雷达扫描一帧打在墙上，Cartographer 分支定界算法必须在 **1 秒内** 将 `map -> odom` TF 对齐锁死，Rviz 中车子瞬间跳回正确位置

---

### 📦 Unit 6: Nav2 Smac & MPPI 运控集成

**功能描述**：消费全局定位，进行 Hybrid-A* 运动学可行寻路，并通过模型预测路径积分（MPPI）进行高频滚动避障运控。

**输入依赖**：Unit 5 输出的定位 + 全局 EKF 节点（统一发布 `map -> odom` TF）。

**产出物**：
- `config/nav2_2d_params.yaml`：Nav2 配置（SmacPlanner2D + MPPI + Behavior Tree）
- `launch/navigation.launch.py`：Nav2 启动脚本

**nav2_2d_params.yaml 关键参数**：
```yaml
# Smac Planner 2D
planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_smac_planner/SmacPlanner2D"]
    SmacPlanner2D:
      downsample_costmap: false
      tolerance: 0.25
      allow_unknown: false
      max_iterations: 1000000
      smooth_path: true

# MPPI Controller
controller_server:
  ros__parameters:
    controller_plugin_types: ["nav2_mppi_controller/MppIController"]
    MppIController:
      motion_model: "DiffDrive"
      time_steps: 56
      model_dt: 0.1
      batch_size: 500
      temperature: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.0
      wz_max: 0.8
      enable_obstacle_collision_check: true

# Behavior Tree 恢复行为
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_recovery_and_remapping.xml"
```

**行为树恢复链**：
```
ClearCostmapRecovery (清除局部代价图残影)
        → Spin (原地旋转 360° 重定位)
        → Backup (倒车退出卡死区域)
        → 全局路径重规划
        → 若全部失败，通过 /adam_hub/recovery_failed 上报
```

**通关标准**：
1. 在 Rviz 中任意下发远距离目标点，Smac Planner 秒级给出 Hybrid-A* 可行路径
2. MPPI 控制器确保底盘平滑加速，动态障碍物丝滑绕行，无突兀降速
3. 前方强行用仿真物体"堵死"去路，观察终端触发 `ClearCostmap → Spin → Backup` 自救序列

---

## 3. 严格 Bottom-Up 构建与调试顺序

严禁跨阶段开发。每一步的前置步骤必须通关后才能进入下一步。

```
步骤 1: Unit 1 资产包骨架 (0.5天)
    ↓
步骤 2: Unit 2 底层 EKF 里程计 (0.5天)
    ↓
步骤 3: Unit 3 Cartographer 建图 + explore_lite 探索 (1天)
    ↓
步骤 4: Unit 4 地图归档流水线 (0.5天)
    ↓
步骤 5: Unit 5 开机一帧重定位 (0.5天)
    ↓
步骤 6: Unit 6 Nav2 Smac + MPPI 运控 (1天)
```

### 🛠️ 步骤 1：构建资产包骨架 (Unit 1) — 0.5 天

**操作**：
1. 在 `src/3.navigation_ai/` 下创建 `adam_assets/` 包
2. 编写 `CMakeLists.txt`，使用 `install(DIRECTORY ...)` 分发所有子目录
3. 编写 `package.xml`
4. 执行 `colcon build --packages-select adam_assets`

**通关验证**：
```bash
# 验证路径可访问
python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('adam_assets'))"
# 输出应为：/home/pi/workplace/robot_adam/install/adam_assets/share/adam_assets
```

### 🛠️ 步骤 2：锁死底层里程计 (Unit 2) — 0.5 天

**操作**：
1. 在 `src/2.localization_mapping/adam_localization/` 下创建 `config/ekf_local.yaml`
2. 编写 `launch/ekf_localization.launch.py`
3. 启动 Gazebo + 启动 EKF 节点

**通关验证**：
```bash
# 在 Rviz 中添加 odom 显示，人为让小车打滑
ros2 run tf2_tools tf2_monitor
# 确认 odom -> base_link 频率 ≥50Hz，无阶跃
```

### 🛠️ 步骤 3：自主盲探建图 (Unit 3) — 1 天

**操作**：
1. 在 `src/2.localization_mapping/adam_slam/` 下创建配置和 Launch 文件
2. 安装依赖：`sudo apt install ros-humble-cartographer ros-humble-cartographer-ros`
3. 编译并启动建图 Launch

**通关验证**：
- Rviz 中看到小车自主探索未知区域
- 实时 `/map` 话题持续更新
- 子图无大范围重影

### 🛠️ 步骤 4：地图落盘归档 (Unit 4) — 0.5 天

**操作**：
1. 编写 `scripts/archive_map.py`
2. 探图完成后触发归档

**通关验证**：
```bash
ls /tmp/adam_maps/           # 确认有时间戳目录和文件
ls src/3.navigation_ai/adam_assets/maps_2d/  # 确认文件已拷贝
```

### 🛠️ 步骤 5：开机一帧重定位 (Unit 5) — 0.5 天

**操作**：
1. 创建纯定位配置和 Launch
2. 关闭所有节点，在 Gazebo 中随机重摆小车位置
3. 启动纯定位模式加载地图

**通关验证**：
- 不给 2D Pose Estimate
- 1 秒内 `map -> odom` TF 自动对齐
- Rviz 中小车瞬间跳回正确位置

### 🛠️ 步骤 6：Nav2 运控调优 (Unit 6) — 1 天

**操作**：
1. 安装 Nav2 依赖：`sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
2. 创建 Nav2 参数配置和 Launch
3. 启动后下发目标点测试

**通关验证**：
- Rviz 下发目标点，秒级规划可行路径
- 小车平滑加速、绕障
- 堵死时触发 BT 恢复序列

---

## 4. 依赖安装

```bash
# 2D 建图与定位
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# Nav2 全栈
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 机器人定位（EKF）
sudo apt install ros-humble-robot-localization

# explore_lite（需源码编译）
git clone https://github.com/hrnr/m-explore.git -b ros2
```

---

## 5. 交付标准总览

| 单元 | 产出 | 通关标准 | 工期 |
|------|------|---------|------|
| Unit 1 | adam_assets 包 | `ament_index_python` 可检索 | 0.5d |
| Unit 2 | EKF 里程计 | `odom->base_link` ≥50Hz 无阶跃 | 0.5d |
| Unit 3 | Cartographer 建图 + explore_lite | 自主探索吐子图 | 1d |
| Unit 4 | 归档脚本 | 地图存入 `adam_assets/maps_2d/` | 0.5d |
| Unit 5 | Cartographer 纯定位 | 1 秒内一帧重定位 | 0.5d |
| Unit 6 | Nav2 Smac + MPPI | 路径规划 + 绕障 + BT 自救 | 1d |
