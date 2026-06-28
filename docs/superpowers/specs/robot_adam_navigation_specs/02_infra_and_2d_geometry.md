# 基础设施与 2D 几何导航栈详细规格说明书 (SPEC 02_Infra_2D)

> **版本**: v1.2 | **日期**: 2026-06-28 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/02_infra_and_2d_geometry.md`
> **范围**：`adam_assets` 资产包骨架 + 基础双层 EKF 里程计 + Cartographer 2D 建图/纯定位 + `explore_lite` 自主探索 + Nav2 Smac/MPPI 运控级集成。
> **关联总设计**：[`docs/spec/01_robot_adam_navigation_architecture.md`](../../../spec/01_robot_adam_navigation_architecture.md) — 本系列宏观架构纲领。
> **前置依赖**：`SPEC 01`（宏观架构），`robot_description`（Level 1 仿真底盘变体及传感器话题已就绪）。
> **总工期预估**：4 天 | **原子交付单元数**：6

---

## 1. 本期范围与边界 (Scope & Boundary)

### 1.1 本期开发目标

在 Gazebo 仿真环境（或真车常驻物理层）中，彻底跑通"常驻基础里程计 → 自主边界探索建图 → 地图异步归档至静态包 → 二次开机一帧无感全局重定位 → Nav2 路径规划与高频避障运控"的完整 2D 黄金闭环。

### 1.2 本期严格不包含

以下高级内容属于后续演进 SPEC，本期**不进行任何代码编写与接口预留**：

- 3D 固态激光雷达紧耦合（FAST-LIO2）与 Nav2 STVL 时空体素层（SPEC 03）。
- 传统特征点视觉 VIO（ORB-SLAM3）与神经网络稠密 SLAM（SPEC 04）。
- 顶层控制中枢 Lifecycle 状态机逻辑管理、大模型具身语义层（SPEC 05）。

---

## 2. 最小原子交付单元拆解 (Sub-Agent Units)

本期任务聚焦于将 Top-Down 蓝图化解为 6 个不带外部依赖、可单点突破和进行单元测试的"最小可执行单元"：

### 📦 Unit 1: adam_assets 集中式静态资源仓库搭建

**功能描述**：建立全局非结构化资产（地图、校准文件）的统一放置规范，利用 ROS2 编译路由解决多算法读写路径硬编码的顽疾。

**物理目录结构**：
```
adam_assets/
├── CMakeLists.txt
├── package.xml
└── share/
    ├── camera_calibration/     # 留空（预留给 SPEC 04）
    ├── maps_2d/               # 存放本期生成的 .yaml 与 .pgm 栅格地图
    └── maps_3d/               # 存放本期生成的 Cartographer .pbstream 序列化子图
```

**接口与路由规范**：

- `CMakeLists.txt` 必须包含：`install(DIRECTORY share/ DESTINATION share/${PROJECT_NAME})`。
- 提供一个 Python 工具函数 `get_asset_path(asset_type, file_name)`，通过 `ament_index_python.packages.get_package_share_directory('adam_assets')` 动态拼接返回非结构化资产的绝对路径。

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

### 📦 Unit 2: adam_localization 基础里程计底座 (核心 Odom 锁死)

**功能描述**：作为开机常驻的"死忠层"，通过扩展卡尔曼滤波（EKF）融合底盘原始轮速与高频 IMU，消除车轮打滑产生的阶跃误差，为上层 SLAM 提供高频平滑位姿先验。

**数据流拓扑与输入输出**：

- **输入话题**：Level 1 底盘发布的原始编码器里程计 `/odom_raw`（包含 `geometry_msgs/msg/TwistWithCovariance`）与高频物理惯导 `/imu/data`。
- **输出 TF**：启动常驻的 `local_ekf_node`，**唯一广播** `odom -> base_link` 的 TF 树。其 `publish_tf` 强制为 `true`。

**数据分流与降级硬性规格**：

- 启动第二层 `global_ekf_node`，负责广播 `map -> odom` 的 TF。
- **2D 阶段降级策略**：本期由于没有 3D/视觉的高精位姿源，`global_ekf_node` 接收来自 Unit 5 的 `/cartographer_pose` 话题作为全局观测源。当小车运动平稳时，以 Cartographer 为高权重修正 `map -> odom` 漂移；一旦 Cartographer 报告匹配低置信度（如处于完全无几何特征的空旷地带），`global_ekf_node` 通过其门限过滤器（Mahalanobis Distance）自动降低其权重，完全依靠底层平滑的 `odom -> base_link` 维持小车姿态，绝不断流或阶跃。

**参数约束（硬性规格）**：

- `ekf_local.yaml` 中，轮速计配置 `odom0_config`：融合 `Pose (x, y)` 和 `Twist (angular_z)`，共 3 自由度
- IMU 配置 `imu0_config`：融合 `angular_velocity (z)` 和 `linear_acceleration (x, y)`，提供高频姿态阻尼
- **全局铁律**：Cartographer 节点的 `provide_odom_frame` 必须强制设为 `false`，彻底剥夺其直接发布 TF 的权力！
- `local_ekf_node` 常驻且**唯一广播** `odom -> base_link` TF
- `differential`：`false`（输出绝对连续位姿，不依赖差分模式）

---

### 📦 Unit 3: adam_slam 2D Cartographer 建图与 explore_lite 自动探索

**功能描述**：驱动 2D 激光雷达进行局部子图（Submap）图优化，同时挂载边界前沿（Frontier）算法让小车在未知仿真环境中自动进行全范围探图。

**算法输入**：Level 1 的 2D 激光雷达话题 `/scan`，以及来自 Unit 2 的平滑 `odom -> base_link` TF 树。

**核心配置参数 (`cartographer_2d.lua`)**：

- `tracking_frame = "base_link"`，`published_frame = "odom"`（注意：不准直接发布到 map）。
- 激活 `use_odometry = true`，将 Unit 2 融合后的平滑里程计作为扫描匹配（Scan Matching）的空域中心先验。
- 配置 `explore_lite`：最小边界像素长度设为 5，增益权重设为 1.0，使其能动态消费 Cartographer 实时吐出的 `/map` 话题并自主发布 `/cmd_vel`。

**Cartographer 关键参数**：
```lua
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.6
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
```

**explore_lite 关键参数**：
```yaml
explore_lite:
  ros__parameters:
    exploration_radius: 3.0
    min_frontier_size: 10
    potential_scale: 1e-4
    gain_scale: 1.0
    transform_tolerance: 0.1
```

---

### 📦 Unit 4: adam_assets 异步数据流落盘与中转归档机制

**功能描述**：扫地机自主探图结束后，解决各个 SLAM 节点运行在物理沙盒内、无权限直接改写 `install/share/` 本地代码包的落盘闭环流程。

**归档自动化脚本 (`archive_map.py`)**：

- 脚本接收落盘 Service 信号 → 动态调用 Cartographer 的标准服务 `/write_state`。
- 算法先将地图文件（`.pbstream`, `.yaml`, `.pgm`）写出到系统高权限临时区 `/tmp/adam_maps/`。
- 脚本利用 OS 级标准库，将文件拷贝搬运至本地 Workspace 的源码路径 `src/3.navigation_ai/adam_assets/share/maps_2d/` 与 `maps_3d/`。
- 脚本在后台静默异步执行系统级编译指令：`colcon build --packages-select adam_assets`。

**归档流程**：
```
触发 /save_current_map 服务
        │
        ▼
调用 Cartographer /write_state 服务 → 写入 /tmp/adam_maps/{timestamp}/
        │
        ▼
脚本拷贝文件至 src/3.navigation_ai/adam_assets/share/maps_2d/
        │
        ▼
后台静默执行 colcon build --packages-select adam_assets
        │
        ▼
验证 install/share/adam_assets/maps_2d/ 下文件就绪
```

**通关效果**：刷新 `install/share/` 路径，使二次开机时系统能无缝路由读取新地图，完成非结构化资源闭环。

---

### 📦 Unit 5: adam_slam 纯定位与一帧无感全局重定位

**功能描述**：机器人二次开机，在完全拒绝人工介入给定初始位姿（不依赖 `2D Pose Estimate` 按钮）的前提下，利用全局分支定界（Branch-and-Bound）扫描匹配，实现开机瞬间定位自动锁死。

**核心配置参数 (`cartographer_localization.lua`)**：

- 将 `TRAJECTORY_BUILDER.pure_localization = true` 激活。
- 将 `POSE_GRAPH.optimize_every_n_nodes = 3`（高频触发全局优化，加速重定位速度）。
- 节点加载 Unit 4 归档的 `.pbstream` 资产，开机一帧雷达数据打在墙上，瞬间解算并输出高频全局绝对位姿话题 `/cartographer_pose` 喂给 Unit 2 的全局 EKF。

**关键参数差异（与建图模式对比）**：
```lua
TRAJECTORY_BUILDER_2D.pure_localization = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil

POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4

-- 通过命令行参数加载 pbstream：-load_state_filename <pbstream_path>
```

---

### 📦 Unit 6: adam_navigation Nav2 Smac & MPPI 运控级集成

**功能描述**：消费 Level 2 发布的绝对平滑的 TF 树，进行 Hybrid-A* 运动学寻路，并通过模型预测路径积分（MPPI）进行高频滚动避障，内嵌扫地机基础自救行为树。

**核心配置参数 (`nav2_2d_config.yaml`)**：

- **Planner**：挂载 `nav2_smac_planner/SmacPlanner2D`，针对差速/全向底盘约束，开辟运动学可行路径。
- **Controller**：挂载 `nav2_mppi_controller/MPPIController`。配置时间前向推演步数 `time_steps = 56`，模型速度扰动采样率 `batch_size = 2000`，确保底盘丝滑加速，动态绕行不突兀减速。
- **自救行为树 (Recovery BT)**：内嵌标准动作序列：`ClearLocalCostmap` → `Spin`（原地旋转重定位） → `Backup`（倒车脱困）。

**Nav2 配置**：
```yaml
planner_server:
  ros__parameters:
    planner_plugin_types: ["nav2_smac_planner/SmacPlanner2D"]
    SmacPlanner2D:
      downsample_costmap: false
      tolerance: 0.25
      allow_unknown: false
      max_iterations: 1000000
      smooth_path: true

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

---

## 3. Bottom-Up 路线图与单点测试验收方案 (Gate Criteria)

测试必须在仿真/真实物理环境中严格按照以下 6 步推进，并闭环通过白纸黑字的测试用例。上一步骤未达成通关标准，严禁推进。

```
[步骤 1: 建立静态包骨架] → [步骤 2: 锁死常驻底座 EKF] → [步骤 3: 自主探索建图]
                                                                  │
[步骤 6: Nav2 丝滑运控] ← [步骤 5: 开机一帧无感定位] ← [步骤 4: 异步中转归档闭环]
```

---

### 🛠️ 步骤 1：构建资产包与 Ament 动态索引 (Unit 1) — 预估 0.5 天

**操作方法**：在本地 `share/maps_2d/` 下手动放置一个虚拟的 `test_map.yaml`。运行 `colcon build --packages-select adam_assets`。打开任意外部 Python 终端，调用 `get_asset_path('maps_2d', 'test_map.yaml')` 函数。

**验收方案与标准（Gate 1）**：

1. **无硬编码验证**：检查打印结果，若能在不依赖系统绝对路径（如 `/home/user/...`）的前提下，准确打印出以 `install/adam_assets/share/adam_assets/maps_2d/test_map.yaml` 结尾的沙盒内绝对路径，则地基通关。

---

### 🛠️ 步骤 2：锁死底层常驻里程计 TF 树与降级流控机制 (Unit 2) — 预估 0.5 天

**操作方法**：启动 Gazebo 仿真小车，控制底盘使其在滑腻地面上原地疯狂旋转打滑，或通过仿真器给人为给车身施加剧烈的物理侧向碰撞。监控终端并观测 Rviz 中的 `odom` 坐标系与 TF 树。

**验收方案与标准（Gate 2）**：

1. **高频连续性验证**：使用 `ros2 run tf2_ros tf2_monitor odom base_link` 监控。`odom -> base_link` 的 TF 广播频率必须稳定常驻 **≥50Hz**，丢包率 ≤0.1%。
2. **剧烈运动防跳变验证**：在整个原地打滑和强碰撞过程中，坐标变换曲线必须绝对物理连续。
3. **通关铁律**：**显式观测 Rviz，整车模型坐标绝不允许发生哪怕 1 厘米或 1 度的瞬间画面跃迁、抖动或断流**。

---

### 🛠️ 步骤 3：自主探索建图与局部子图质量调优 (Unit 3) — 预估 1 天

**操作方法**：一键拉起建图 Launch 链，启动 Cartographer 与 explore_lite。观察小车是否能自主提取未建图区域的边界（Frontier），并自主生成稳定的 `/cmd_vel` 驱动底盘。

**验收方案与标准（Gate 3）**：

1. **自主探图覆盖率验证**：在 100 平方米的复杂多房间仿真地图中，小车必须在 **15 分钟内**自主探索完大面积区域，中途无任何卡死或原地无限转圈现象。
2. **地图锋利度验证**：观察 Rviz 中逐渐吐出的局部子图（Submaps）。
3. **通关铁律**：当小车在仿真环境中高速行走、甚至原地快速自旋时，建出来的墙壁和几何线条必须保持锋利（厚度在 1-2 像素以内）。**在没有触发回环检测（Loop Closure）前，由于有融合里程计的精准空域先验，局部子图绝不允许出现可见的重影、线条分叉或墙壁重叠现象**。

---

### 🛠️ 步骤 4：打通资产序列化中转与归档流水线 (Unit 4) — 预估 0.5 天

**操作方法**：小车自主探索建图完毕后，通过终端或中枢发送触发信号激活 `archive_map.py` 归档脚本。

**验收方案与标准（Gate 4）**：

1. **落盘完整性验证**：检查系统的 `/tmp/adam_maps/` 目录，必须包含完好的、带统一时间戳后缀的 `.yaml`（栅格配置文件）、`.pgm`（占用概率图像）和 `.pbstream`（Cartographer 序列化地图状态）三件套。
2. **源码仓库自动刷新验证**：静待数秒，检查本地 Workspace 源码路径 `src/3.navigation_ai/adam_assets/share/maps_2d/` 下，必须已经静默同步拷入了上述新生成的地图文件。
3. **通关铁律**：后台异步编译进程完成后，执行 `ros2 asset` 或检查 `install/` 目录，**新地图文件必须以 100% 的确定性被打包部署进 ROS2 静态运行沙盒内，系统无需二次手动 colcon build 即可被其他节点加载**。

---

### 🛠️ 步骤 5：劫持机器人验证开机一帧无感全局重定位 (Unit 5) — 预估 0.5 天

**操作方法**：关闭全车所有节点。在 Gazebo 仿真器中，**将小车随机传送到静态地图的任意未知角落**（模拟人类在关机状态下把机器人抱到了别的房间，即经典的机器人"被劫持"场景）。拉起纯定位 Launch 链，加载步骤 4 归档的 `.pbstream` 地图。

**验收方案与标准（Gate 5）**：

1. **盲开禁止验证**：整个启动过程**完全禁止人工点击 Rviz 上的 `2D Pose Estimate` 按钮下发初始位置猜测**。
2. **时效性量化指标**：车子在完全静止状态下，雷达扫描一帧打在墙上，全局分支定界匹配算法必须在 **1.0 秒之内（硬性门槛）** 瞬间匹配成功。
3. **通关铁律**：定位成功瞬间，`global_ekf_node` 发布的 `map -> odom` TF 瞬间对齐锁死，Rviz 上的整车 3D 模型**必须瞬间精准跳回小车当前在 Gazebo 里所处的实际房间和绝对坐标系中**，重定位位置误差 ≤3cm，角度误差 ≤2°。

---

### 🛠️ 步骤 6：Nav2 运控高频避障与行为树自救测试 (Unit 6) — 预估 1 天

**操作方法**：拉起配置了 Smac 规划器与 MPPI 控制器的 Nav2 导航全栈。在 Rviz 地图上跨区域下发一个远距离目标点。在小车开始全速前行的路线正前方，通过仿真器**突发性地丢下一块静止的障碍物箱子**。随后，再用一圈箱子将小车的全向去路**彻底堵死**。

**验收方案与标准（Gate 6）**：

1. **规划时效验证**：Smac Planner 必须在 **50 毫秒内**给出符合车身运动学、无任何多余尖锐折角的 Hybrid-A* 可行寻路轨迹。
2. **滚动避障丝滑度验证**：当突发障碍物抛下时，MPPI 控制器必须在其前向 56 步的时空推演中迅速感知代价图升高。小车必须以极度平滑的弧线优雅绕过箱子。
3. **通关铁律（运控无顿挫）**：绕行期间，**小车的线速度减速幅值绝对不允许超过原设定速度的 20%，整车绝不允许发生任何突兀的急刹、停顿或倒退**。
4. **自救行为树闭环验证**：当去路被彻底堵死时，底盘在原地死锁 2.0 秒内，必须无条件触发定制行为树。查看终端日志，小车必须严密执行 `ClearCostmapRecovery` → `Spin`（原地旋转找出口） → `Backup`（倒车脱困）的自救跳转流，且状态机反馈（Status Feedback）绝不抛出任何指针异常。

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

| 单元 | 产出物 | 通关标准 | 工期 |
|------|-------|---------|------|
| Unit 1 | adam_assets 包 + get_asset_path() | `ament_index_python` 可检索，无硬编码 | 0.5d |
| Unit 2 | EKF 里程计底座（local + global） | `odom->base_link` ≥50Hz 无阶跃，降级保护 | 0.5d |
| Unit 3 | Cartographer 建图 + explore_lite | 100m² 15min 自主探索，子图锋利无重影 | 1d |
| Unit 4 | archive_map.py 归档脚本 | 地图三件套落盘 + 自动 colcon build 刷新 | 0.5d |
| Unit 5 | Cartographer 纯定位 | 1s 内一帧重定位，位置误差 ≤3cm，角度 ≤2° | 0.5d |
| Unit 6 | Nav2 Smac + MPPI + BT 自救 | 50ms 规划，绕障不降速 20%，BT 自救链完整 | 1d |
