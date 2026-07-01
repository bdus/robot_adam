# 导航系统自动化测试基础设施 (SPEC 06_Test_Infra)

> **版本**: v1.0 | **日期**: 2026-07-01 | **状态**: Working
> **存放目录**: `docs/superpowers/specs/robot_adam_navigation_specs/06_test_infrastructure.md`
> **范围**：`adam_test_tools` 包设计 — 统一覆盖 SPEC 02~05 所有 Gate 的自动化场景注入与验收判定工具。
> **关联总设计**：[`02_infra_and_2d_geometry.md`](./02_infra_and_2d_geometry.md) 至 [`05_central_hub_and_vln.md`](./05_central_hub_and_vln.md) — 本工具包服务于全部导航 SPEC 的测试环节。
> **前置依赖**：ROS2 Humble + Gazebo 仿真环境已就绪。
> **总工期预估**：2 天 | **原子交付单元数**：3

---

## 1. 本期范围与边界 (Scope & Boundary)

### 1.1 本期开发目标

建立一套独立于各导航算法的测试基础设施包 `adam_test_tools`，提供：

1. **自动化场景注入工具**：通过 ROS2 Service / Topic 驱动 Gazebo 仿真器动态生成障碍物、劫持机器人、控制光照、注入外力等
2. **验收判定工具**：对各 Gate 的量化指标进行自动化监控与判定（TF 频率、体素消散时间、位姿阶跃幅度等）
3. **专用测试 World**：覆盖 2D 多房间建图、3D 悬空障碍物、暗室致盲等场景的 Gazebo 仿真世界

### 1.2 本期严格不包含

- 不替代 CI/CD pipeline
- 不涉及硬件在环（HIL）测试
- 不覆盖 Unit 级别的单元测试（`colcon test`）
- 不编写各 SLAM 算法自身的测试代码（各算法在自己的 SPEC 中处理）

---

## 2. 最小原子交付单元拆解 (Sub-Agent Units)

### 📦 Unit 1: 包骨架与测试世界定义

**功能描述**：建立 `adam_test_tools` 包的目录结构、编译配置和三个仿真测试世界文件。

**物理目录结构**：
```
adam_test_tools/
├── CMakeLists.txt
├── package.xml
├── worlds/
│   ├── test_2d_base.world           # 100m² 复杂多房间（SPEC 02 默认地图）
│   ├── test_3d_obstacles.world      # 桌子 + 悬空横板 + 行人（SPEC 03）
│   └── dark_room_test.world         # 封闭暗室 + 可控点光源（SPEC 04/05）
├── launch/
│   ├── test_tools.launch.py         # 统一启动工具包节点
│   └── dark_room.launch.py          # 暗室专项启动
├── scripts/                         # （Unit 2 实现）
└── verification/                    # （Unit 3 实现）
```

**测试 world 规格**：

#### `test_2d_base.world`
- 约 100m² 多房间封闭建筑
- 若干可通行门廊和区域
- **没有**悬空障碍物（SPEC 02 只需纯 2D 场景）
- 默认光照充足

#### `test_3d_obstacles.world`
- 一张普通四腿桌子（桌腿直径 ~4cm）
- 一块悬空横板（底盘上方 25cm，完全凌空）
- 一个可控制的仿真行人（Actor）自动路径
- 一个空旷的走廊区域（用于 FAST-LIO2 退化测试）

#### `dark_room_test.world`
- 封闭无窗房间（墙壁完整）
- 内含一个可控点光源
- 光源通过 Gazebo 插件暴露 `/dark_room/toggle_light`（`std_srvs/srv/SetBool`）

**世界文件依赖**：
```
test_2d_base.world        → 基于 bigH.world 简化，去除 3D 元素
test_3d_obstacles.world   → 基于 playground.world 扩展
dark_room_test.world      → 全新编写，添加 LightControl plugin
```

---

### 📦 Unit 2: 场景自动注入脚本

**功能描述**：6 个独立的 ROS2 Python 可执行节点，通过自定义 Service 或 Topic 驱动 Gazebo 仿真器执行测试操作。

**全部脚本放在 `scripts/` 目录下**：

#### `spawn_object.py`

在仿真器中动态生成/删除模型。

```
服务类型: std_srvs/srv/Trigger 扩展 (含 string command)
服务名:   /test_tools/spawn_object
命令格式: spawn <model_type> <x> <y> <z>
         delete <entity_name>
         clear                          # 清除所有动态障碍物
```

```python
# 使用示例
$ ros2 service call /test_tools/spawn_object custom_interfaces/srv/Spawn "{command: 'spawn box 1.5 0.5 0.0'}"
# 等价于执行：ros2 run gazebo_ros spawn_entity.py -entity box_123 -file path/to/box.sdf -x 1.5 -y 0.5 -z 0.0
```

- 内置 3 种基础模型：`box`（0.3m 立方体）、`cylinder`（0.2m 直径）、`barrier`（1.0m 长障碍物）
- 每个 spawned 实体带时间戳命名，避免重名冲突
- 对接底层 `gazebo_ros::spawn_entity` 与 `delete_entity`

#### `teleport_robot.py`

将 Gazebo 中的机器人模型瞬间传送到指定位置（劫持模拟）。

```
服务类型: std_srvs/srv/SetBool 扩展 (含 float x, float y, float yaw)
服务名:   /test_tools/teleport
```

```python
# 使用示例
$ ros2 service call /test_tools/teleport custom_interfaces/srv/Teleport "{x: 5.0, y: 3.0, yaw: 1.57}"
# 等价于：通过 gz transport 设置模型位姿
```

- 使用 `gz transport`（`ignition::msgs::Pose`）直接设置模型 `Pose`
- 需要获取机器人实体名称（从参数传入，默认 `laser_2wd`）

#### `control_light.py`

控制 `dark_room_test.world` 中的光照开关。

```
服务类型: std_srvs/srv/SetBool
服务名:   /test_tools/toggle_light
          true = 开灯, false = 关灯
```

- 对接 `dark_room_test.world` 中 Gazebo Light 插件

#### `spawn_actor.py`

控制仿真行人（Actor）沿预设路径行走。

```
服务类型: std_srvs/srv/Trigger
服务名:   /test_tools/spawn_actor
          trigger = 启动行人从左向右横穿
```

- 使用 Gazebo Actor 机制
- 预定义一条"从机器人左侧 5m 走到右侧 5m"的横穿路径
- 行走速度 ~1.0m/s

#### `apply_force.py`

对机器人模型施加瞬时物理外力（模拟碰撞、侧向冲击）。

```
服务类型: std_srvs/srv/Trigger 扩展
服务名:   /test_tools/apply_force
          force_x, force_y, force_z 方向参数
          duration_ms 持续时间
```

- 对接 `gz transport` Physics 接口

#### `set_friction.py`

动态修改地面摩擦系数（模拟打滑地面）。

```
服务类型: std_srvs/srv/SetFloat
服务名:   /test_tools/set_friction
          data: 0.01  # 极低摩擦 ≈ 冰面
```

---

### 📦 Unit 3: 验收判定节点

**功能描述**：自动化监控和量化判定各 Gate 的通过标准。全部放在 `verification/` 目录下。

#### `tf_monitor_node.py`

监控 TF 广播频率与丢包率。

- **订阅**：TF tree（`/tf`）
- **输出**：`/test_verdict/tf_health` topic（自定义消息）
- **判定指标**：
  - `odom → base_link` 频率 ≥ 50Hz
  - 丢包率 ≤ 0.1%
  - 发布 verdict: `PASS` / `FAIL`

#### `odom_health_node.py`

监控各 SLAM 位姿源的协方差，检测退化事件。

- **订阅**：`/slam_pose/*`（多个 Odometry 话题）
- **判定指标**：
  - 协方差对角线发散阈值探测（≥ 0.5 -> 触发退化标志）
  - 退化到恢复的时间计量
  - 发布 verdict: `HEALTHY` / `DEGRADED` / `LOST`

#### `voxel_decay_meter.py`

测量 STVL 体素消散时间（SPEC 03 Gate 3）。

- **订阅**：STVL 发布的体素地图话题
- **判定指标**：
  - 最后观测时间戳 → 体素计数归零的时间差
  - 通过标准：1.0s ± 0.1s
  - 发布 verdict: `PASS` / `FAIL`

#### `pose_jump_detector.py`

检测 `map → odom` TF 的位姿阶跃（SPEC 05 Gate 3）。

- **订阅**：TF `/tf` 话题
- **判定指标**：
  - `map → odom` 的瞬态位移变化量
  - 通过标准：单帧阶跃 ≤ 3cm, ≤ 2°
  - 发布 verdict: `PASS` / `FAIL`

---

## 3. Bottom-Up 路线图与单元划分

```
[Step 1: 包骨架 + 3 个 world 文件]
                 │
                 ▼
[Step 2: 场景注入脚本 x6]
                 │
                 ▼
[Step 3: 验收判定节点 x4]
```

---

## 4. 各 SPEC Gate 的测试工具引用关系

| 场景 | SPEC | 使用的工具 | 验收判定器 |
|------|------|-----------|-----------|
| 资产包完整性 | 02 Gate 1 | — | 手工验证 `ament_index_python` |
| EKF 高频防跳变 | 02 Gate 2 | `apply_force`, `set_friction` | `tf_monitor_node` |
| 自主探索建图 | 02 Gate 3 | — | 手工验证 Rviz |
| 地图归档 | 02 Gate 4 | — | 手工验证文件系统 |
| 一帧重定位 | 02 Gate 5 | `teleport_robot` | `pose_jump_detector` |
| Nav2 避障自救 | 02 Gate 6 | `spawn_object` | 手工验证日志 |
| FAST-LIO2 退化 | 03 Gate 1 | — | `odom_health_node` |
| 悬空障碍物捕获 | 03 Gate 2 | `spawn_object` | 手工验证 Rviz |
| STVL 体素消散 | 03 Gate 3 | `spawn_actor` | `voxel_decay_meter` |
| ORB-SLAM3 回环 | 04 Gate 1 | — | 手工验证日志 |
| DROID-SLAM GIL | 04 Gate 2 | — | 手工验证 `htop` |
| 暗室致盲续命 | 04 Gate 3 / 05 Gate 3 | `control_light`, `spawn_object` | `odom_health_node` + `pose_jump_detector` |
| Lifecycle 切换 | 05 Gate 1 | — | 手工验证日志 |
| YOLO 3D 投影 | 05 Gate 2 | — | 手工验证 JSON |
| 终极模糊指令 | 05 Gate 3 | `control_light`, `spawn_object`, `teleport_robot` | `odom_health_node` + `pose_jump_detector` |

---

## 5. 依赖安装

```bash
# Gazebo 工具依赖（已随仿真环境安装）
# spawn_entity, delete_entity 由 gazebo_ros 包提供

# Python 依赖
# 全部使用 ROS2 内置库 + 标准库，无额外 pip 依赖
```

---

## 6. 交付标准总览

| 单元 | 产出物 | 通关标准 | 工期 |
|------|-------|---------|------|
| Unit 1 | 包骨架 + 3 个 world 文件 | `colcon build` 通过，Gazebo 可加载各 world | 0.5d |
| Unit 2 | 自动化脚本 x6 | 各脚本可通过 ros2 service call 调用并生效 | 1d |
| Unit 3 | 验收节点 x4 | 节点可自动产出 PASS/FAIL 判定 | 0.5d |
