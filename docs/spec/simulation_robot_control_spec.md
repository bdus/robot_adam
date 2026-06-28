# Robot Adam 仿真机器人运控系统规格说明书 (SPEC)

## 1. 问题定义

### 1.1 当前问题
- `gazebo_diff_drive_mid360.launch.py` 运动控制异常：发送 `cmd_vel (linear.x=0.5, angular.z=-1.0)` 后，小车直走正常但转向困难（odom yaw 几乎不变）
- 需要统一规划多种运控方式，便于后续开发和测试
- 当前 `gazebo_ackermann_laser.launch.py` 运动效果丝滑，与 mid360 版本存在差异

### 1.2 Root Cause 分析
- diff_drive 插件只驱动 **2个后轮**（rear_left/right_wheel_joint）
- 前轮为 passive continuous joint，摩擦力矩不足以实现有效转向
- 缺少真正的四轮驱动（4WD）配置

## 2. 目标

为 Robot Adam 实现 6 种标准运控方式：

| # | 运控类型 | 激光雷达 | 驱动方式 | 控制插件 | 轮子配置 |
|---|---------|---------|---------|---------|---------|
| 1 | 2WD | 3D (Mid360) | 后两轮驱动 | libgazebo_ros_diff_drive.so | 后轮 continuous（驱动），前轮 continuous（自由旋转） |
| 2 | 4WD | 3D (Mid360) | 四轮驱动 | libgazebo_ros_diff_drive.so (num_wheel_pairs=2) | 四轮全部 continuous |
| 3 | Omni | 3D (Mid360) | 全向移动 | libgazebo_ros_planar_move.so | 四轮 fixed  |
| 4 | 2WD | 2D (Lidar) | 后两轮驱动 | libgazebo_ros_diff_drive.so | 后轮 continuous（驱动），前轮 continuous（自由旋转） |
| 5 | 4WD | 2D (Lidar) | 四轮驱动 | libgazebo_ros_diff_drive.so (num_wheel_pairs=2) | 四轮全部 continuous |
| 6 | Omni | 2D (Lidar) | 全向移动 | libgazebo_ros_planar_move.so | 四轮 fixed  |

### 2.1 设计说明

- **2WD（从动轮）**：
  - 后轮：continuous joint，gazebo_ros_diff_drive 驱动
  - 前轮：**必须设计为万向轮（Caster Wheel）**，不能只是 continuous
  - 转向原理：后轮差速驱动，前轮通过万向结构被动跟随
  - **前轮万向实现方案（需调试选定）**：
    - **方案 A**：使用球形碰撞体（Sphere），mu1/mu2=0。球体无方向性，任意方向均可自由滚动
    - **方案 B**：两层 joint 结构 — 上层 revolute（绕 Z 轴左右偏航）+ 下层 continuous（绕 Y 轴前后滚动），近似真实万向轮
    - **方案 C**：保留圆柱体，mu1/mu2=0 或极小值（0.01），允许无条件侧滑

- **4WD（四轮驱动）**：
  - 四轮：全部 continuous joint
  - 使用 gazebo_ros_diff_drive 的 num_wheel_pairs=2 配置
  - 前后轮同时驱动，转向更稳定

- **Omni（全向移动）**：
  - 四轮：全部 fixed joint（仅视觉模型），或 continuous + mu=0，或去除 collision
  - 使用 gazebo_ros_planar_move 直接控制 base_footprint 位置/姿态
  - 运动丝滑但无物理约束
  - **Omni 轮子碰撞处理方案（需调试选定）**：
    - **方案 A**：去除轮子 collision 标签，仅保留 visual。优点是无物理干涉，运动最丝滑；缺点是轮子穿透地面/物体
    - **方案 B**：使用真正的麦克纳姆轮物理插件（`libgazebo_ros_mecanum_drive.so` 或等效），优点是有真实物理反馈；缺点是复杂度高
    - **方案 C**：轮子设为 continuous joint，mu1/mu2=0。优点是保留碰撞箱但不产生摩擦力；缺点是轮子可能产生异常物理行为

## 3. 目录结构

```
src/simulation/robot_description/          # 机器人描述包
    ├── urdf/
    │   ├── common/
    │   │   ├── base.xacro              # 底盘 link 定义（base_footprint + base_link）
    │   │   ├── wheel.xacro             # 统一轮子宏：joint_type、mu 作为参数复用
    │   │   └── sensors/
    │   │       ├── mid360_macro.xacro       # 3D 激光雷达
    │   │       ├── lidar_2d_macro.xacro    # 2D 激光雷达
    │   │       ├── imu_macro.xacro          # IMU 传感器
    │   │       └── camera_macro.xacro      # 单目相机
    │   ├── controllers/
    │   │   ├── diff_drive_2wd.xacro     # 2WD 差速控制器插件
    │   │   ├── diff_drive_4wd.xacro    # 4WD 差速控制器插件
    │   │   └── omni_drive.xacro        # 全向移动控制器插件（wheel_odom）
    │   ├── plugins/
    │   │   └── gazebo_sensors.xacro    # Gazebo 传感器插件（相机、IMU）
    │   └── robots/
    │       ├── mid360_2wd.urdf.xacro   # 3D + 2WD
    │       ├── mid360_4wd.urdf.xacro   # 3D + 4WD
    │       ├── mid360_omni.urdf.xacro  # 3D + Omni
    │       ├── laser_2wd.urdf.xacro     # 2D + 2WD
    │       ├── laser_4wd.urdf.xacro     # 2D + 4WD
    │       └── laser_omni.urdf.xacro    # 2D + Omni
    ├── launch/
    ├── config/rviz/
    ├── worlds/                         # 仿真世界地图（包含 linorobot2 的 world 文件）
    └── package.xml
```

## 4. 机器人组成

每个机器人变体都包含以下传感器：

| 传感器 | Mid360 版本 | Laser 版本 |
|--------|------------|------------|
| 3D激光雷达 | Mid360 (Livox) | 无 |
| 2D激光雷达 | 无 | LMS291 或等效 |
| IMU | IMU (如 BMI088) | IMU |
| 相机 | 单目相机 | 单目相机 |
| 里程计 | /odom | /odom |

## 5. 物理参数

### 5.1 底盘物理参数（继承现有工程和 linorobot2）
| 参数 | 值 | 说明 |
|------|-----|------|
| 底盘尺寸 | 240 x 180 x 80 mm | |
| 底盘质量 | 1.5 kg | 继承现有 adam_description |
| 底盘惯性张量 | ixx=0.005, iyy=0.003, izz=0.007 | 基于 box 计算 |
| 前后轴距 | 160 mm | |
| 轮距 | 170 mm | |
| 轮子半径 | 40 mm | |
| 轮子宽度 | 32 mm | |
| 轮子质量 | 0.1 kg | 继承现有配置 |
| 轮子惯性张量 | ixx=0.0000485, iyy=0.00008, izz=0.0000485 | **正确公式**：Iyy=½mr²（滚动轴），Ixx=Izz=1/12mh²+¼mr²（侧向轴） |
| COM 高度 | 30 mm | |
| 整体质量 | 约 2.0 kg（含传感器） | |

**惯性张量计算参考**：

### 5.2 控制器参数（参考 linorobot2）

**Diff Drive (2WD/4WD):**
```xml
<updateRate>100</updateRate>
<wheel_separation>0.17</wheel_separation>
<wheel_diameter>0.08</wheel_diameter>
<max_wheel_acceleration>1.0</max_wheel_acceleration>
<max_wheel_torque>100</max_wheel_torque>
<robot_base_frame>base_link</robot_base_frame>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<publish_wheel_tf>false</publish_wheel_tf>
<odometry_frame>odom</odometry_frame>
<odometry_source>1</odometry_source>
```

**重要**：所有控制器（2WD/4WD/Omni）的 `<robot_base_frame>` 必须设为 `base_link`，禁止使用 `base_footprint`。base_footprint 到 base_link 的变换由 robot_state_publisher 维护，插件直接控制 base_link 可确保物理碰撞约束不被绕过。

**4WD 特有配置（diff_drive_4wd.xacro 必须包含）：**
```xml
<!-- covariance 参数：控制里程计不确定性，改善 EKF/融合效果 -->
<covariance_x>0.0001</covariance_x>
<covariance_y>0.0001</covariance_y>
<covariance_yaw>0.01</covariance_yaw>
<!-- wheel_separation：4WD 转向时前后轮拖拽导致有效动力学轮距 > 物理轮距 -->
<!-- 通常乘以 1.1~1.3 修正系数，即设为 0.19~0.22，需通过实测校准 -->
```

**Omni Drive (Planar Move):**
```xml
<update_rate>100</update_rate>
<max_vel_x>0.8</max_vel_x>
<max_vel_theta>0.6</max_vel_theta>
<max_accel_x>0.4</max_accel_x>
<max_accel_theta>0.3</max_accel_theta>
<robot_base_frame>base_link</robot_base_frame>
<publish_odom>true</publish_odom>
<publish_odom_tf>false</publish_odom_tf>
<odometry_topic>wheel_odom</odometry_topic>
```

**注意**：Omni 使用的 planar_move 插件默认发布到 `/odom`，但与 diff_drive 版本共用 `/odom` 话题名会在多模式切换时产生冲突。因此 Omni 版本将里程计话题重命名为 `wheel_odom`，外部节点通过 topic remapping 统一订阅 `/odom`

### 5.3 摩擦参数（分运控方式独立配置）

**2WD 摩擦参数：**
| 部件 | mu1 | mu2 | 说明 |
|------|-----|-----|------|
| 底盘 | 0.8 | 0.8 | |
| 前轮（从动万向轮） | 0.0 | 0.0 | 必须极小或 0，否则侧向卡死无法转向 |
| 后轮（驱动轮） | 1.0 | 1.0 | 需要足够抓地力驱动 |

**4WD 摩擦参数：**
| 部件 | mu1 | mu2 |
|------|-----|-----|
| 底盘 | 0.8 | 0.8 |
| 四轮（全部驱动） | 1.0 | 1.0 |

**Omni 摩擦参数：**
| 部件 | mu1 | mu2 |
|------|-----|-----|
| 底盘 | 0.8 | 0.8 |
| 轮子 | 见 "Omni 轮子碰撞处理方案" 调试选定 |

**摩擦微调说明：**
- 2WD 前轮：mu 必须设为 0 或极小值（0.01~0.1），否则 continuous joint 只能前后滚动、侧向刮擦，导致转向失败
- 4WD 四轮同时受扭力，侧向摩擦力过大可能导致转向不足或抖动，可尝试降低至 0.5~0.8 迭代调试
- 各运控方式使用独立的 wheels_*.xacro 文件，摩擦参数互不影响

**Omni 轮子碰撞处理方案（需调试选定）：**
- **方案 A**：去除轮子 collision 标签，仅保留 visual。优点是无物理干涉，运动最丝滑；缺点是轮子穿透地面/物体
- **方案 B**：使用真正的麦克纳姆轮物理插件（`libgazebo_ros_mecanum_drive.so` 或等效），优点是有真实物理反馈；缺点是复杂度高
- **方案 C**：轮子设为 continuous joint，mu1/mu2=0。优点是保留碰撞箱但不产生摩擦力；缺点是轮子可能产生异常物理行为

## 6. 实现步骤

### Phase 1: 创建包结构
- [ ] 创建 `src/simulation/robot_description/` 目录
- [ ] 创建子目录结构
- [ ] 创建 package.xml
- [ ] 创建 CMakeLists.txt

### Phase 2: 创建公共组件
- [ ] 创建 common/base.xacro（底盘 link）
- [ ] 创建 common/wheel.xacro（统一轮子宏，支持单层 joint 或双层 caster 结构）
- [ ] 创建 common/caster_wheel.xacro（2WD 前轮专用：双层 joint 万向轮，当调试选定方案 B 时使用）
- [ ] 创建 common/sensors/mid360_macro.xacro
- [ ] 创建 common/sensors/lidar_2d_macro.xacro
- [ ] 创建 common/sensors/imu_macro.xacro
- [ ] 创建 common/sensors/camera_macro.xacro

### Phase 3: 创建控制器
- [ ] 创建 controllers/diff_drive_2wd.xacro（2轮差速）
- [ ] 创建 controllers/diff_drive_4wd.xacro（4轮差速，参考 skid_steer）
- [ ] 创建 controllers/omni_drive.xacro（planar_move）
- [ ] 创建 plugins/gazebo_sensors.xacro（相机、IMU插件）

### Phase 4: 创建机器人 URDF
- [ ] 创建 robots/mid360_2wd.urdf.xacro
- [ ] 创建 robots/mid360_4wd.urdf.xacro
- [ ] 创建 robots/mid360_omni.urdf.xacro
- [ ] 创建 robots/laser_2wd.urdf.xacro
- [ ] 创建 robots/laser_4wd.urdf.xacro
- [ ] 创建 robots/laser_omni.urdf.xacro

### Phase 5: 创建 Launch 文件
- [ ] 创建 6 个 launch 文件
- [ ] 创建 rviz 配置文件

### Phase 6: 添加 World 文件
- [ ] 复制 `linorobot2/linorobot2_gazebo/worlds/*.world` 到 `robot_description/worlds/`
- [ ] 保留现有的 bigH.world
- [ ] 添加 empty.world

### Phase 7: 编译测试
- [ ] 编译 simulation/robot_description 包
- [ ] **测试 1**: mid360_2wd - 直走 + 转向 + 传感器
- [ ] **测试 2**: mid360_4wd - 直走 + 转向 + 传感器
- [ ] **测试 3**: mid360_omni - 直走 + 转向 + 传感器
- [ ] **测试 4**: laser_2wd - 直走 + 转向 + 传感器
- [ ] **测试 5**: laser_4wd - 直走 + 转向 + 传感器
- [ ] **测试 6**: laser_omni - 直走 + 转向 + 传感器

## 7. 传感器功能验收

### 7.1 激光雷达
- [ ] Mid360: /livox/lidar 正常发布点云
- [ ] Laser 2D: /scan 正常发布激光数据

### 7.2 IMU
- [ ] /imu/data 或 /imu topic 正常发布 IMU 数据

### 7.3 相机
- [ ] /camera_sensor/image_raw 正常发布图像

### 7.4 里程计
- [ ] /odom 正常发布位置和姿态

## 8. 运控功能验收

### 8.1 功能验收
- [ ] cmd_vel 命令可正常控制机器人运动
- [ ] /odom 话题正常发布

### 8.2 性能验收
- [ ] **2WD**: 直走正常，转向有效（odom yaw 变化）
- [ ] **4WD**: 直走正常，转向更稳定
- [ ] **Omni**: 全向移动丝滑，无卡顿，无翻车

### 8.3 测试方法
```bash
# 直走测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 右转测试（验证转向）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"

# 原地旋转测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"
```

### 8.4 判定标准
- z ≈ 0.03~0.06m（接近地面，无翻车）
- orientation x/y ≈ 0（无倾斜）
- 位置和姿态随时间变化（运动有效）
- 传感器话题有数据输出

### 8.5 全向轮（Omni）专项测试方法

全向轮核心优势在于全向解耦控制，需利用 `linear.y` 进行多维度压测：

```bash
# 1. 纯横移测试（Strafing）- 验证 Y 轴控制
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"

# 2. 45度斜向漂移测试 - 验证 X/Y 轴复合解耦
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5}, angular: {z: 0.0}}"

# 3. 绕圆心"刷圈"测试（公转且车头始终朝内/朝前）- 验证全向车的极限复合运动
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.5}}"
```

### 8.6 全向轮（Omni）专项判定标准

**纯横移测试标准（定量指标）：**
发送 `linear.y = 0.5` 的 3 秒内：
- 里程计 `/odom` 的 `pose.pose.position.x` 波动量应 < ±0.02m（证明没有向前/向后偷跑）
- 车头姿态 `orientation.z` 波动量应 < ±0.01rad（证明没有发生自转打滑）
- `position.y` 必须呈线性单调递增

**45度斜向漂移测试标准（定量指标）：**
- `/odom` 记录的运动轨迹中，X 轴位移 ΔX 与 Y 轴位移 ΔY 的比值应在 0.95 ~ 1.05 之间
- 偏航角 `orientation.z` 应始终保持为 0

**动态响应时间（定量指标）：**
- 当 `ros2 topic pub` 命令执行结束后（触发了 0.5s 超时机制），车辆必须在 0.2 秒内完全静止
- 通过 `/odom` 验证最终位置不再发生微小漂移
- Planar_move 插件容易在停止后产生惯性滑行，需要严格卡死减速度

### 8.7 全向轮（Omni）/odom 指标检查对照表

| 测试 | 指标 | 允许范围 |
|------|------|---------|
| 纯横移 | position.x 波动 | < ±0.02m |
| 纯横移 | orientation.z 波动 | < ±0.01rad |
| 纯横移 | position.y 趋势 | 线性单调递增 |
| 45° 斜向 | ΔX / ΔY 比值 | 0.95 ~ 1.05 |
| 45° 斜向 | 偏航角 orientation.z | ≈ 0 |
| 动态响应 | 停止到静止 | < 0.2s |
| 动态响应 | 停止后位置漂移 | 0 |

## 9. 测试流程（必须遵循）

根据 **cerebrum.md Do-Not-Repeat** 中的经验：

```
1. 修改阶段: 编辑代码 → 保存文件
2. 编译阶段: colcon build --packages-select robot_description 或 ./build_sim.sh
3. 环境准备: source install/setup.bash
4. 仿真准备:
   - 检查残留: source /opt/ros/humble/setup.bash && ros2 node list
   - 有残留则运行: bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
   - 验证清理成功 (应显示 "PASS: No residual simulation nodes detected")
5. 启动测试: ros2 launch robot_description <launch_file>
6. 测试执行: 使用 ros2 topic pub 发送控制命令，观察 /odom 等话题验证状态
7. 测试结束: 清理进程（使用 ros-simulation-clean skill）以准备后续操作
```

## 10. 依赖

- ROS2 Humble
- Gazebo Classic 11
- gazebo_ros_pkgs
- Livox SDK2 (用于 Mid360)
- 现有 adam_description 的传感器定义

## 11. 参考实现

linorobot2 项目提供了成熟的实现参考：
- `/home/pi/Documents/projects/ros2_navigation_stvl_humble/linorobot2/linorobot2_description/urdf/controllers/`
  - `diff_drive.urdf.xacro` - 2WD 配置
  - `skid_steer.urdf.xacro` - 4WD 配置
  - `omni_drive.urdf.xacro` - Omni 配置

linorobot2 的 world 文件：
- `/home/pi/Documents/projects/ros2_navigation_stvl_humble/linorobot2/linorobot2_gazebo/worlds/`
  - 复制到 `robot_description/worlds/` 目录

## 12. 注意事项

1. **use_sim_time 全局硬性约束**：所有 Launch 文件必须为所有节点（robot_state_publisher、joint_state_publisher、rviz2、传感器节点等）设置 `use_sim_time:=true`。否则节点读取系统墙上时间（Wall Time），与 Gazebo 发布的仿真时间（Sim Time）产生时间戳错位，导致 TF 变换全面崩溃。
2. **统一 wheel.xacro**：轮子几何参数复用，通过 `joint_type`、`mu1`、`mu2` 参数区分不同配置。
3. **2WD 前轮万向设计**：必须避免 "continuous 只能滚、侧向卡死" 的问题。根据实际效果选定：球形碰撞+mu=0 / 双层 joint / mu=0 圆柱体
4. **2WD 摩擦参数隔离**：前轮 mu 必须极小（0 或 0.01~0.1），否则转向失败；后轮 mu=1.0 保证驱动力
5. **4WD 四轮驱动**：使用 num_wheel_pairs=2 配置，四轮 joint 全部传入控制器
6. **Omni 里程计话题重命名**：planar_move 发布到 `wheel_odom`，避免与 diff_drive 的 `/odom` 冲突
7. **Omni 轮子碰撞处理**：三种方案（去 collision / 真麦克纳姆插件 / continuous+mu=0）需调试选定
8. **测试流程**：每次测试前必须清理残留进程，使用 ros-simulation-clean skill
9. **传感器必须正常工作**：所有传感器话题都需要验证有数据输出