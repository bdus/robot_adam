# 仿真机器人描述包设计文档

> 基于 `docs/spec/simulation_robot_control_spec.md` 实现 6 种运控方式仿真小车
> 设计决策 + 物理参数 + 实现要点 + 完整测试验收方案

## 1. 设计决策

### 1.1 2WD 前轮万向轮方案 — 方案 A（球体方案）

与 linorobot2 一致的球形万向轮方案：
- 碰撞体使用 **球体（Sphere）**，半径约为 `(wheel_radius - com_z) / 2 ≈ 5mm`
- **fixed joint**（非 continuous），不设驱动轴
- **mu1=0.01, mu2=0.01**（极低摩擦），允许任意方向自由滑动
- 位置：前后轴正中央（`x = base_length/2 - sphere_radius`）
- 参考文件：`linorobot2/linorobot2_description/urdf/mech/caster_wheel.urdf.xacro`

**为什么不是方案 B（双层 joint）或方案 C（圆柱+mu=0）：**
- 方案 B 复杂度高，两层 joint 在 Gazebo 中容易产生数值不稳定
- 方案 C 圆柱体侧向仍会卡死，mu=0 只能解决滑动摩擦力但几何碰撞仍会干涉
- 方案 A 球体无方向性，最简单可靠，linorobot2 已验证

### 1.2 Omni 轮子碰撞方案 — 方案 A（去 collision）

- 四轮去除 collision 标签，仅保留 visual
- joint 类型：fixed（纯视觉模型）
- **原因**：planar_move 直接控制 base_footprint 的位姿，轮子碰撞体只会产生不必要的物理阻力
- mu1/mu2 不适用（无 collision），在 `<gazebo reference>` 中不需要配置摩擦

### 1.3 传感器复用策略

不复制传感器文件，全部通过 `$(find adam_description)` 引用现有宏：

| 传感器 | 引用路径 |
|--------|---------|
| Mid360 3D 激光雷达 | `$(find adam_description)/urdf/sensors/mid360.xacro` |
| 2D 激光雷达 (LMS291) | `$(find adam_description)/urdf/sensors/laser_2d.xacro` |
| IMU (BMI088) | `$(find adam_description)/urdf/sensors/imu.xacro` |
| 单目相机 | `$(find adam_description)/urdf/sensors/mono_camera.xacro` |

Gazebo 传感器插件也复用现有文件：
- `$(find adam_description)/urdf/plugins/gazebo_sensors.xacro`

### 1.4 控制器统一配置

所有控制器的 `<robot_base_frame>` 统一使用 **base_link**（禁止使用 base_footprint）。
base_footprint 到 base_link 的变换由 robot_state_publisher 维护。

**Omni 里程计话题重命名**：
- planar_move 插件发布到 `wheel_odom`（避免与 diff_drive 的 `/odom` 冲突）
- 外部节点通过 topic remapping 统一订阅 `/odom`

### 1.5 use_sim_time 全局硬性约束

所有 Launch 文件必须为所有节点设置 `use_sim_time:=true`，包括：
- robot_state_publisher
- joint_state_publisher  
- rviz2

否则节点读取墙上时间（Wall Time），与 Gazebo 仿真时间（Sim Time）产生时间戳错位。

---

## 2. 物理参数

### 2.1 底盘物理参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 底盘尺寸 | 240 x 180 x 80 mm | 继承 adam_description |
| 底盘质量 | 1.5 kg | 同现有配置 |
| 底盘惯性张量 | ixx=0.005, iyy=0.003, izz=0.007 | 基于 box 公式计算 |
| 前后轴距 | 160 mm | 轮子 x 坐标 ±0.08m |
| 轮距 | 170 mm | 轮子 y 坐标 ±0.085m |
| 轮子半径 | 40 mm | |
| 轮子宽度 | 32 mm | |
| 轮子质量 | 0.1 kg | |
| 轮子惯性张量 | ixx=0.0000485, iyy=0.00008, izz=0.0000485 | Iyy=½mr²（滚动轴），Ixx=Izz=1/12mh²+¼mr² |
| COM 高度 | 30 mm | base_footprint 到 base_link 的 z 偏移 |
| 整体质量 | 约 2.0 kg（含传感器） | |

### 2.2 2WD 摩擦参数

| 部件 | mu1 | mu2 | 说明 |
|------|-----|-----|------|
| 底盘 | 0.8 | 0.8 | |
| 前轮（万向球） | 0.01 | 0.01 | 极低摩擦，允许任意方向滑动 |
| 后轮（驱动轮） | 1.0 | 1.0 | 高摩擦保证驱动力 |

### 2.3 4WD 摩擦参数

| 部件 | mu1 | mu2 |
|------|-----|------|
| 底盘 | 0.8 | 0.8 |
| 前左轮（驱动） | 1.0 | 1.0 |
| 前右轮（驱动） | 1.0 | 1.0 |
| 后左轮（驱动） | 1.0 | 1.0 |
| 后右轮（驱动） | 1.0 | 1.0 |

### 2.4 Omni 摩擦参数

| 部件 | mu1 | mu2 | 说明 |
|------|-----|------|------|
| 底盘 | 0.8 | 0.8 | |
| 四轮 | N/A | N/A | 无 collision，无摩擦配置 |

---

## 3. 控制器参数

### 3.1 Diff Drive 2WD

```xml
<update_rate>100</update_rate>
<left_joint>rear_left_wheel_joint</left_joint>
<right_joint>rear_right_wheel_joint</right_joint>
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
<covariance_x>0.0001</covariance_x>
<covariance_y>0.0001</covariance_y>
<covariance_yaw>0.01</covariance_yaw>
```

### 3.2 Diff Drive 4WD

```xml
<update_rate>100</update_rate>
<num_wheel_pairs>2</num_wheel_pairs>
<left_joint>front_left_wheel_joint</left_joint>
<right_joint>front_right_wheel_joint</right_joint>
<left_joint>rear_left_wheel_joint</left_joint>
<right_joint>rear_right_wheel_joint</right_joint>
<wheel_separation>0.17</wheel_separation>
<wheel_separation>0.17</wheel_separation>
<wheel_diameter>0.08</wheel_diameter>
<wheel_diameter>0.08</wheel_diameter>
<max_wheel_acceleration>1.0</max_wheel_acceleration>
<max_wheel_torque>100</max_wheel_torque>
<robot_base_frame>base_link</robot_base_frame>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<publish_wheel_tf>false</publish_wheel_tf>
<odometry_frame>odom</odometry_frame>
<odometry_source>1</odometry_source>
<covariance_x>0.0001</covariance_x>
<covariance_y>0.0001</covariance_y>
<covariance_yaw>0.01</covariance_yaw>
```

**注意**：`<wheel_separation>` 和 `<wheel_diameter>` 必须重复出现两次（每对轮子一次）。

### 3.3 Omni Drive (Planar Move)

```xml
<update_rate>100</update_rate>
<max_vel_x>0.8</max_vel_x>
<max_vel_y>0.8</max_vel_y>
<max_vel_theta>0.6</max_vel_theta>
<max_accel_x>0.4</max_accel_x>
<max_accel_y>0.4</max_accel_y>
<max_accel_theta>0.3</max_accel_theta>
<robot_base_frame>base_link</robot_base_frame>
<publish_odom>true</publish_odom>
<publish_odom_tf>false</publish_odom_tf>
<odometry_topic>wheel_odom</odometry_topic>
```

**注意**：Omni 的 `max_vel_y` 和 `max_accel_y` 设为非零值以支持横向移动，这是与 ackermann 版本的关键区别。

---

## 4. 6 种变体详细配置

### 4.1 变体总表

| # | 文件名 | 激光雷达 | 驱动方式 | 前轮 | 后轮 | 控制插件 |
|---|--------|---------|---------|------|------|---------|
| 1 | mid360_2wd | Mid360 | 后2轮驱动 | sphere万向(fixed, mu=0.01) | continuous(mu=1.0) | diff_drive(1pair) |
| 2 | mid360_4wd | Mid360 | 四轮驱动 | continuous(mu=1.0) | continuous(mu=1.0) | diff_drive(2pairs) |
| 3 | mid360_omni | Mid360 | 全向移动 | fixed(无collision) | fixed(无collision) | planar_move |
| 4 | laser_2wd | LMS291 | 后2轮驱动 | sphere万向(fixed, mu=0.01) | continuous(mu=1.0) | diff_drive(1pair) |
| 5 | laser_4wd | LMS291 | 四轮驱动 | continuous(mu=1.0) | continuous(mu=1.0) | diff_drive(2pairs) |
| 6 | laser_omni | LMS291 | 全向移动 | fixed(无collision) | fixed(无collision) | planar_move |

### 4.2 传感器配置（所有变体通用）

| 传感器 | Mid360 版本 | Laser 版本 |
|--------|------------|------------|
| 3D激光雷达 | Mid360 (Livox) | 无 |
| 2D激光雷达 | 无 | LMS291 或等效 |
| IMU | IMU (如 BMI088) | IMU |
| 相机 | 单目相机 | 单目相机 |
| 里程计 | /odom | /odom |

### 4.3 轮子位置（所有变体通用）

| 轮子 | X (m) | Y (m) | Z (m) |
|------|-------|-------|-------|
| 前左 | 0.08 | 0.085 | -0.05 |
| 前右 | 0.08 | -0.085 | -0.05 |
| 后左 | -0.08 | 0.085 | -0.05 |
| 后右 | -0.08 | -0.085 | -0.05 |

---

## 5. 目录结构

```
src/simulation/robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── common/
│   │   ├── base.xacro              # 底盘 link 定义（base_footprint + base_link）
│   │   ├── wheel.xacro             # 统一轮子宏（joint_type、mu 参数化）
│   │   └── caster_wheel.xacro     # 2WD 万向轮宏（球体方案，mu=0.01）
│   ├── controllers/
│   │   ├── diff_drive_2wd.xacro    # 2WD 差速控制器插件
│   │   ├── diff_drive_4wd.xacro    # 4WD 差速控制器插件 (num_wheel_pairs=2)
│   │   └── omni_drive.xacro        # 全向移动控制器插件 (planar_move)
│   ├── plugins/
│   │   └── gazebo_sensors.xacro    # Gazebo 传感器插件（空壳，传感器在各自 xacro 中）
│   └── robots/
│       ├── mid360_2wd.urdf.xacro   # 3D + 2WD
│       ├── mid360_4wd.urdf.xacro   # 3D + 4WD
│       ├── mid360_omni.urdf.xacro  # 3D + Omni
│       ├── laser_2wd.urdf.xacro    # 2D + 2WD
│       ├── laser_4wd.urdf.xacro    # 2D + 4WD
│       └── laser_omni.urdf.xacro   # 2D + Omni
├── launch/
│   ├── mid360_2wd.launch.py
│   ├── mid360_4wd.launch.py
│   ├── mid360_omni.launch.py
│   ├── laser_2wd.launch.py
│   ├── laser_4wd.launch.py
│   └── laser_omni.launch.py
├── config/rviz/
│   ├── mid360_2wd.rviz
│   ├── mid360_4wd.rviz
│   ├── mid360_omni.rviz
│   ├── laser_2wd.rviz
│   ├── laser_4wd.rviz
│   └── laser_omni.rviz
└── worlds/
    ├── empty.world
    └── bigH.world
```

---

## 6. 测试方法（完整）

### 6.1 测试流程（必须遵循）

根据 cerebrum.md Do-Not-Repeat 的经验，每次测试必须走完整流程：

```
1. 修改阶段: 编辑代码 → 保存文件
2. 编译阶段: colcon build --packages-select robot_description 或 ./build_sim.sh
3. 环境准备: source install/setup.bash
4. 仿真准备:
   - 检查残留: source /opt/ros/humble/setup.bash && ros2 node list
   - 有残留则运行: bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
   - 验证清理成功 (应显示 "PASS: No residual simulation nodes detected")
5. 启动测试: ros2 launch robot_description <launch_file> rviz:=true
6. 测试执行: 使用 ros2 topic pub 发送控制命令，观察 /odom 等话题验证状态
7. 测试结束: 清理进程（使用 ros-simulation-clean skill）以准备后续操作
```

### 6.2 测试命令（所有变体通用）

```bash
# 直走测试 - 验证前进
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 右转测试 - 验证转向能力
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"

# 原地旋转测试 - 验证差速转向
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"

# 后退测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}"

# 观察 odom
ros2 topic echo /odom --once
```

### 6.3 判定标准

```bash
ros2 topic echo /odom --once | grep -E "position|orientation"
```

- **z ≈ 0.03~0.06m**（接近地面，无翻车）
- **orientation x/y ≈ 0**（无倾斜）
- 位置和姿态随时间变化（运动有效）
- 传感器话题有数据输出

### 6.4 Omni 全向轮专项测试

```bash
# 1. 纯横移测试（Strafing）- 验证 Y 轴控制
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"

# 2. 45度斜向漂移测试 - 验证 X/Y 轴复合解耦
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5}, angular: {z: 0.0}}"

# 3. 绕圆心"刷圈"测试 - 验证全向车的极限复合运动
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.5}}"
```

### 6.5 Omni 专项判定标准

**纯横移测试标准：**
- 里程计 `/odom` 的 `pose.pose.position.x` 波动量应 < ±0.02m（无向前/向后偷跑）
- 车头姿态 `orientation.z` 波动量应 < ±0.01rad（无自转打滑）
- `position.y` 必须呈线性单调递增

**45度斜向漂移测试标准：**
- `/odom` 记录的运动轨迹中，ΔX 与 ΔY 的比值应在 0.95 ~ 1.05 之间
- 偏航角 `orientation.z` 应始终保持为 0

**动态响应时间：**
- 命令执行结束后，车辆必须在 0.2 秒内完全静止
- 通过 `/odom` 验证最终位置不再发生微小漂移

### 6.6 传感器功能验收

```bash
# 检查所有话题
ros2 topic list

# Mid360 点云
ros2 topic echo /livox/lidar --once

# 2D 激光雷达
ros2 topic echo /scan --once

# IMU
ros2 topic echo /imu/data --once

# 相机
ros2 topic echo /camera_sensor/image_raw --once

# 里程计
ros2 topic echo /odom --once
```

### 6.7 传感器验收标准

| 传感器 | 话题 | 数据类型 | 预期结果 |
|--------|------|---------|---------|
| Mid360 | /livox/lidar | sensor_msgs/PointCloud2 | 正常发布点云 |
| 2D Laser | /scan | sensor_msgs/LaserScan | 正常发布激光数据 |
| IMU | /imu/data | sensor_msgs/Imu | 正常发布 IMU 数据 |
| 相机 | /camera_sensor/image_raw | sensor_msgs/Image | 正常发布图像 |
| 里程计 | /odom | nav_msgs/Odometry | 正常发布位置和姿态 |

### 6.8 各变体测试矩阵

| 测试项 | 2WD | 4WD | Omni | 说明 |
|--------|-----|-----|------|------|
| 直走 x=0.5 | ✅ | ✅ | ✅ | 基础功能 |
| 后退 x=-0.3 | ✅ | ✅ | ✅ | 基础功能 |
| 右转 x=0.5, z=-1.0 | ✅ | ✅ | ✅ | 转向验证 |
| 原地旋转 z=0.8 | ✅ | ✅ | ❌ | planar_move 不支持原地旋转？ |
| 纯横移 y=0.5 | ❌ | ❌ | ✅ | Omni 独有 |
| 45° 斜向 x=0.5, y=0.5 | ❌ | ❌ | ✅ | Omni 独有 |
| 传感器话题 | ✅ | ✅ | ✅ | 所有变体 |
| 翻车检查 | ✅ | ✅ | ✅ | 所有变体 |

---

## 7. 实现步骤

### Phase 1: 创建包结构
- [ ] 创建 `src/simulation/robot_description/` 目录和子目录
- [ ] 创建 `package.xml`（依赖：urdf, xacro, gazebo_ros, rclcpp, sensor_msgs, geometry_msgs）
- [ ] 创建 `CMakeLists.txt`（ament_cmake，安装 urdf/launch/config/worlds）

### Phase 2: 创建公共组件
- [ ] 创建 `common/base.xacro` — 底盘 link（参数化：尺寸、质量、COM高度）
- [ ] 创建 `common/wheel.xacro` — 统一轮子宏（参数化：joint_type, mu1, mu2, side）
- [ ] 创建 `common/caster_wheel.xacro` — 2WD 万向轮（球体方案，mu=0.01）

### Phase 3: 创建控制器
- [ ] 创建 `controllers/diff_drive_2wd.xacro`
- [ ] 创建 `controllers/diff_drive_4wd.xacro`（num_wheel_pairs=2）
- [ ] 创建 `controllers/omni_drive.xacro`（planar_move，odom→wheel_odom）
- [ ] 创建 `plugins/gazebo_sensors.xacro`（空壳）

### Phase 4: 创建机器人 URDF
- [ ] 创建 `robots/mid360_2wd.urdf.xacro`
- [ ] 创建 `robots/mid360_4wd.urdf.xacro`
- [ ] 创建 `robots/mid360_omni.urdf.xacro`
- [ ] 创建 `robots/laser_2wd.urdf.xacro`
- [ ] 创建 `robots/laser_4wd.urdf.xacro`
- [ ] 创建 `robots/laser_omni.urdf.xacro`

### Phase 5: 创建 Launch 文件
- [ ] 创建 6 个 `launch/*.launch.py`（继承现有模式，use_sim_time 全局约束）

### Phase 6: 创建 RViz 配置
- [ ] 基于现有 rviz 配置创建 6 个变体的配置

### Phase 7: 添加 World 文件
- [ ] 复制 `empty.world`（从 adam_description/world/）
- [ ] 复制 `bigH.world`（从 adam_description/world/）

### Phase 8: 编译与测试
- [ ] colcon build --packages-select robot_description
- [ ] 测试 1: mid360_2wd — 直走 + 转向 + 传感器
- [ ] 测试 2: mid360_4wd — 直走 + 转向 + 传感器
- [ ] 测试 3: mid360_omni — 直走 + 横移 + 传感器
- [ ] 测试 4: laser_2wd — 直走 + 转向 + 传感器
- [ ] 测试 5: laser_4wd — 直走 + 转向 + 传感器
- [ ] 测试 6: laser_omni — 直走 + 横移 + 传感器

---

## 8. 注意事项

1. **use_sim_time 全局硬性约束**：所有节点必须设置 `use_sim_time:=true`
2. **统一 wheel.xacro**：轮子几何参数复用，通过 joint_type、mu1、mu2 参数区分不同配置
3. **2WD 前轮万向设计**：球体方案，fixed joint，mu=0.01
4. **2WD 摩擦参数隔离**：前轮 mu=0.01，后轮 mu=1.0
5. **4WD 四轮驱动**：num_wheel_pairs=2，四轮 joint 全部传入控制器
6. **Omni 里程计话题重命名**：planar_move 发布到 wheel_odom
7. **Omni 轮子碰撞处理**：去除 collision 标签，纯视觉
8. **测试流程**：每次测试前必须清理残留进程（ros-simulation-clean skill）
9. **传感器必须正常工作**：所有传感器话题都需要验证有数据输出
