# Robot Adam 转弯翻车问题调试记录

## 问题现象

使用 `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1}}"` 或 teleop_twist_keyboard 按 o 键（右上方向）时，小车在 Gazebo 仿真中翻车。

## 环境

- ROS2 Distro: Humble
- Gazebo Classic 11.10.2
- 机器人: ackermann_mid360 (4轮小车 + Mid360 激光雷达)

## Root Cause 分析

### 物理模型问题

1. **轮子为 FIXED joint**：所有 4 个轮子都是固定关节（visual + collision only），用 `libgazebo_ros_planar_move.so` 插件直接施加力在 base_link 上驱动。轮子只是视觉效果，不滚动。

2. **窄轮距（14cm）+ 高 COM（4cm）**：转弯时 lateral acceleration 产生侧翻力矩。轮距窄，力臂小，容易翻。

3. **轮地摩擦过高 (mu=1.0)**：固定轮子 + 高摩擦 = 转弯时侧向拖拽力直接传到底盘。由于轮子不能转动，转弯时轮胎必须侧滑，高摩擦系数产生巨大横向力矩使底盘翻转。

4. **planar_move 插件转弯加速度过大**：`max_vel_theta=1.0`、`max_accel_theta=0.5`，在窄底盘上过于激进。

### 运动控制问题

5. **ros2_control 死代码**：`ros2_control.xacro` 定义了 6 个关节（steering + drive）和 `joint_state_broadcaster`，但这些关节在 URDF 中**不存在**。`ros2_control.xacro` 引用了 `ackermann_controller.yaml` 但没有配置 Ackermann 控制器。实际运动由 `planar_move` 插件直接驱动底盘。

6. **planar_move 插件限制**：仅支持 `cmd_vel` → 平面运动，不支持真正的阿克曼转向几何。它忽略前轮转向角，直接在底盘中心施加力/力矩。

## 修改内容

### 2026-06-27: URDF 稳定性调优

| 修改 | 原值 → 新值 | 理由 |
|------|------------|------|
| 底盘宽度 | 14cm → 18cm | 加宽轮距，增大抗侧翻力矩 |
| 底盘高度 (base_joint) | 4cm → 9cm | 抬高底盘使碰撞箱离开地面，轮子接触地而非底盘刮地 |
| 轮子 Y 坐标 | ±0.07 → ±0.085 | 匹配加宽的底盘 |
| 轮子 Z 坐标 | -0.04 → -0.05 | 轮心在 base_link.z - 0.05 = 0.04 = 轮半径，轮底刚好接触地面 |
| 轮地摩擦 mu1/mu2 | 1.0 → 0.5 | 降低转弯时侧向拖拽力，允许有限滑动 |
| max_vel_theta | 1.0 → 0.6 | 降低最大转弯速度 |
| max_accel_theta | 0.5 → 0.3 | 降低转弯加速度，减少突然的横向冲击 |

### 尚未解决的问题

- ros2_control 关节定义（steering/drive joints）在 URDF 中不存在 → 产生 6 条 `Skipping joint` 警告
- 没有 AckermannController，只有 planar_move
- 机器人底盘碰撞箱宽度被增大到了 18cm（视觉 + 碰撞），需要验证是否与环境物体干涉

## 测试方法

```bash
# 编译
source /opt/ros/humble/setup.bash
colcon build --packages-select adam_description

# 启动
source install/setup.bash
ros2 launch adam_description gazebo_ackermann_mid360.launch.py

# 测试各种运动（在另一个终端）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"  # 右转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}"   # 左转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.5}}"  # 倒车右转
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"   # 直线
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"   # 原地旋转

# 观察里程计确认是否翻车（z 高度应接近 0.05，orientation x/y ≈ 0）
ros2 topic echo /odom --once
```

## 修复验证 (2026-06-27)

### 问题解决确认
- 持续右转测试：z=1e-7（接近地面），orientation x≈y≈0 → **无翻车**
- 机器人移动了 2.66m 仍保持稳定

### 关键修复
1. **删除 ros2_control.xacro**：原来 URDF 包含的 ros2_control.xacro 定义了 6 个关节但这些关节在 Gazebo 模型中不存在，导致 gazebo_ros2_control 加载失败并级联导致 planar_move 插件报错无法运行。删除 ros2_control.xacro 后 planar_move 正常启动。
2. **轮子高度修复**：轮子 joint origin 从 `-0.04` 改为 `-0.05`（wheel center at base_link.z - 0.05 = 0.04 = 轮半径，轮底刚好接触地面，不刮底盘）
3. **底盘高度修复**：base_joint 从 0.03→0.09（base_link COM 抬高到底盘半高位置，避免碰撞箱拖地）
4. **摩擦参数和运动参数**：mu1/mu2 降为 0.5，转弯速度/加速度降低

## 运动控制方案演进 (2026-06-27 后续)

### 方案对比

| | mid360 (当前 diff_drive) | laser (planar_move) |
|---|---|---|
| 控制插件 | `libgazebo_ros_diff_drive.so` | `libgazebo_ros_planar_move.so` |
| 运动原理 | 物理引擎+摩擦驱动（后轮差速） | 直接位置插值（无视物理约束） |
| 前轮 | 被动自由旋转 (continuous joint) | 固定 (fixed joint，视觉模型) |
| 转向方式 | 靠地面摩擦差速转向 | 直接旋转 base_link |
| 直走 | ✅ 正常 | ✅ 正常 |
| 转向 | ❌ 困难（力矩不足） | ✅ 丝滑 |
| 翻车风险 | 低（物理约束） | 高（COM高+加速度大） |

### diff_drive 转向困难分析

发送 `cmd_vel (linear.x=0.5, angular.z=-1)` 后：
- ✅ 直走正常（有前进位移）
- ❌ 转弯效果差（odom yaw 几乎不变，tf rotation y≈0）

**根本原因**: diff_drive 只驱动 **2个后轮** 实现差速转向，前轮为 passive。当摩擦系数过大或 wheel_separation 不当时，转向力矩不足以克服惯性。

当前参数：
- `wheel_separation=0.17m` (正确值，0.085+0.085)
- `wheel_diameter=0.08m` (正确值)
- 后轮摩擦 mu=0.8，底盘摩擦 mu=0.5
- 车轮质量惯性仅 0.1kg (可能偏小)

### 最终方案：planar_move + 防翻车优化

放弃 diff_drive，换回 planar_move，并解决之前翻车的 root cause：

1. **降低 COM**: base_joint z=0.03m (已修改)
2. **限制转弯加速度**: `max_vel_theta=0.6`, `max_accel_theta=0.3`
3. **降低摩擦**: mu1/mu2=0.5 (减少侧向拖拽力)
4. **轮子改为 fixed**: 四轮均为视觉模型 (与 laser 一致)

**尚未解决的问题**
- ros2_control.xacro 保留在源码中但不再被 URDF 引用（关节定义存在但无效）
- 运动仍一卡一卡的原因待进一步分析（planar_move 30Hz + 固定轮子 + 摩擦）
