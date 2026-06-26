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
| COM 高度 | 4cm → 3cm | 降低重心减少侧翻力矩 |
| 轮子 Y 坐标 | ±0.07 → ±0.085 | 匹配加宽的底盘 |
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
