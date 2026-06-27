# Adam Robot - 阿克曼转向机器人仿真

基于 ROS 2 的阿克曼转向小车仿真项目，支持 Gazebo 仿真和实体车部署。

## 项目结构

```
robot_adam/
├── adam_description/          # 机器人描述包
│   ├── urdf/                  # URDF/Xacro 机器人模型
│   │   ├── common/            # 公共组件
│   │   ├── base/              # 底盘和轮子
│   │   ├── sensors/           # 传感器模型
│   │   ├── plugins/           # Gazebo 和 ROS2 插件
│   │   ├── ackermann_mid360.urdf.xacro  # Mid360 版本
│   │   └── ackermann_laser.urdf.xacro   # 2D 激光版本
│   ├── config/                # 配置文件
│   │   ├── ackermann_controller.yaml    # 控制器配置
│   │   └── rviz/              # RViz 配置
│   ├── launch/                # Launch 文件
│   └── world/                 # Gazebo 世界模型
├── adam_bringup/              # 启动包
└── 3rd_packages/              # 第三方包
    └── livox_ros_driver2/     # Livox 激光雷达驱动
```

## 机器人配置

本项目提供两种传感器配置的阿克曼小车：

### 1. Ackermann Mid360 版本
- **传感器**: Mid360 3D 激光雷达 + IMU + 单目相机
- **用途**: SLAM 和 3D 感知应用

### 2. Ackermann Laser 版本
- **传感器**: 2D 激光雷达 + IMU + 单目相机
- **用途**: 2D 导航和 SLAM 应用

## 技术规格

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 240 x 180 x 80 mm |
| 前后轴距 | 160 mm |
| 轮距 | 180 mm |
| 车轮半径 | 40 mm |
| 最大速度 | 1.0 m/s |
| COM 高度 | 90 mm (低重心设计，防翻车) |

### 运动控制

使用 `diff_drive` 插件实现四轮差速驱动：
- **后轮**: continuous 关节，由 diff_drive 物理驱动（左后+右后）
- **前轮**: continuous 关节，自由旋转（不拖拽，减少阻力）
- **里程计**: 50Hz 刷新率，输出 `/odom`

支持的运动模式：
- 前进/后退直行 (0.5 m/s 约 1.5-1.8m/3秒)
- 左右转向
- 原地旋转
- 前进+转向组合运动
- 无翻车：经过验证的极限指令 `linear.x=0.5, angular.z=-1.0` 可平稳前进右转，z≈0

### 关键改进
1. 四个车轮全部改为 `continuous` 关节（后轮驱动，前轮自由旋转）
2. 车轮摩擦系数提升至 mu1=mu2=1.0（防止打滑）
3. 车轮质量降至 0.1kg（减小惯量
4. diff_drive 最大扭矩提升至 100Nm（确保充足驱动力）
5. 更新 robot_base_frame 为 base_link（修复 odom TF 问题）

## 安装

### 依赖

```bash
# ROS2 基础依赖
sudo apt install ros-humble-gazebo-ros*
sudo apt install ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rqt-tf-tree

# Livox SDK2 (用于 Mid360 激光雷达)
cd /tmp && git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j
sudo -S make install && sudo ldconfig
```

### 编译

```bash
cd ~/robot_adam
./build_sim.sh
source install/setup.bash
```

## 使用方法

### 启动 Gazebo 仿真

#### 3D激光雷达(Mid360) 版本
```bash
ros2 launch adam_description gazebo_planar_mid360.launch.py
```

#### 2D 激光版本
```bash
ros2 launch adam_description gazebo_ackermann_laser.launch.py
```

#### 使用 RViz 可视化
```bash
ros2 launch adam_description gazebo_planar_mid360.launch.py rviz:=true
```

### 控制机器人

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1}}"

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 查看传感器数据

```bash
# 查看话题列表
ros2 topic list

# Mid360 版本话题:
#   /livox/lidar                - 3D 点云
#   /livox/imu                  - IMU 数据
#   /camera_sensor/image_raw    - 相机图像
#   /odom                       - 里程计
#   /joint_states               - 关节状态

# 2D 激光版本话题:
#   /scan                       - LaserScan
#   /imu/data                   - IMU 数据
#   //camera_sensor/image_raw   - 相机图像
#   /odom                       - 里程计
#   /joint_states               - 关节状态

# 查看点云数据
ros2 topic echo /livox/lidar --no-lost

# 查看激光雷达数据
ros2 topic echo /scan --no-lost

# 查看 IMU 数据
ros2 topic echo /imu/data --no-lost
```

## 扩展实车

本项目设计时考虑了仿真与实车的统一性：

1. **URDF 配置**: 仿真和实车使用相同的 URDF 文件
2. **运动控制**: 使用 Gazebo `diff_drive` 插件进行仿真，实车可通过 ROS2 control 框架替换

### 实车配置

实车部署时需要：
1. 将 URDF 中的 `gazebo_control_diff.xacro` 替换为 ros2_control 配置
2. 实现自定义的 hardware 接口插件
3. 使用 `ackermann_steering_controller` 或 `diff_drive_controller`
 
## RViz 可视化

启动时会自动打开 RViz，也可以手动启动：

```bash
# Mid360 版本
rviz2 -d src/adam_description/config/rviz/ackermann_mid360.rviz

# 2D 激光版本
rviz2 -d src/adam_description/config/rviz/ackermann_laser.rviz
```

 
## 仿真清理

仿真结束后，使用 ros-simulation-clean skill 清理残留节点和进程：

```bash
. .claude/skills/ros-simulation-clean/clean.sh
```

这会清理：
- `/gazebo`, `/joint_state_publisher`, `/robot_state_publisher` 等仿真节点
- Gazebo 相关进程
- ROS daemon 缓存

## 许可证

Apache License 2.0
