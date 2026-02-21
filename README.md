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
└── adam_bringup/              # 启动包
    └── launch/
        └── simulation.launch.py
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
| 底盘尺寸 | 240 x 140 x 80 mm |
| 前后轴距 | 160 mm |
| 轮距 | 140 mm |
| 车轮半径 | 40 mm |
| 最大转向角 | ±40° (0.7 弧度) |
| 最大速度 | 1.0 m/s |

## 安装

### 依赖

```bash
sudo apt install ros-<distro>-gazebo-ros*
sudo apt install ros-<distro>-ackermann-steering-controller
sudo apt install ros-<distro>-joint-state-broadcaster
sudo apt install ros-<distro>-robot-state-publisher
sudo apt install ros-<distro>-gazebo-ros2-control
```

### 编译

```bash
cd ~/robot_adam
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 启动 Gazebo 仿真

#### Mid360 版本
```bash
ros2 launch adam_description gazebo_ackermann_mid360.launch.py
```

#### 2D 激光版本
```bash
ros2 launch adam_description gazebo_ackermann_laser.launch.py
```

#### 使用统一启动文件
```bash
# 启动 Mid360 版本
ros2 launch adam_bringup simulation.launch.py model:=ackermann_mid360

# 启动 2D 激光版本
ros2 launch adam_bringup simulation.launch.py model:=ackermann_laser
```

### 控制机器人

```bash
# 发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 查看传感器数据

```bash
# 查看话题列表
ros2 topic list

# Mid360 版本话题:
#   /livox/lidar      - 3D 点云
#   /livox/imu        - IMU 数据
#   /camera/image_raw - 相机图像
#   /odom             - 里程计
#   /joint_states     - 关节状态

# 2D 激光版本话题:
#   /scan             - LaserScan
#   /imu/data         - IMU 数据
#   /camera/image_raw - 相机图像
#   /odom             - 里程计
#   /joint_states     - 关节状态

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
2. **ROS2 Control**: 通过更改 `ros2_control` 中的 `hardware` 插件来切换仿真/实车
3. **控制器**: 使用相同的 `ackermann_steering_controller` 配置

### 实车配置

在实车上使用时，需要修改 `urdf/plugins/ros2_control.xacro`:

```xml
<hardware>
    <!-- 仿真 -->
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    <!-- 实车 -->
    <!-- <plugin>your_real_robot_driver/YourHardwareInterface</plugin> -->
</hardware>
```

## 话题和服务

### 发布的话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| /cmd_vel | geometry_msgs/Twist | 速度指令 |
| /odom | nav_msgs/Odometry | 里程计 |
| /joint_states | sensor_msgs/JointState | 关节状态 |
| /scan | sensor_msgs/LaserScan | 2D 激光雷达 |
| /livox/lidar | sensor_msgs/PointCloud2 | 3D 点云 |
| /imu/data | sensor_msgs/Imu | IMU 数据 |
| /camera/image_raw | sensor_msgs/Image | 相机图像 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| /ackermann_steering_controller/set_odom_pose | geometry_msgs/PoseWithCovarianceStamped | 设置里程计位姿 |

## RViz 可视化

启动时会自动打开 RViz，也可以手动启动：

```bash
# Mid360 版本
rviz2 -d src/adam_description/config/rviz/ackermann_mid360.rviz

# 2D 激光版本
rviz2 -d src/adam_description/config/rviz/ackermann_laser.rviz
```

## 后续计划

1. **Navigation2 集成**: 添加导航配置和参数
2. **SLAM 集成**: 支持 Cartographer/LOAM 等 SLAM 算法
3. **实体车支持**: 添加实车驱动和硬件接口
4. **更多传感器**: 支持深度相机、毫米波雷达等

## 许可证

Apache License 2.0
