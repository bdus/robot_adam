# 仿真机器人描述包 - 6种驱动方式

本包提供6种不同的驱动方式仿真小车：
- 2WD/4WD/Omni × Mid360 3D激光雷达
- 2WD/4WD/Omni × 2D激光雷达

## 快速启动

### 启动 Gazebo 仿真

#### Mid360 3D激光雷达版本
```bash
# 2WD 差速驱动
ros2 launch robot_description mid360_2wd.launch.py

# 4WD 差速驱动
ros2 launch robot_description mid360_4wd.launch.py

# 全向移动 (Omni)
ros2 launch robot_description mid360_omni.launch.py
```

#### 2D激光雷达版本
```bash
# 2WD 差速驱动
ros2 launch robot_description laser_2wd.launch.py

# 4WD 差速驱动
ros2 launch robot_description laser_4wd.launch.py

# 全向移动 (Omni)
ros2 launch robot_description laser_omni.launch.py
```

### 控制机器人

```bash
# 发布速度命令 (linear.x: 前后速度 m/s, angular.z: 转向速度 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### 常用测试命令

#### 基础运动测试
```bash
# 前进测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# 左转测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}"

# 右转测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"

# 后退测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}"

# 原地旋转测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"
```

#### 全向移动专项测试 (仅Omni变体)
```bash
# 纯横移测试 (y轴方向)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"

# 45度斜向漂移测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5}, angular: {z: 0.0}}"

# 圆周运动测试
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0}, angular: {z: 0.5}}"
```

### 查看传感器数据

```bash
# 查看所有激活的话题
ros2 topic list

# Mid360 3D激光雷达版本话题:
#   /livox/lidar                - 3D 点云 (30000点/帧)
#   /imu/data                   - IMU 数据 (加速度、角速度、姿态)
#   /camera_sensor/image_raw    - 相机图像 (640x480 RGB8)
#   /odom或/wheel_odom          - 里程计 (Omni使用/wheel_odom避免与diff_drive的/odom冲突)
#   /joint_states               - 关节状态

# 2D激光雷达版本话题:
#   /scan                       - LaserScan (360度, 10m范围)
#   /imu/data                   - IMU 数据
#   /camera_sensor/image_raw    - 相机图像 (640x480 RGB8)
#   /odom或/wheel_odom          - 里程计
#   /joint_states               - 关节状态

# 查看具体数据示例
ros2 topic echo /livox/lidar --once   # 查看点云数据
ros2 topic echo /scan --once          # 查看激光线激光数据
ros2 topic echo /imu/data --once      # 查看IMU数据
ros2 topic echo /odom --once          # 查看里程计 (2WD/4WD)
ros2 topic echo /wheel_odom --once    # 查看里程计 (Omni)
```

### 仿真环境清理

仿真测试结束后，建议使用以下命令清理残留进程：

```bash
# 使用内置清理脚本
. .claude/skills/ros-simulation-clean/clean.sh

# 或者手动执行
bash .claude/skills/ros-simulation-clean/clean.sh
```

这将清理：
- /gazebo, /joint_state_publisher, /robot_state_publisher 等仿真节点
- Gazebo 相关进程 (gzserver, gzclient)
- ROS daemon 缓存

### 技术说明

- 所有启动文件强制设置 `use_sim_time:=true` 以同步仿真时间
- 机器人基坐标框架统一为 `base_link` (不使用 `base_footprint`)
- Omni变体的里程计发布到 `/wheel_odom` 话题，避免与差速驱动的 `/odom` 冲突
- 传感器均通过复用 `adam_description` 包中的定义，避免代码重复