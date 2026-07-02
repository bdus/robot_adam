# Adam Robot - ROS 2 仿真机器人

基于 ROS 2 Humble 的差速驱动/全向移动小车仿真项目，支持 Gazebo 仿真和实体车部署。

## 快速启动

### 建图模式（一键启动）

```bash
ros2 launch adam_navigation spec_02_all.launch.py mode:=mapping
```

启动内容：Gazebo（bigH 地图）+ EKF 里程计融合 + Cartographer 2D 建图 + Nav2 导航避障 + RViz2

- 使用 `teleop_twist_keyboard` 手动控制，或
- 在 RViz2 中用 "Nav2 Goal" 点选目标，小车自动规划路径并避障
- 地图实时生成，可边探索边导航

### 保存地图

```bash
ros2 run adam_slam archive_map.py               # 保存到缓存目录
ros2 run adam_slam archive_map.py --commit       # 同时同步到源码树
```

地图文件保存在 `~/.ros/adam_assets/maps_2d/` 或 `adam_assets/share/maps_2d/`。

### 定位模式（加载已有地图）

```bash
ros2 launch adam_navigation spec_02_all.launch.py mode:=localize load_state_filename:=/path/to/map.pbstream
```

启动内容：Gazebo + EKF + Cartographer 纯定位 + Nav2 导航

## 驱动方式

详见[仿真机器人描述包使用说明](./src/simulation/README.md)，支持：

- 2WD 差速驱动（当前建图使用）
- 4WD 差速驱动
- Omni 全向移动
- 可选 Mid360 激光雷达或 2D 激光雷达

## 系统架构

```
Gazebo Simulation
  ├─ diff_drive plugin → /wheel_odom (里程计原始数据)
  ├─ IMU plugin → /imu/data
  └─ laser plugin → /scan
         ↓
EKF (robot_localization) → /odometry/filtered + odom→base_link TF
         ↓
Cartographer SLAM → /map + map→odom TF
         ↓
Nav2 (规划 + 控制 + 避障) → /cmd_vel
```

## 编译

```bash
./build_sim.sh
source install/setup.bash
```
