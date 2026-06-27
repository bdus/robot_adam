# OpenWolf

@.wolf/OPENWOLF.md

This project uses OpenWolf for context management. Read and follow .wolf/OPENWOLF.md every session. Check .wolf/cerebrum.md before generating code. Check .wolf/anatomy.md before reading files.

## 编译

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