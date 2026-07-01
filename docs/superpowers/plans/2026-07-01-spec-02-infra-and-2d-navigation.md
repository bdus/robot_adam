# SPEC 02 基础设施与 2D 几何导航栈实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 跑通"常驻基础里程计 → 自主探索建图 → 地图持久化归档 → 一帧无感重定位 → Nav2 运控避障"的完整 2D 黄金闭环

**Architecture:** 6 个独立可交付的原子单元，从资产包骨架开始逐层筑基。L1 diff_drive 发原始 `/wheel_odom` → L2 EKF 融合成 `/odom→base_link` TF → L3 Cartographer 做 SLAM 避障。各层通过标准化 ROS2 接口通信，严禁跨层依赖。

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, Cartographer, Nav2 (Smac + MPPI), robot_localization

## 全局约束

- 基准测试变体：`laser_2wd`（使用 `/scan` 2D 激光 + `/imu/data` IMU + diff_drive 插件）
- TF 链标准：`wheel_odom→base_link`(diff_drive) → `odom→base_link`(EKF) → `map→odom`(Global EKF)
- Cartographer 参数：`provide_odom_frame=false`, `published_frame="odom"`
- 所有 SLAM 算法只发 Topic 不发布 TF
- 测试工具调用 `/test_tools/*` 服务（`adam_test_tools` 包提供）

---

## 文件创建/修改计划

### 新包结构
```
src/
├── simulation/robot_description/          (已有，需小修改)
└── localization_mapping/                  (新建 — SPEC 02 创建)
│   ├── adam_localization/CMakeLists.txt
│   ├── adam_localization/package.xml
│   ├── adam_localization/config/ekf_local.yaml
│   ├── adam_localization/config/ekf_global.yaml
│   ├── adam_localization/launch/localization.launch.py
│   ├── adam_localization/scripts/get_asset_path.py  (shared utility)
│   ├── adam_slam/CMakeLists.txt
│   ├── adam_slam/package.xml
│   ├── adam_slam/config/cartographer_2d.lua
│   ├── adam_slam/config/cartographer_localization.lua
│   ├── adam_slam/config/custom_explorer.yaml
│   ├── adam_slam/launch/cartographer_2d.launch.py
│   ├── adam_slam/launch/cartographer_localization.launch.py
│   └── adam_slam/scripts/archive_map.py
└── navigation_ai/                         (新建)
    ├── adam_assets/CMakeLists.txt
    ├── adam_assets/package.xml
    └── adam_navigation/CMakeLists.txt
    ├── adam_navigation/package.xml
    ├── adam_navigation/config/nav2_2d_config.yaml
    ├── adam_navigation/launch/nav2_bringup.launch.py
    └── adam_navigation/launch/spec_02_all.launch.py  (一键启动全栈)
```

### 修改已有文件
- `src/simulation/robot_description/urdf/controllers/diff_drive_2wd.xacro` — `/odom` → `/wheel_odom`, `odometry_frame=wheel_odom`

---

### Task 1: diff_drive 输出改为 `/wheel_odom`

**Files:**
- Modify: `src/simulation/robot_description/urdf/controllers/diff_drive_2wd.xacro`

**Interfaces:**
- Produces: `/wheel_odom` (nav_msgs/Odometry), TF `wheel_odom→base_link`

- [ ] **Step 1: 修改 topic remapping**

  ```xml
  <remapping>odom:=/wheel_odom</remapping>
  ```
  从 `/odom` 改为 `/wheel_odom`（第 13 行）。

- [ ] **Step 2: 修改 odometry_frame**

  ```xml
  <odometry_frame>wheel_odom</odometry_frame>
  ```
  从 `odom` 改为 `wheel_odom`（第 19 行）。

- [ ] **Step 3: 编译验证**

  ```bash
  source /opt/ros/humble/setup.bash
  cd ~/robot_adam
  colcon build --packages-select robot_description
  ```

- [ ] **Step 4: 验证话题输出**

  ```bash
  source install/setup.bash
  ros2 launch src/simulation/robot_description/launch/laser_2wd.launch.py
  # 新开终端
  ros2 topic list | grep wheel_odom
  # 预期输出: /wheel_odom
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/simulation/robot_description/urdf/controllers/diff_drive_2wd.xacro
  git commit -m "feat: change diff_drive odom topic to /wheel_odom with wheel_odom frame

  L1 底盘原始轮速发布到 /wheel_odom，TF 帧 wheel_odom→base_link。
  L2 local_ekf_node 独占 odom→base_link 的广播权，互不冲突。

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

### Task 2: adam_assets —— 集中式静态资源仓库

**Files:**
- Create: `src/navigation_ai/adam_assets/CMakeLists.txt`
- Create: `src/navigation_ai/adam_assets/package.xml`
- Create: `src/navigation_ai/adam_assets/share/maps_2d/.gitkeep`
- Create: `src/navigation_ai/adam_assets/share/maps_3d/.gitkeep`
- Create: `src/navigation_ai/adam_assets/share/camera_calibration/.gitkeep`

**Interfaces:**
- Provides: 可被 `ament_index_python.packages.get_package_share_directory('adam_assets')` 检索

- [ ] **Step 1: 创建目录结构**

  ```bash
  mkdir -p src/navigation_ai/adam_assets/share/{maps_2d,maps_3d,camera_calibration}
  touch src/navigation_ai/adam_assets/share/maps_2d/.gitkeep
  touch src/navigation_ai/adam_assets/share/maps_3d/.gitkeep
  touch src/navigation_ai/adam_assets/share/camera_calibration/.gitkeep
  ```

- [ ] **Step 2: 编写 package.xml**

  ```xml
  <?xml version="1.0"?>
  <package format="3">
    <name>adam_assets</name>
    <version>1.0.0</version>
    <description>Centralized asset repository for Robot Adam navigation stack</description>
    <maintainer email="adam@example.com">Adam Team</maintainer>
    <license>MIT</license>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
  ```

- [ ] **Step 3: 编写 CMakeLists.txt**

  ```cmake
  cmake_minimum_required(VERSION 3.8)
  project(adam_assets)
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY share/ DESTINATION share/${PROJECT_NAME})
  ament_package()
  ```

- [ ] **Step 4: 创建工具函数文件**

  Create `src/localization_mapping/adam_localization/scripts/get_asset_path.py`

  ```python
  import os
  from ament_index_python.packages import get_package_share_directory

  def get_asset_path(asset_type, file_name):
      """两级优先查找：运行时缓存区 > 编译时静态区"""
      cache = os.path.expanduser(f'~/.ros/adam_assets/{asset_type}/{file_name}')
      if os.path.exists(cache):
          return cache
      pkg_dir = get_package_share_directory('adam_assets')
      return os.path.join(pkg_dir, 'share', asset_type, file_name)
  ```

  Note: 这个文件属于 `adam_localization` 包，将在 Task 3 中创建该包。这里提前列出以保持规划完整性。

- [ ] **Step 5: 编译验证**

  ```bash
  colcon build --packages-select adam_assets
  source install/setup.bash
  python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('adam_assets'))"
  # 预期输出: .../install/adam_assets/share/adam_assets
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/navigation_ai/adam_assets/
  git commit -m "feat: add adam_assets package with share directory structure

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

### Task 3: adam_localization —— 基础 EKF 里程计底座

**Files:**
- Create: `src/localization_mapping/adam_localization/CMakeLists.txt`
- Create: `src/localization_mapping/adam_localization/package.xml`
- Create: `src/localization_mapping/adam_localization/config/ekf_local.yaml`
- Create: `src/localization_mapping/adam_localization/config/ekf_global.yaml`
- Create: `src/localization_mapping/adam_localization/launch/localization.launch.py`

**Interfaces:**
- Consumes: `/wheel_odom` (nav_msgs/Odometry), `/imu/data` (sensor_msgs/Imu), `/slam_pose/cartographer` (nav_msgs/Odometry)
- Produces: TF `odom→base_link` (local_ekf_node, ≥50Hz), TF `map→odom` (global_ekf_node)

- [ ] **Step 1: 创建包骨架**

  ```bash
  mkdir -p src/localization_mapping/adam_localization/{config,launch}
  ```

  `package.xml`:
  ```xml
  <?xml version="1.0"?>
  <package format="3">
    <name>adam_localization</name>
    <version>1.0.0</version>
    <description>Dual EKF localization base for Robot Adam</description>
    <maintainer email="adam@example.com">Adam Team</maintainer>
    <license>MIT</license>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>robot_localization</depend>
    <depend>nav_msgs</depend>
    <depend>sensor_msgs</depend>
    <depend>tf2_ros</depend>
    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
  ```

  `CMakeLists.txt`:
  ```cmake
  cmake_minimum_required(VERSION 3.8)
  project(adam_localization)
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_package()
  ```

- [ ] **Step 2: 编写 ekf_local.yaml**

  local_ekf_node 配置：融合 `/wheel_odom` (轮速 x, y, angular_z) + `/imu/data` (角速度 z, 加速度 x/y)。

  ```yaml
  ekf_local_node:
    ros__parameters:
      publish_tf: true
      map_frame: map
      odom_frame: odom
      base_link_frame: base_link
      world_frame: odom

      odom0: /wheel_odom
      odom0_config: [true, true, false,    # x, y
                     false, false, false,
                     false, false, false,
                     false, false, true,   # angular_z
                     false, false, false]
      odom0_differential: false
      odom0_queue_size: 10

      imu0: /imu/data
      imu0_config: [false, false, false,
                    false, false, false,
                    false, false, false,
                    false, false, true,    # angular_velocity_z
                    true, true, false,     # linear_acceleration_x, y
                    false]
      imu0_differential: false
      imu0_queue_size: 10
      imu0_remove_gravitational_acceleration: true
  ```

- [ ] **Step 3: 编写 ekf_global.yaml**

  global_ekf_node 配置：接收 Cartographer 位姿作为全局观测源，发布 `map→odom` TF。

  ```yaml
  ekf_global_node:
    ros__parameters:
      publish_tf: true
      map_frame: map
      odom_frame: odom
      base_link_frame: base_link
      world_frame: map

      odom0: /slam_pose/cartographer
      odom0_config: [true, true, false,
                     false, false, true,
                     false, false, false,
                     false, false, false,
                     false, false, false]
      odom0_differential: false
      odom0_queue_size: 10
  ```

- [ ] **Step 4: 编写 localization.launch.py**

  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node

  def generate_launch_description():
      return LaunchDescription([
          Node(
              package='robot_localization',
              executable='ekf_node',
              name='ekf_local_node',
              parameters=[{'use_sim_time': True},
                          'config/ekf_local.yaml'],
              output='screen',
          ),
          Node(
              package='robot_localization',
              executable='ekf_node',
              name='ekf_global_node',
              parameters=[{'use_sim_time': True},
                          'config/ekf_global.yaml'],
              output='screen',
          ),
      ])
  ```

- [ ] **Step 5: 编译验证**

  ```bash
  colcon build --packages-select adam_localization
  source install/setup.bash
  ```

- [ ] **Step 6: 提交**

  ```bash
  git add src/localization_mapping/adam_localization/
  git commit -m "feat: add adam_localization with dual EKF configuration

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

### Task 4: adam_slam —— Cartographer 建图 + custom_explorer 自主探索

**Files:**
- Create: `src/localization_mapping/adam_slam/CMakeLists.txt`
- Create: `src/localization_mapping/adam_slam/package.xml`
- Create: `src/localization_mapping/adam_slam/config/cartographer_2d.lua`
- Create: `src/localization_mapping/adam_slam/config/cartographer_localization.lua`
- Create: `src/localization_mapping/adam_slam/config/custom_explorer.yaml`
- Create: `src/localization_mapping/adam_slam/launch/cartographer_2d.launch.py`
- Create: `src/localization_mapping/adam_slam/launch/cartographer_localization.launch.py`
- Create: `src/localization_mapping/adam_slam/scripts/archive_map.py`

**Interfaces:**
- Consumes: `/scan`, `/imu/data`, TF `odom→base_link`
- Produces: `/slam_pose/cartographer` (nav_msgs/Odometry), `/map` (OccupancyGrid, via occupancy_grid_node)
- Services: `/write_state` (cartographer standard)

- [ ] **Step 1: 创建包骨架**

  ```bash
  mkdir -p src/localization_mapping/adam_slam/{config,launch,scripts}
  ```

  `package.xml`:
  ```xml
  <?xml version="1.0"?>
  <package format="3">
    <name>adam_slam</name>
    <version>1.0.0</version>
    <description>Cartographer 2D SLAM wrapper for Robot Adam</description>
    <maintainer email="adam@example.com">Adam Team</maintainer>
    <license>MIT</license>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>cartographer_ros</depend>
    <depend>nav_msgs</depend>
    <exec_depend>custom_explorer</exec_depend>
    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
  ```

  `CMakeLists.txt`:
  ```cmake
  cmake_minimum_required(VERSION 3.8)
  project(adam_slam)
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS scripts/archive_map.py DESTINATION lib/${PROJECT_NAME})
  ament_package()
  ```

- [ ] **Step 2: 编写 cartographer_2d.lua**

  ```lua
  include "cartographer_ros.lua"

  TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
  TRAJECTORY_BUILDER_2D.min_range = 0.1
  TRAJECTORY_BUILDER_2D.max_range = 8.0
  TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
  TRAJECTORY_BUILDER_2D.use_imu_data = true
  TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
  TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.6
  TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)

  POSE_GRAPH.optimize_every_n_nodes = 90
  POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
  POSE_GRAPH.optimization_problem.huber_scale = 1e2
  POSE_GRAPH.optimization_problem.fix_z_in_3d = false

  return {
      map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER_2D,
      pose_graph = POSE_GRAPH,
      map_frame = "map",
      tracking_frame = "base_link",
      published_frame = "odom",
      odom_frame = "odom",
      provide_odom_frame = false,
      publish_frame_projected_to_2d = false,
      use_odometry = true,
      use_nav_sat = false,
      use_landmarks = false,
      num_laser_scans = 1,
      num_multi_echo_laser_scans = 0,
      num_subdivisions_per_laser_scan = 1,
      num_point_clouds = 0,
      lookup_transform_timeout_sec = 0.2,
      submap_publish_period_sec = 0.3,
      pose_publish_period_sec = 5e-3,
      trajectory_publish_period_sec = 30e-3,
      pose_published_as_odometry = true,
      publish_to_tf = false,
  }
  ```

  **关键约束**：`provide_odom_frame = false`, `published_frame = "odom"`, `publish_to_tf = false`, `pose_published_as_odometry = true`（使 Cartographer 输出 nav_msgs/Odometry 类型的话题而非 PoseStamped）。

- [ ] **Step 3: 编写 cartographer_localization.lua**

  ```lua
  include "cartographer_ros.lua"

  TRAJECTORY_BUILDER_2D.pure_localization = true
  TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
  TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
  TRAJECTORY_BUILDER_2D.min_range = 0.1
  TRAJECTORY_BUILDER_2D.max_range = 8.0
  TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
  TRAJECTORY_BUILDER_2D.use_imu_data = true

  POSE_GRAPH.optimize_every_n_nodes = 3
  POSE_GRAPH.overlapping_submaps_trimmer_2d = nil
  POSE_GRAPH.constraint_builder.sampling_ratio = 0.9
  POSE_GRAPH.constraint_builder.min_score = 0.55
  POSE_GRAPH.constraint_builder.global_localization_min_score = 0.4

  return {
      map_builder = MAP_BUILDER,
      trajectory_builder = TRAJECTORY_BUILDER_2D,
      pose_graph = POSE_GRAPH,
      map_frame = "map",
      tracking_frame = "base_link",
      published_frame = "odom",
      odom_frame = "odom",
      provide_odom_frame = false,
      publish_frame_projected_to_2d = false,
      use_odometry = true,
      use_nav_sat = false,
      use_landmarks = false,
      num_laser_scans = 1,
      num_multi_echo_laser_scans = 0,
      num_subdivisions_per_laser_scan = 1,
      num_point_clouds = 0,
      lookup_transform_timeout_sec = 0.2,
      submap_publish_period_sec = 0.3,
      pose_publish_period_sec = 5e-3,
      trajectory_publish_period_sec = 30e-3,
      pose_published_as_odometry = true,
      publish_to_tf = false,
  }
  ```

- [ ] **Step 4: 编写 custom_explorer 配置**

  `config/custom_explorer.yaml`:
  ```yaml
  custom_explorer:
    ros__parameters:
      exploration_frequency: 1.0
      visualize_frontiers: true
      frontier_queue_size: 10
  ```

- [ ] **Step 5: 编写建图 launch 文件**

  `launch/cartographer_2d.launch.py`:
  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from ament_index_python.packages import get_package_share_directory

  def generate_launch_description():
      config_dir = get_package_share_directory('adam_slam')

      cartographer_node = Node(
          package='cartographer_ros',
          executable='cartographer_node',
          name='cartographer_node',
          parameters=[{'use_sim_time': True}],
          arguments=[
              '-configuration_directory', config_dir + '/config',
              '-configuration_basename', 'cartographer_2d.lua',
          ],
          output='screen',
      )

      occupancy_grid_node = Node(
          package='cartographer_ros',
          executable='cartographer_occupancy_grid_node',
          name='cartographer_occupancy_grid_node',
          parameters=[{'use_sim_time': True}],
          arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
      )

      explorer_node = Node(
          package='custom_explorer',
          executable='explorer',
          name='explorer_node',
          parameters=[config_dir + '/config/custom_explorer.yaml'],
          output='screen',
      )

      return LaunchDescription([
          cartographer_node,
          occupancy_grid_node,
          explorer_node,
      ])
  ```

- [ ] **Step 6: 编写纯定位 launch 文件**

  `launch/cartographer_localization.launch.py`:
  ```python
  import os
  from launch import LaunchDescription
  from launch_ros.actions import Node
  from launch.substitutions import LaunchConfiguration
  from launch.actions import DeclareLaunchArgument
  from ament_index_python.packages import get_package_share_directory

  def generate_launch_description():
      config_dir = get_package_share_directory('adam_slam')

      return LaunchDescription([
          DeclareLaunchArgument('load_state_filename', default_value=''),
          Node(
              package='cartographer_ros',
              executable='cartographer_node',
              name='cartographer_node',
              parameters=[{'use_sim_time': True}],
              arguments=[
                  '-configuration_directory', config_dir + '/config',
                  '-configuration_basename', 'cartographer_localization.lua',
                  '-load_state_filename', LaunchConfiguration('load_state_filename'),
              ],
              output='screen',
          ),
          Node(
              package='cartographer_ros',
              executable='cartographer_occupancy_grid_node',
              name='cartographer_occupancy_grid_node',
              parameters=[{'use_sim_time': True}],
              arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
          ),
      ])
  ```

- [ ] **Step 7: 编写 archive_map.py 归档脚本**

  ```python
  #!/usr/bin/env python3
  """Cartographer map archiving script.

  Usage:
    ros2 run adam_slam archive_map.py                    # archive to runtime cache
    ros2 run adam_slam archive_map.py --commit            # also sync to source tree
  """
  import os
  import sys
  import shutil
  import subprocess
  import time
  from datetime import datetime

  CACHE_DIR = os.path.expanduser('~/.ros/adam_assets/maps_2d')
  CART_SAVE_SERVICE = '/write_state'

  def main():
      timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
      tmp_dir = f'/tmp/adam_maps/{timestamp}'
      os.makedirs(tmp_dir, exist_ok=True)

      pbstream_path = f'{tmp_dir}/map_{timestamp}.pbstream'

      # Call Cartographer /write_state service
      cmd = [
          'ros2', 'service', 'call', CART_SAVE_SERVICE,
          'cartographer_ros_msgs/srv/WriteState',
          f'{{filename: "{pbstream_path}"}}',
      ]
      result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

      if result.returncode != 0:
          print(f'ERROR: Failed to write state: {result.stderr}')
          sys.exit(1)

      # Copy to runtime cache
      os.makedirs(CACHE_DIR, exist_ok=True)
      shutil.copy2(pbstream_path, f'{CACHE_DIR}/map_{timestamp}.pbstream')
      print(f'Archived {pbstream_path} → {CACHE_DIR}/')

      # --commit: also sync to source tree
      if '--commit' in sys.argv:
          src_dir = os.path.expanduser('src/navigation_ai/adam_assets/share/maps_2d')
          if os.path.exists(src_dir):
              shutil.copy2(pbstream_path, f'{src_dir}/map_{timestamp}.pbstream')
              subprocess.run(['colcon', 'build', '--packages-select', 'adam_assets'],
                           timeout=120)
              print(f'Committed to source tree: {src_dir}/')

  if __name__ == '__main__':
      main()
  ```

  记得 `chmod +x scripts/archive_map.py`。

- [ ] **Step 8: 编译验证**

  ```bash
  chmod +x src/localization_mapping/adam_slam/scripts/archive_map.py
  colcon build --packages-select adam_slam
  source install/setup.bash
  ```

- [ ] **Step 9: Commit**

  ```bash
  git add src/localization_mapping/adam_slam/
  git commit -m "feat: add adam_slam with Cartographer 2D mapping + custom_explorer + archive script

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

### Task 5: adam_navigation —— Nav2 Smac + MPPI 运控集成

**Files:**
- Create: `src/navigation_ai/adam_navigation/CMakeLists.txt`
- Create: `src/navigation_ai/adam_navigation/package.xml`
- Create: `src/navigation_ai/adam_navigation/config/nav2_2d_config.yaml`
- Create: `src/navigation_ai/adam_navigation/launch/nav2_bringup.launch.py`

**Interfaces:**
- Consumes: TF `map→odom→base_link`, `/scan` (for costmap), `/map`
- Produces: `/cmd_vel` (via MPPI controller), navigate_to_pose Action

- [ ] **Step 1: 创建包骨架**

  ```bash
  mkdir -p src/navigation_ai/adam_navigation/{config,launch}
  ```

  `package.xml`:
  ```xml
  <?xml version="1.0"?>
  <package format="3">
    <name>adam_navigation</name>
    <version>1.0.0</version>
    <description>Nav2 navigation stack integration for Robot Adam</description>
    <maintainer email="adam@example.com">Adam Team</maintainer>
    <license>MIT</license>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <depend>navigation2</depend>
    <depend>nav2_bringup</depend>
    <depend>nav2_smac_planner</depend>
    <depend>nav2_mppi_controller</depend>
    <export>
      <build_type>ament_cmake</build_type>
    </export>
  </package>
  ```

  `CMakeLists.txt`:
  ```cmake
  cmake_minimum_required(VERSION 3.8)
  project(adam_navigation)
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
  ament_package()
  ```

- [ ] **Step 2: 编写 nav2_2d_config.yaml**

  ```yaml
  planner_server:
    ros__parameters:
      use_sim_time: true
      planner_plugin_types: ["nav2_smac_planner/SmacPlanner2D"]
      SmacPlanner2D:
        downsample_costmap: false
        tolerance: 0.25
        allow_unknown: false
        max_iterations: 1000000
        smooth_path: true

  controller_server:
    ros__parameters:
      use_sim_time: true
      controller_plugin_types: ["nav2_mppi_controller/MppIController"]
      MppIController:
        motion_model: "DiffDrive"
        time_steps: 56
        model_dt: 0.1
        batch_size: 500
        temperature: 0.3
        vx_max: 0.5
        vx_min: -0.2
        vy_max: 0.0
        wz_max: 0.8
        enable_obstacle_collision_check: true

  local_costmap:
    local_costmap:
      ros__parameters:
        use_sim_time: true
        global_frame: odom
        robot_base_frame: base_link
        rolling_window: true
        width: 3
        height: 3
        resolution: 0.05
        plugins: ["obstacle_layer", "inflation_layer"]
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: true
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 0.5
            clearing: true
            marking: true
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55

  global_costmap:
    global_costmap:
      ros__parameters:
        use_sim_time: true
        global_frame: map
        robot_base_frame: base_link
        static_map: true
        plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        static_layer:
          plugin: "nav2_costmap_2d::StaticLayer"
          map_subscribe_transient_local: true
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          enabled: true
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 0.5
            clearing: true
            marking: true
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55

  bt_navigator:
    ros__parameters:
      use_sim_time: true
      default_nav_to_pose_bt_xml: "navigate_to_pose_w_recovery_and_remapping.xml"

  amcl:
    ros__parameters:
      use_sim_time: true
      # Disable AMCL — we use Cartographer for localization in SPEC 02
      # (AMCL remains available as fallback but is not the primary method)

  map_server:
    ros__parameters:
      use_sim_time: true
      # Cartographer's occupancy_grid_node provides /map directly,
      # so map_server is not needed for SPEC 02.
      # Include it for optional .yaml/.pgm loading in standalone mode.
  ```

- [ ] **Step 3: 编写 nav2_bringup.launch.py**

  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from ament_index_python.packages import get_package_share_directory

  def generate_launch_description():
      config_dir = get_package_share_directory('adam_navigation')

      return LaunchDescription([
          # Nav2 标准 bringup
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource([
                  get_package_share_directory('nav2_bringup'),
                  '/launch', '/navigation_launch.py'
              ]),
              launch_arguments={
                  'use_sim_time': 'true',
                  'params_file': config_dir + '/config/nav2_2d_config.yaml',
              }.items()
          ),
      ])
  ```

- [ ] **Step 4: 编译验证**

  ```bash
  colcon build --packages-select adam_navigation
  source install/setup.bash
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/navigation_ai/adam_navigation/
  git commit -m "feat: add adam_navigation with Nav2 Smac + MPPI configuration

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

### Task 6: 全栈一键启动 launch + 集成测试

**Files:**
- Create: `src/navigation_ai/adam_navigation/launch/spec_02_all.launch.py`

**Interfaces:**
- Launches: Gazebo (laser_2wd), robot_state_publisher, local_ekf_node, global_ekf_node, Cartographer (建图模式), Nav2, test_tools

- [ ] **Step 1: 编写 spec_02_all.launch.py**

  ```python
  """一键启动 SPEC 02 全栈：Gazebo + EKF + Cartographer + Nav2 + test_tools。

  启动模式:
    mode=mapping    — 建图模式 (默认): Cartographer 建图 + custom_explorer
    mode=localize   — 纯定位模式: Cartographer 加载 pbstream 定位 + Nav2
  """
  from launch import LaunchDescription
  from launch.substitutions import LaunchConfiguration
  from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from launch.conditions import IfCondition
  from ament_index_python.packages import get_package_share_directory

  def generate_launch_description():
      robot_desc_dir = get_package_share_directory('robot_description')
      loc_dir = get_package_share_directory('adam_localization')
      slam_dir = get_package_share_directory('adam_slam')
      nav_dir = get_package_share_directory('adam_navigation')
      tools_dir = get_package_share_directory('adam_test_tools')

      return LaunchDescription([
          DeclareLaunchArgument('mode', default_value='mapping',
                                description='mapping | localize'),
          DeclareLaunchArgument('load_state_filename', default_value='',
                                description='pbstream path (localize mode only)'),
          DeclareLaunchArgument('use_sim_time', default_value='true'),

          # === L1: Gazebo + robot ===
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                  robot_desc_dir + '/launch/laser_2wd.launch.py'
              ),
              launch_arguments={'use_sim_time': 'true'}.items(),
          ),

          # === L2: EKF 里程计 ===
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                  loc_dir + '/launch/localization.launch.py'
              ),
          ),

          # === L3: Cartographer (建图模式) ===
          GroupAction(
              condition=IfCondition(
                  LaunchConfiguration('mode') == 'mapping'
              ),
              actions=[
                  IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(
                          slam_dir + '/launch/cartographer_2d.launch.py'
                      ),
                  ),
              ],
          ),

          # === L3: Cartographer (纯定位模式) ===
          GroupAction(
              condition=IfCondition(
                  LaunchConfiguration('mode') == 'localize'
              ),
              actions=[
                  IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(
                          slam_dir + '/launch/cartographer_localization.launch.py'
                      ),
                      launch_arguments={
                          'load_state_filename': LaunchConfiguration('load_state_filename'),
                      }.items(),
                  ),
                  IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(
                          nav_dir + '/launch/nav2_bringup.launch.py'
                      ),
                  ),
              ],
          ),

          # === Test tools ===
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                  tools_dir + '/launch/test_tools.launch.py'
              ),
          ),
      ])
  ```

- [ ] **Step 2: 编译验证**

  ```bash
  colcon build --packages-select adam_navigation
  source install/setup.bash
  # 语法验证 (不实际启动仿真)
  ros2 launch src/navigation_ai/adam_navigation/launch/spec_02_all.launch.py --show-arguments
  ```

- [ ] **Step 3: Commit**

  ```bash
  git add src/navigation_ai/adam_navigation/launch/spec_02_all.launch.py
  git commit -m "feat: add spec_02_all.launch.py for one-click full stack bringup

  Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
  ```

---

## 验证方案

每个 Task 完成后测试其独立交付物。全栈集成验证在 Task 6 完成后按 SPEC 02 §3 Gate Criteria 逐条执行：

1. **Gate 1**: `python3 -c "from ament_index_python import get_package_share_directory; print(get_package_share_directory('adam_assets'))"` — 无硬编码路径
2. **Gate 2**: `ros2 run adam_test_tools tf_monitor_node.py` → `/test_verdict/tf_health` 输出 PASS
3. **Gate 3**: 启动 `mode=mapping`，建图 100m² 15min 内完成，子图锋利
4. **Gate 4**: `ros2 service call /write_state` → `.pbstream` 落盘到 `~/.ros/adam_assets/`
5. **Gate 5**: `ros2 service call /test_tools/teleport` 劫持 → Cartographer 1s 内重定位
6. **Gate 6**: `ros2 service call /test_tools/spawn_object "spawn box 1.5 0.5 0.0"` → Nav2 绕行
