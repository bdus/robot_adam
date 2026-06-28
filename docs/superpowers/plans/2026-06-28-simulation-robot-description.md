# Simulation Robot Description 包实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建 `src/simulation/robot_description/` ROS2 包，实现 2WD/4WD/Omni × Mid360/Laser = 6 种运控方式小车仿真

**Architecture:** 新 ROS2 包，通过 `$(find adam_description)` 引用现有传感器宏，自身提供公共底盘/轮子/控制器宏，组装为 6 个 URDF 变体，每个变体对应独立 launch 文件

**Tech Stack:** ROS2 Humble, Gazebo Classic 11, gazebo_ros_pkgs, Livox SDK2, xacro

## Global Constraints

- `use_sim_time:=true` 所有节点强制设置
- 控制器 `<robot_base_frame>` 统一使用 `base_link`，禁止使用 `base_footprint`
- Omni 里程计话题重命名为 `wheel_odom`
- 2WD 前轮万向轮：球体方案（sphere + fixed joint + mu=0.01）
- Omni 轮子：去除 collision 标签（纯视觉）
- 每次测试前必须清理残留仿真进程（ros-simulation-clean skill）
- 修改代码后必须 `colcon build --packages-select robot_description` + `source install/setup.bash`

---
### Task 1: 创建包目录结构和构建文件

**Files:**
- Create: `src/simulation/robot_description/package.xml`
- Create: `src/simulation/robot_description/CMakeLists.txt`
- Create: `src/simulation/robot_description/urdf/common/.gitkeep`
- Create: `src/simulation/robot_description/urdf/controllers/.gitkeep`
- Create: `src/simulation/robot_description/urdf/plugins/.gitkeep`
- Create: `src/simulation/robot_description/urdf/robots/.gitkeep`
- Create: `src/simulation/robot_description/launch/.gitkeep`
- Create: `src/simulation/robot_description/config/rviz/.gitkeep`
- Create: `src/simulation/robot_description/worlds/.gitkeep`

- [ ] **Step 1: 创建目录结构**

```bash
mkdir -p /home/pi/workplace/robot_adam/src/simulation/robot_description/{urdf/{common,controllers,plugins,robots},launch,config/rviz,worlds}
```

- [ ] **Step 2: 创建 package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.0</version>
  <description>Simulation robot description package with 6 drive variants (2WD/4WD/Omni x Mid360/Laser)</description>
  <maintainer email="adam@example.com">Adam Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>
  <depend>gazebo_ros</depend>
  <depend>gazebo_plugins</depend>
  <depend>adam_description</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 3: 创建 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install rviz config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()
```

- [ ] **Step 4: 创建 .gitkeep 占位文件**

```bash
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/common/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/controllers/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/plugins/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/launch/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/config/rviz/.gitkeep
touch /home/pi/workplace/robot_adam/src/simulation/robot_description/worlds/.gitkeep
```

- [ ] **Step 5: 编译验证包结构**

```bash
cd /home/pi/workplace/robot_adam
./build_sim.sh
source install/setup.bash
ros2 pkg list | grep robot_description
```

Expected: `robot_description` appears in package list

- [ ] **Step 6: Commit**

```bash
git add src/simulation/
git commit -m "feat: create robot_description package structure"
```

---
### Task 2: 创建公共组件 — base.xacro、wheel.xacro、caster_wheel.xacro

**Files:**
- Create: `src/simulation/robot_description/urdf/common/base.xacro`
- Create: `src/simulation/robot_description/urdf/common/wheel.xacro`
- Create: `src/simulation/robot_description/urdf/common/caster_wheel.xacro`

**Interfaces:**
- Consumes: `$(find adam_description)/urdf/common/common_inertia.xacro`（引用惯性矩阵宏）
- Produces: `base`, `wheel`, `caster_wheel` 宏，被 robots/*.urdf.xacro 引用

- [x] **Step 1: 创建 base.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find adam_description)/urdf/common/common_inertia.xacro"/>

    <!-- 底盘基座（参数化：尺寸、质量、COM高度、摩擦系数） -->
    <xacro:macro name="base" params="
        length:=0.24 width:=0.18 height:=0.08 mass:=1.5
        com_z:=0.03 mu:=0.8
        color_r:=0.2 color_g:=0.4 color_b:=0.8">

        <!-- base_footprint 虚 link -->
        <link name="base_footprint"/>

        <!-- base_link 主底盘 -->
        <joint name="base_joint" type="fixed">
            <parent link="base_footprint"/>
            <child link="base_link"/>
            <origin xyz="0 0 ${com_z}" rpy="0 0 0"/>
        </joint>

        <link name="base_link">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <box size="${length} ${width} ${height}"/>
                </geometry>
                <material name="base_material">
                    <color rgba="${color_r} ${color_g} ${color_b} 1.0"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <box size="${length} ${width} ${height}"/>
                </geometry>
            </collision>
            <xacro:box_inertia m="${mass}" w="${length}" h="${height}" d="${width}"/>
        </link>

        <gazebo reference="base_link">
            <material>Gazebo/Blue</material>
            <mu1>${mu}</mu1>
            <mu2>${mu}</mu2>
        </gazebo>

        <!-- 前向箭头（视觉指示） -->
        <link name="forward_arrow">
            <visual>
                <geometry>
                    <box size="0.08 0.01 0.01"/>
                </geometry>
                <material name="arrow_material">
                    <color rgba="1.0 0.0 0.0 1.0"/>
                </material>
                <origin xyz="${length/2 + 0.04} 0 ${height/2 + 0.02}" rpy="0 0 0"/>
            </visual>
        </link>
        <joint name="forward_arrow_joint" type="fixed">
            <parent link="base_link"/>
            <child link="forward_arrow"/>
            <origin xyz="0 0 0"/>
        </joint>

    </xacro:macro>

</robot>
```

- [x] **Step 2: 创建 wheel.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find adam_description)/urdf/common/common_inertia.xacro"/>

    <!-- 统一轮子宏（参数化：位置、joint_type、摩擦系数、质量） -->
    <xacro:macro name="wheel" params="
        side pos_x pos_y pos_z
        joint_type:='fixed' mu:=1.0
        radius:=0.04 width:=0.032 mass:=0.1
        has_collision:=true">

        <xacro:property name="wheel_name" value="${side}_wheel" />

        <link name="${wheel_name}_link">
            <visual>
                <origin xyz="0 0 0" rpy="1.57079 0 0"/>
                <geometry>
                    <cylinder length="${width}" radius="${radius}"/>
                </geometry>
                <material name="wheel_material">
                    <color rgba="0.1 0.1 0.1 1.0"/>
                </material>
            </visual>

            <xacro:if value="${has_collision}">
            <collision>
                <origin xyz="0 0 0" rpy="1.57079 0 0"/>
                <geometry>
                    <cylinder length="${width}" radius="${radius}"/>
                </geometry>
            </collision>
            </xacro:if>

            <xacro:cylinder_inertia m="${mass}" r="${radius}" h="${width}"/>
        </link>

        <joint name="${wheel_name}_joint" type="${joint_type}">
            <parent link="base_link"/>
            <child link="${wheel_name}_link"/>
            <origin xyz="${pos_x} ${pos_y} ${pos_z}" rpy="0 0 0"/>
            <xacro:if value="${joint_type == 'continuous'}">
                <axis xyz="0 1 0"/>
            </xacro:if>
        </joint>

        <gazebo reference="${wheel_name}_link">
            <material>Gazebo/Black</material>
            <mu1>${mu}</mu1>
            <mu2>${mu}</mu2>
        </gazebo>

    </xacro:macro>

</robot>
```

- [x] **Step 3: 创建 caster_wheel.xacro（2WD 万向轮，linorobot2 风格球体方案）**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="$(find adam_description)/urdf/common/common_inertia.xacro"/>

    <!-- 2WD 前轮万向轮（球体方案，mu=0.01 极低摩擦） -->
    <xacro:macro name="caster_wheel" params="
        side pos_x pos_y pos_z
        sphere_radius:=0.005 mass:=0.01 mu:=0.01">

        <xacro:property name="caster_name" value="${side}_caster_wheel" />

        <link name="${caster_name}_link">
            <visual>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <sphere radius="${sphere_radius}"/>
                </geometry>
                <material name="caster_material">
                    <color rgba="0.3 0.3 0.3 1.0"/>
                </material>
            </visual>
            <collision>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <geometry>
                    <sphere radius="${sphere_radius}"/>
                </geometry>
            </collision>
            <xacro:sphere_inertia m="${mass}" r="${sphere_radius}"/>
        </link>

        <joint name="${caster_name}_joint" type="fixed">
            <parent link="base_link"/>
            <child link="${caster_name}_link"/>
            <origin xyz="${pos_x} ${pos_y} ${pos_z}" rpy="0 0 0"/>
        </joint>

        <gazebo reference="${caster_name}_link">
            <material>Gazebo/Black</material>
            <mu1>${mu}</mu1>
            <mu2>${mu}</mu2>
        </gazebo>

    </xacro:macro>

</robot>
```

- [x] **Step 4: Commit**

```bash
git add src/simulation/robot_description/urdf/common/
git commit -m "feat: add base, wheel, caster_wheel common macros"
```

---
### Task 3: 创建控制器和插件文件

**Files:**
- Create: `src/simulation/robot_description/urdf/controllers/diff_drive_2wd.xacro`
- Create: `src/simulation/robot_description/urdf/controllers/diff_drive_4wd.xacro`
- Create: `src/simulation/robot_description/urdf/controllers/omni_drive.xacro`
- Create: `src/simulation/robot_description/urdf/plugins/gazebo_sensors.xacro`

**Interfaces:**
- Produces: `diff_drive_2wd`, `diff_drive_4wd`, `omni_drive`, `gazebo_sensors` 宏

- [ ] **Step 1: 创建 diff_drive_2wd.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- 2WD 差速控制器（后两轮驱动） -->
    <xacro:macro name="diff_drive_2wd">

        <gazebo>
            <plugin name="diff_drive_2wd" filename="libgazebo_ros_diff_drive.so">

                <ros>
                    <namespace>/</namespace>
                    <remapping>cmd_vel:=/cmd_vel</remapping>
                    <remapping>odom:=/odom</remapping>
                </ros>

                <update_rate>100</update_rate>
                <publish_rate>10</publish_rate>

                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>
                <publish_wheel_tf>false</publish_wheel_tf>
                <publishWheelJointState>false</publishWheelJointState>

                <covariance_x>0.0001</covariance_x>
                <covariance_y>0.0001</covariance_y>
                <covariance_yaw>0.01</covariance_yaw>

                <!-- 后轮驱动 -->
                <left_joint>rear_left_wheel_joint</left_joint>
                <right_joint>rear_right_wheel_joint</right_joint>

                <wheel_separation>0.17</wheel_separation>
                <wheel_diameter>0.08</wheel_diameter>

                <max_wheel_torque>100</max_wheel_torque>
                <max_wheel_acceleration>1.0</max_wheel_acceleration>

                <command_topic>cmd_vel</command_topic>
                <odometry_source>1</odometry_source>

            </plugin>
        </gazebo>

    </xacro:macro>

</robot>
```

- [ ] **Step 2: 创建 diff_drive_4wd.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- 4WD 差速控制器（四轮驱动，num_wheel_pairs=2） -->
    <xacro:macro name="diff_drive_4wd">

        <gazebo>
            <plugin name="diff_drive_4wd" filename="libgazebo_ros_diff_drive.so">

                <ros>
                    <namespace>/</namespace>
                    <remapping>cmd_vel:=/cmd_vel</remapping>
                    <remapping>odom:=/odom</remapping>
                </ros>

                <update_rate>100</update_rate>
                <publish_rate>10</publish_rate>

                <num_wheel_pairs>2</num_wheel_pairs>

                <!-- 第一对：前轮 -->
                <left_joint>front_left_wheel_joint</left_joint>
                <right_joint>front_right_wheel_joint</right_joint>
                <wheel_separation>0.17</wheel_separation>
                <wheel_diameter>0.08</wheel_diameter>

                <!-- 第二对：后轮 -->
                <left_joint>rear_left_wheel_joint</left_joint>
                <right_joint>rear_right_wheel_joint</right_joint>
                <wheel_separation>0.17</wheel_separation>
                <wheel_diameter>0.08</wheel_diameter>

                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>
                <publish_wheel_tf>false</publish_wheel_tf>
                <publishWheelJointState>false</publishWheelJointState>

                <covariance_x>0.0001</covariance_x>
                <covariance_y>0.0001</covariance_y>
                <covariance_yaw>0.01</covariance_yaw>

                <max_wheel_torque>100</max_wheel_torque>
                <max_wheel_acceleration>1.0</max_wheel_acceleration>

                <command_topic>cmd_vel</command_topic>
                <odometry_source>1</odometry_source>

            </plugin>
        </gazebo>

    </xacro:macro>

</robot>
```

- [ ] **Step 3: 创建 omni_drive.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Omni 全向移动控制器（planar_move 插件） -->
    <!-- 里程计发布到 wheel_odom，避免与 diff_drive 的 /odom 冲突 -->
    <xacro:macro name="omni_drive">

        <gazebo>
            <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">

                <ros>
                    <namespace>/</namespace>
                    <remapping>cmd_vel:=/cmd_vel</remapping>
                    <remapping>odom:=/wheel_odom</remapping>
                </ros>

                <update_rate>100</update_rate>

                <odometry_frame>odom</odometry_frame>
                <robot_base_frame>base_link</robot_base_frame>
                <publish_odom>true</publish_odom>
                <publish_odom_tf>true</publish_odom_tf>

                <max_vel_x>0.8</max_vel_x>
                <max_vel_y>0.8</max_vel_y>
                <max_vel_theta>0.6</max_vel_theta>

                <max_accel_x>0.4</max_accel_x>
                <max_accel_y>0.4</max_accel_y>
                <max_accel_theta>0.3</max_accel_theta>

            </plugin>
        </gazebo>

    </xacro:macro>

</robot>
```

- [ ] **Step 4: 创建 plugins/gazebo_sensors.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Gazebo 传感器插件（空壳，传感器在各传感器 xacro 文件中定义） -->
    <xacro:macro name="gazebo_sensors">
    </xacro:macro>

</robot>
```

- [ ] **Step 5: Commit**

```bash
git add src/simulation/robot_description/urdf/controllers/ src/simulation/robot_description/urdf/plugins/
git commit -m "feat: add diff_drive_2wd/4wd and omni_drive controllers"
```

---
### Task 4: 创建 Mid360 3D 激光版本 URDF（mid360_2wd, mid360_4wd, mid360_omni）

**Files:**
- Create: `src/simulation/robot_description/urdf/robots/mid360_2wd.urdf.xacro`
- Create: `src/simulation/robot_description/urdf/robots/mid360_4wd.urdf.xacro`
- Create: `src/simulation/robot_description/urdf/robots/mid360_omni.urdf.xacro`

**Interfaces:**
- Consumes: `base`, `wheel`, `caster_wheel` 宏（Task 2），控制器宏（Task 3），`$(find adam_description)/urdf/sensors/mid360.xacro`, `imu.xacro`, `mono_camera.xacro`
- Produces: 可被 robot_state_publisher 加载的完整 URDF

- [ ] **Step 1: 创建 mid360_2wd.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mid360_2wd">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/caster_wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/diff_drive_2wd.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/mid360.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 后轮（驱动轮，continuous joint, mu=1.0） -->
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>

    <!-- 前轮（万向从动轮，球体方案，mu=0.01） -->
    <xacro:caster_wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05"/>
    <xacro:caster_wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05"/>

    <!-- 传感器 -->
    <xacro:mid360 name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:mid360>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:diff_drive_2wd/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 2: 创建 mid360_4wd.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mid360_4wd">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/diff_drive_4wd.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/mid360.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 四轮全部驱动（continuous joint, mu=1.0） -->
    <xacro:wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>

    <!-- 传感器 -->
    <xacro:mid360 name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:mid360>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:diff_drive_4wd/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 3: 创建 mid360_omni.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mid360_omni">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/omni_drive.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/mid360.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 四轮纯视觉（fixed joint, 无 collision） -->
    <xacro:wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="fixed" has_collision="false"/>

    <!-- 传感器 -->
    <xacro:mid360 name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:mid360>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:omni_drive/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 4: Commit**

```bash
git add src/simulation/robot_description/urdf/robots/mid360_*.urdf.xacro
git commit -m "feat: add mid360_2wd, mid360_4wd, mid360_omni URDFs"
```

---
### Task 5: 创建 2D 激光版本 URDF（laser_2wd, laser_4wd, laser_omni）

**Files:**
- Create: `src/simulation/robot_description/urdf/robots/laser_2wd.urdf.xacro`
- Create: `src/simulation/robot_description/urdf/robots/laser_4wd.urdf.xacro`
- Create: `src/simulation/robot_description/urdf/robots/laser_omni.urdf.xacro`

- [ ] **Step 1: 创建 laser_2wd.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="laser_2wd">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/caster_wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/diff_drive_2wd.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/laser_2d.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 后轮（驱动轮，continuous joint, mu=1.0） -->
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>

    <!-- 前轮（万向从动轮，球体方案，mu=0.01） -->
    <xacro:caster_wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05"/>
    <xacro:caster_wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05"/>

    <!-- 传感器 -->
    <xacro:laser_2d name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:laser_2d>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:diff_drive_2wd/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 2: 创建 laser_4wd.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="laser_4wd">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/diff_drive_4wd.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/laser_2d.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 四轮全部驱动（continuous joint, mu=1.0） -->
    <xacro:wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="continuous" mu="1.0"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="continuous" mu="1.0"/>

    <!-- 传感器 -->
    <xacro:laser_2d name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:laser_2d>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:diff_drive_4wd/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 3: 创建 laser_omni.urdf.xacro**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="laser_omni">

    <xacro:include filename="$(find robot_description)/urdf/common/base.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/common/wheel.xacro"/>

    <xacro:include filename="$(find robot_description)/urdf/controllers/omni_drive.xacro"/>
    <xacro:include filename="$(find robot_description)/urdf/plugins/gazebo_sensors.xacro"/>

    <xacro:include filename="$(find adam_description)/urdf/sensors/laser_2d.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/imu.xacro"/>
    <xacro:include filename="$(find adam_description)/urdf/sensors/mono_camera.xacro"/>

    <!-- 底盘 -->
    <xacro:base com_z="0.03"/>

    <!-- 四轮纯视觉（fixed joint, 无 collision） -->
    <xacro:wheel side="front_left"  pos_x="0.08" pos_y="0.085"  pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="front_right" pos_x="0.08" pos_y="-0.085" pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="rear_left"   pos_x="-0.08" pos_y="0.085"  pos_z="-0.05" joint_type="fixed" has_collision="false"/>
    <xacro:wheel side="rear_right"  pos_x="-0.08" pos_y="-0.085" pos_z="-0.05" joint_type="fixed" has_collision="false"/>

    <!-- 传感器 -->
    <xacro:laser_2d name="laser" parent="base_link">
        <origin xyz="0.0 0.0 0.06" rpy="0 0 0"/>
    </xacro:laser_2d>
    <xacro:imu name="imu" parent="base_link">
        <origin xyz="0.0 0.0 0.02" rpy="0 0 0"/>
    </xacro:imu>
    <xacro:mono_camera name="camera" parent="base_link">
        <origin xyz="0.12 0.0 0.03" rpy="0 0.2 0"/>
    </xacro:mono_camera>

    <!-- 应用插件 -->
    <xacro:omni_drive/>
    <xacro:gazebo_sensors/>

</robot>
```

- [ ] **Step 4: Commit**

```bash
git add src/simulation/robot_description/urdf/robots/laser_*.urdf.xacro
git commit -m "feat: add laser_2wd, laser_4wd, laser_omni URDFs"
```

---
### Task 6: 创建 Launch 文件

**Files:**
- Create: `src/simulation/robot_description/launch/mid360_2wd.launch.py`
- Create: `src/simulation/robot_description/launch/mid360_4wd.launch.py`
- Create: `src/simulation/robot_description/launch/mid360_omni.launch.py`
- Create: `src/simulation/robot_description/launch/laser_2wd.launch.py`
- Create: `src/simulation/robot_description/launch/laser_4wd.launch.py`
- Create: `src/simulation/robot_description/launch/laser_omni.launch.py`

**Note:** 由于 6 个 launch 文件只有模型名称、URDF 路径、rviz 配置三个差异，下面只写一个完整示例，其他 5 个仅列出差异参数。

- [x] **Step 1: 创建 mid360_2wd.launch.py**

```python
import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default='false')

    robot_name = "mid360_2wd"
    urdf_path = get_package_share_directory('robot_description')
    default_model_path = urdf_path + '/urdf/robots/mid360_2wd.urdf.xacro'
    default_world_path = urdf_path + '/worlds/bigH.world'
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    action_declare_arg_model = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='URDF 的绝对路径')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py'
        ]),
        launch_arguments=[
            ('world', default_world_path),
            ('verbose', 'true'),
            ('use_sim_time', use_sim_time)
        ]
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name, ])

    delayed_spawn = launch.actions.TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'mid360_2wd.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_path],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('rviz', default_value='false',
                                             description='Open RViz'),
        action_declare_arg_model,
        robot_state_publisher_node,
        joint_state_publisher_node,
        launch_gazebo,
        delayed_spawn,
        rviz
    ])
```

- [x] **Step 2: 创建 mid360_4wd.launch.py**

与 mid360_2wd.launch.py 仅 3 处差异：
- `robot_name = "mid360_4wd"`
- `default_model_path = urdf_path + '/urdf/robots/mid360_4wd.urdf.xacro'`
- `rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'mid360_4wd.rviz')`

复制 mid360_2wd.launch.py 并替换这 3 行。

- [x] **Step 3: 创建 mid360_omni.launch.py**

与 mid360_2wd.launch.py 仅 3 处差异：
- `robot_name = "mid360_omni"`
- `default_model_path = urdf_path + '/urdf/robots/mid360_omni.urdf.xacro'`
- `rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'mid360_omni.rviz')`

复制 mid360_2wd.launch.py 并替换这 3 行。

- [x] **Step 4: 创建 laser_2wd.launch.py**

与 mid360_2wd.launch.py 仅 3 处差异：
- `robot_name = "laser_2wd"`
- `default_model_path = urdf_path + '/urdf/robots/laser_2wd.urdf.xacro'`
- `rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'laser_2wd.rviz')`

- [x] **Step 5: 创建 laser_4wd.launch.py**

与 mid360_2wd.launch.py 仅 3 处差异：
- `robot_name = "laser_4wd"`
- `default_model_path = urdf_path + '/urdf/robots/laser_4wd.urdf.xacro'`
- `rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'laser_4wd.rviz')`

- [x] **Step 6: 创建 laser_omni.launch.py**

与 mid360_2wd.launch.py 仅 3 处差异：
- `robot_name = "laser_omni"`
- `default_model_path = urdf_path + '/urdf/robots/laser_omni.urdf.xacro'`
- `rviz_path = os.path.join(urdf_path, 'config', 'rviz', 'laser_omni.rviz')`

- [ ] **Step 7: Commit**

```bash
git add src/simulation/robot_description/launch/
git commit -m "feat: add 6 launch files for all drive variants"
```

---
### Task 7: 创建 RViz 配置文件和 World 文件

**Files:**
- Create: `src/simulation/robot_description/config/rviz/mid360_2wd.rviz`
- Create: `src/simulation/robot_description/config/rviz/mid360_4wd.rviz`
- Create: `src/simulation/robot_description/config/rviz/mid360_omni.rviz`
- Create: `src/simulation/robot_description/config/rviz/laser_2wd.rviz`
- Create: `src/simulation/robot_description/config/rviz/laser_4wd.rviz`
- Create: `src/simulation/robot_description/config/rviz/laser_omni.rviz`
- Create: `src/simulation/robot_description/worlds/empty.world`
- Create: `src/simulation/robot_description/worlds/bigH.world`

**Note:** 6 个 rviz 配置分为两类（Mid360 和 Laser），同类之间仅标题不同。下面只写两类各一个完整示例。

- [ ] **Step 1: 读取源 rviz 配置**

```bash
cat /home/pi/workplace/robot_adam/src/adam_description/config/rviz/ackermann_mid360.rviz
cat /home/pi/workplace/robot_adam/src/adam_description/config/rviz/ackermann_laser.rviz
```

- [x] **Step 2: 创建 mid360 系列 rviz 配置**

**Note:** 未执行 — 需要读取源 rviz 配置后手动生成。当前测试使用 rviz:=false 默认，不影响仿真测试。**后续需补充完成。**

创建 `config/rviz/mid360_2wd.rviz`（内容复制自 ackermann_mid360.rviz，修改 Title 和 Fixed Frame）。
	
创建 `config/rviz/mid360_4wd.rviz`（同上，Title 改为 "mid360_4wd"）。
	
创建 `config/rviz/mid360_omni.rviz`（同上，Title 改为 "mid360_omni"）。

基于 ackermann_mid360.rviz，将 Fixed Frame 从 `base_footprint` 改为 `odom`，并将 Title 改为对应变体名。

创建 `config/rviz/mid360_2wd.rviz`（内容复制自 ackermann_mid360.rviz，修改 Title 和 Fixed Frame）。

创建 `config/rviz/mid360_4wd.rviz`（同上，Title 改为 "mid360_4wd"）。

创建 `config/rviz/mid360_omni.rviz`（同上，Title 改为 "mid360_omni"）。

- [x] **Step 3: 创建 laser 系列 rviz 配置**

**Note:** 未执行 — 需要读取源 rviz 配置后手动生成。当前测试使用 rviz:=false 默认，不影响仿真测试。**后续需补充完成。**

基于 ackermann_laser.rviz，将 Fixed Frame 从 `base_footprint` 改为 `odom`，并将 Title 改为对应变体名。
	
创建 `config/rviz/laser_2wd.rviz`、`laser_4wd.rviz`、`laser_omni.rviz`。

基于 ackermann_laser.rviz，将 Fixed Frame 从 `base_footprint` 改为 `odom`，并将 Title 改为对应变体名。

创建 `config/rviz/laser_2wd.rviz`、`laser_4wd.rviz`、`laser_omni.rviz`。

- [x] **Step 5: 复制 world 文件**

```bash
cp /home/pi/workplace/robot_adam/src/adam_description/world/empty.world \
   /home/pi/workplace/robot_adam/src/simulation/robot_description/worlds/empty.world
cp /home/pi/workplace/robot_adam/src/adam_description/world/bigH.world \
   /home/pi/workplace/robot_adam/src/simulation/robot_description/worlds/bigH.world
```

- [ ] **Step 6: Commit**

```bash
git add src/simulation/robot_description/config/rviz/ src/simulation/robot_description/worlds/
git commit -m "feat: add rviz configs and world files"
```

---
### Task 8: 编译验证

- [x] **Step 1: 编译整个工作空间**

```bash
cd /home/pi/workplace/robot_adam
./build_sim.sh
source install/setup.bash
```

Expected: 编译成功，无错误

- [x] **Step 2: 验证包和 URDF 可加载**

```bash
# 验证包存在
ros2 pkg list | grep robot_description

# 验证 URDF 可通过 xacro 展开（无错误）
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/mid360_2wd.urdf.xacro > /dev/null && echo "mid360_2wd: OK"
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/mid360_4wd.urdf.xacro > /dev/null && echo "mid360_4wd: OK"
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/mid360_omni.urdf.xacro > /dev/null && echo "mid360_omni: OK"
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/laser_2wd.urdf.xacro > /dev/null && echo "laser_2wd: OK"
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/laser_4wd.urdf.xacro > /dev/null && echo "laser_4wd: OK"
xacro /home/pi/workplace/robot_adam/src/simulation/robot_description/urdf/robots/laser_omni.urdf.xacro > /dev/null && echo "laser_omni: OK"
```

Expected: 所有 6 个 URDF 展开成功

- [x] **Step 3: Commit**

```bash
git commit --allow-empty -m "chore: build verification passed"
```

---
### Task 9: 仿真测试 — mid360_2wd

- [x] **Step 1: 清理残留进程**

```bash
source /opt/ros/humble/setup.bash && ros2 node list
# 如有残留，运行:
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

Expected: "PASS: No residual simulation nodes detected"

- [x] **Step 2: 启动仿真**

```bash
cd /home/pi/workplace/robot_adam
source install/setup.bash
ros2 launch robot_description mid360_2wd.launch.py &
sleep 15
```

- [x] **Step 3: 检查传感器话题**

```bash
ros2 topic list
ros2 topic echo /livox/lidar --once --timeout 2
ros2 topic echo /imu/data --once --timeout 2
ros2 topic echo /camera_sensor/image_raw --once --timeout 2
```

- [x] **Step 4: 直走测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: position.x 增大, position.z ≈ 0.03~0.06, orientation x/y ≈ 0

- [x] **Step 5: 转向测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: orientation.z 有变化（偏航角）

- [x] **Step 6: 原地旋转测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"
sleep 1
ros2 topic echo /odom --once
```

- [x] **Step 7: 清理**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

---
### Task 10: 仿真测试 — mid360_4wd

- [ ] **Step 1: 清理 + 启动**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
source /home/pi/workplace/robot_adam/install/setup.bash
ros2 launch robot_description mid360_4wd.launch.py &
sleep 15
```

- [ ] **Step 2: 直走测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: position.x 增大, 运动应比 2WD 更稳定

- [ ] **Step 3: 转向测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: orientation.z 变化，4WD 转向应比 2WD 更灵敏

- [ ] **Step 4: 清理**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

---
### Task 11: 仿真测试 — mid360_omni

- [ ] **Step 1: 清理 + 启动**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
source /home/pi/workplace/robot_adam/install/setup.bash
ros2 launch robot_description mid360_omni.launch.py &
sleep 15
```

- [ ] **Step 2: 直走测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once
```

- [ ] **Step 3: 纯横移测试（Omni 独有）**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: position.y 线性增大, position.x 波动 < ±0.02m, orientation.z 波动 < ±0.01rad

- [ ] **Step 4: 45度斜向漂移测试**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once
```

Check: ΔX / ΔY 比值在 0.95 ~ 1.05 之间

- [ ] **Step 5: 清理**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

---
### Task 12: 仿真测试 — laser_2wd / laser_4wd / laser_omni

重复 Task 9-11 的流程，但用 laser 版本的 launch 文件。

- [ ] **Step 1: 测试 laser_2wd**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
source /home/pi/workplace/robot_adam/install/setup.bash
ros2 launch robot_description laser_2wd.launch.py &
sleep 15

# 直走
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once

# 转向
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"
sleep 1
ros2 topic echo /odom --once

# 2D 激光雷达
ros2 topic echo /scan --once --timeout 2

# 清理
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

- [ ] **Step 2: 测试 laser_4wd**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
source /home/pi/workplace/robot_adam/install/setup.bash
ros2 launch robot_description laser_4wd.launch.py &
sleep 15

# 直走
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once

# 转向
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: -1.0}}"
sleep 1
ros2 topic echo /odom --once

# 清理
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

- [ ] **Step 3: 测试 laser_omni**

```bash
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
source /home/pi/workplace/robot_adam/install/setup.bash
ros2 launch robot_description laser_omni.launch.py &
sleep 15

# 直走
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once

# 横移
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5}, angular: {z: 0.0}}"
sleep 1
ros2 topic echo /odom --once

# 清理
bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh
```

- [ ] **Step 4: 最终 Commit**

```bash
git add -A
git commit -m "chore: all 6 drive variants tested and verified"
```
