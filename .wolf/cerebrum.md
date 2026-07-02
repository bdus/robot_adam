## Skill: ros-simulation-clean
- **Purpose**: Clean up Gazebo/ROS2 simulation residual processes
- **Files created**:
  - .claude/skills/ros-simulation-clean/SKILL.md (skill definition)
  - .claude/skills/ros-simulation-clean/clean.sh (executable Bash script)
- **Logic**: Pure Bash control loop with max 5 retries, kills gzserver/gzclient/ruby/*ign/*gz/robot_state_publisher (sim-related)/ros2_control_node/component_container/ros2 launch processes, verifies with ros2 node list and port 11345 check
- **Usage**: Invoked via Skill tool, spawns background Bash sub-agent to execute clean.sh directly
- **Tested**: Script executes successfully and reports PASS when no simulation nodes remain
## Key Learnings
- Cartographer in ROS2 Humble uses `map_builder.lua` + `trajectory_builder.lua`, NOT `cartographer_ros.lua` (ROS1-only)
- Cartographer requires IMU frame colocated with tracking_frame — if IMU has frame_id `base_footprint`, set `tracking_frame = "base_footprint"` or disable IMU with `use_imu_data = false`
- EKF from robot_localization needs `two_d_mode: true` for flat 2D robots, otherwise it may not publish TF
- Fast-DDS SHM transport (`RMW_FASTRTPS_USE_SHM`) can cause message loss on `/tf` — disable with `SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0')` in launch files
- Cartographer should consume EKF-filtered odometry (`/odometry/filtered`, frame `odom`) rather than raw `/wheel_odom` (frame `wheel_odom`)
- Diff_drive `publish_odom_tf: true` 会发布 `wheel_odom→base_link` TF，与 EKF 的 `odom→base_link` 冲突导致 `base_link` 双父帧，TF 树分裂。EKF 模式下必须设为 `publish_odom_tf: false`
- EKF `odom0_differential: true` 让里程计数据作为增量而非绝对位姿融合，避免帧不匹配导致的滤波器发散
- EKF `odom0_config` 中索引 5（位姿 yaw）和索引 11（角速度 yaw）都提供朝向信息，转弯时轮式里程计 yaw 有噪声，与 IMU 冲突导致定位晃动。建图模式下只保留索引 11，去掉索引 5

## Do-Not-Repeat

<!-- Mistakes made and corrected. Each entry prevents the same mistake recurring. -->
<!-- Format: [YYYY-MM-DD] Description of what went wrong and what to do instead. -->
- [2026-06-27] **多进程残留问题**: Claude Code 每次用 Bash 工具启动仿真是独立的后台任务，如果上一次任务还没结束又启动新任务，会产生多个 gzserver 进程互相冲突。
  - **现象**: ps aux 能看到多组 gzserver/gzclient/ros2 launch 进程在跑
  - **教训**: **每次启动仿真前**，必须先关闭上一次的仿真，如果有残留，调用 ros-simulation-clean skill，确保只有一个 Gazebo 实例
  - **正确流程**: `ros2 node list` 检查残留 → 有则 ros-simulation-clean → verify → launch → test → clean(如果测试完成)
- [2026-06-27] **修改代码后运行仿真流程**: 修改代码（如 xacro、cpp、launch）后，必须走完整流程才能生效：
  1. `colcon build --packages-select <package>` 或 `./build_sim.sh` 重新编译
  2. `source install/setup.sh` 让修改生效
  3. `ros2 launch ...` 启动仿真
  - **直接 ros2 launch 而不先 build 的话，修改不会生效**
- [2026-06-27] **修改/仿真/测试工作流经验**: 通过多次测试验证运动稳定性，总结出正确的通用工作流程以避免进程冲突、修改失效和假阳性结果。
  - **现象**: 未清理残留进程导致多个仿真实例冲突；未重新编译导致修改未生效；测试流程不统一导致结果不可靠
  - **教训**: 每次修改必须遵循完整流程，每次测试必须确保环境干净
  - **正确流程**: 
    1. **修改阶段**: 编辑代码 → 保存文件
    2. **编译阶段**: `colcon build --packages-select <package_name>`  重新编译 affected packages 或 `./build_sim.sh` 全部重新编译
    3. **环境准备**: `source install/setup.bash` 让修改生效
    4. **仿真准备**: 
       - 检查残留: `source /opt/ros/humble/setup.bash && ros2 node list` 
       - 有残留则运行: `bash /home/pi/workplace/robot_adam/.claude/skills/ros-simulation-clean/clean.sh`
       - 验证清理成功 (应显示 "PASS: No residual simulation nodes detected")
    5. **启动测试**: `ros2 launch <package> <launch_file>` 启动仿真
    6. **测试执行**: 根据情况使用类似`ros2 topic pub` 发送控制或导航命令，观察相应话题（如 `/odom`、`/tf` 等）验证系统状态
    7. **测试结束**: 清理进程（使用 ros-simulation-clean skill）以准备后续操作


- [2026-06-28] **verify-ros-clean hook 不生效**:
  - **原因**: settings.json 的 PostToolUse 中没有注册 Skill 的 hook matcher，所以 verify-ros-clean.js 从未作为 hook 执行
  - **另**: 该 hook 原本把非核心节点全部标记为残留（如 joy_linux_node、teleop_twist_joy_node、ekf_filter_node 等正常机器人控制节点），产生大量误报
  - **修复**: 
    1. settings.json PostToolUse 中添加 Skill matcher → `"matcher": "Skill"`
    2. hook 改为用仿真特定模式匹配，只检测 gazebo/gzserver/gzclient/ignition/spawn_entity/robot_state_publisher/joint_state_publisher 相关节点
