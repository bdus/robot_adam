## Skill: ros-simulation-clean
- **Purpose**: Clean up Gazebo/ROS2 simulation residual processes
- **Files created**:
  - .claude/skills/ros-simulation-clean/SKILL.md (skill definition)
  - .claude/skills/ros-simulation-clean/clean.sh (executable Bash script)
- **Logic**: Pure Bash control loop with max 5 retries, kills gzserver/gzclient/ruby/*ign/*gz/robot_state_publisher (sim-related)/ros2_control_node/component_container/ros2 launch processes, verifies with ros2 node list and port 11345 check
- **Usage**: Invoked via Skill tool, spawns background Bash sub-agent to execute clean.sh directly
- **Tested**: Script executes successfully and reports PASS when no simulation nodes remain
## Do-Not-Repeat

<!-- Mistakes made and corrected. Each entry prevents the same mistake recurring. -->
<!-- Format: [YYYY-MM-DD] Description of what went wrong and what to do instead. -->
- [2026-06-27] **多进程残留问题**: Claude Code 每次用 Bash 工具启动仿真都是独立的后台任务，如果上一次任务还没结束又启动新任务，会产生多个 gzserver 进程互相冲突。
  - **现象**: ps aux 能看到多组 gzserver/gzclient/ros2 launch 进程在跑
  - **教训**: **每次启动仿真前**，必须先关闭上一次的仿真，如果有残留，调用 ros-simulation-clean skill，确保只有一个 Gazebo 实例
  - **正确流程**: `ros2 node list` 检查残留 → 有则 ros-simulation-clean → verify → launch → test → clean(如果测试完成)
- [2026-06-27] **修改代码后运行仿真流程**: 修改代码（如 xacro、cpp、launch）后，必须走完整流程才能生效：
  1. `colcon build --packages-select <package>` 或 `./build_sim.sh` 重新编译
  2. `source install/setup.sh` 让修改生效
  3. `ros2 launch ...` 启动仿真
  - **直接 ros2 launch 而不先 build 的话，修改不会生效**

