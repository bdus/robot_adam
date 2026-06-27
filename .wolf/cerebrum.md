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

