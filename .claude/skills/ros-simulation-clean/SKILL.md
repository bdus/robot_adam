---
name: ros-simulation-clean
description: Aggressively clean up Gazebo/ROS2 simulation residual processes. This skill spawns a single background Bash sub-agent that runs the clean.sh script directly (no LLM involvement). The script kills gzserver/gzclient/ignition processes and robot_state_publisher (when simulation-related), then verifies via ros2 node list in a retry loop (max 5 attempts). Also detects and resolves port 11345 (Gazebo master) conflicts by extracting the blocking PID and force-killing it, plus cleans up Gazebo lock files (/tmp/*gazebo*, ~/.gazebo/*.lock). Use when Gazebo simulation leaves dangling nodes, port 11345 is occupied ("bind: Address already in use"), or simulation processes won't terminate cleanly.
user-invocable: true
---

# ROS Simulation Clean

**When to use**: After Gazebo simulation crashes or hangs, leaving residual processes that block new simulations, or when the user explicitly requests cleaning simulation nodes.

**Common failure modes this skill resolves**:
- `Unable to start server[bind: Address already in use]` — old gzserver holding port 11345
- `Service /spawn_entity unavailable` — Gazebo not fully started due to port conflict
- `queue limit reached` warnings — stale publishers from zombie processes

**Execution**: Upon invocation, spawn ONE background Bash sub-agent to execute `.claude/skills/ros-simulation-clean/clean.sh` directly. **Do NOT read the script content** — just run it via Bash. The script contains all control logic (loops, process killing, port check, verification) internally.

**Reporting**: After the sub-agent completes, report its exit code and stdout/stderr summary in one line to the user.