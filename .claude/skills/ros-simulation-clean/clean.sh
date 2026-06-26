#!/bin/bash
# ==============================================================================
# Skill: ros-simulation-clean
# Description: Deterministically cleans up Gazebo and ROS2 simulation remnants.
# Invoked by SKILL.md. All logic in Bash — zero LLM cycles.
# ==============================================================================

MAX_RETRIES=5
RETRY_COUNT=0

# Source ROS2 environment — try common distros, fall back to user's env
for setup in \
    "/opt/ros/${ROS_DISTRO}/setup.bash" \
    /opt/ros/jazzy/setup.bash \
    /opt/ros/humble/setup.bash \
    /opt/ros/iron/setup.bash \
    /opt/ros/rolling/setup.bash; do
    if [ -f "$setup" ]; then
        source "$setup"
        break
    fi
done

clean_processes() {
    # Kill all known simulation-spawned processes aggressively
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "ruby.*ign" 2>/dev/null || true
    pkill -9 -f "ruby.*gz" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "ros2_control_node" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    # Only kill robot_state_publisher if it's simulation-related (often spawned by Gazebo)
    pkill -9 -f "robot_state_publisher.*gazebo" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher.*sim" 2>/dev/null || true

    # Clean up Gazebo lock files that can prevent restart
    rm -rf /tmp/*gazebo* /tmp/*ignition* 2>/dev/null || true
    rm -rf "$HOME/.gazebo"/*.lock 2>/dev/null || true

    # Reset ROS2 daemon to flush stale node references
    ros2 daemon stop 2>/dev/null || true
    sleep 0.5
    ros2 daemon start 2>/dev/null || true
    sleep 1.0
}

check_port() {
    local port=${1:-11345}
    if ss -tlnp | grep -q ":$port "; then
        echo "PORT-CONFLICT: Port $port is still in use after process kill. Forcing release..."
        local pid
        pid=$(ss -tlnp "sport = :$port" | grep -oP 'pid=\K\d+' | head -1)
        if [ -n "$pid" ]; then
            kill -9 "$pid" 2>/dev/null || true
            sleep 1
        fi
        if ss -tlnp | grep -q ":$port "; then
            echo "PORT-CONFLICT: Port $port still blocked. Waiting 3s..."
            sleep 3
        fi
        if ss -tlnp | grep -q ":$port "; then
            echo "FAIL: Port $port remains occupied. Manual intervention required (fuser -k ${port}/tcp)."
            return 1
        fi
    fi
    echo "OK: Port $port is available."
    return 0
}

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    NODE_LIST=$(ros2 node list 2>/dev/null | grep -v '^$')

    if [ -z "$NODE_LIST" ]; then
        # No ROS2 nodes — but could still have a zombie gzserver blocking port
        check_port 11345 || exit 1
        echo "PASS: No residual ROS2 simulation nodes detected."
        exit 0
    fi

    # Filter for simulation-related nodes only
    SIM_NODES=$(echo "$NODE_LIST" | grep -E "(gazebo|sim|gz)" || true)

    if [ -z "$SIM_NODES" ]; then
        check_port 11345 || exit 1
        echo "PASS: No residual simulation nodes detected (only core/system nodes remain)."
        exit 0
    fi

    echo "WARN: Detected simulation nodes (attempt $((RETRY_COUNT + 1))/$MAX_RETRIES):"
    echo "$SIM_NODES"

    clean_processes
    ((RETRY_COUNT++))
done

# Final port check even on failure
check_port 11345 || true

echo "FAIL: Simulation nodes remain after $MAX_RETRIES cleanup attempts."
exit 1