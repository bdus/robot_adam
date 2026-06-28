#!/bin/bash
# ==============================================================================
# Skill: ros-simulation-clean
# Description: Nuclear clean of Gazebo/ROS2 simulation — kills ALL remaining
# ROS2 nodes (not just simulation-specific ones), resets daemon, frees port.
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

nuke_all_nodes() {
    # Phase 1: Kill known simulation-spawned processes
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    pkill -9 -f "ruby.*ign" 2>/dev/null || true
    pkill -9 -f "ruby.*gz" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "ros2_control_node" 2>/dev/null || true
    pkill -9 -f "component_container" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "joint_state_publisher" 2>/dev/null || true
    pkill -9 -f "spawn_entity" 2>/dev/null || true

    # Phase 2: Kill ALL remaining ROS2 node processes dynamically
    # Every non-core node found in ros2 node list gets its process killed
    local remaining
    remaining=$(ros2 node list 2>/dev/null | grep -v -E '^/_$|^/rosout$|WARNING' || true)
    if [ -n "$remaining" ]; then
        echo "$remaining" | while IFS= read -r node; do
            local proc_name
            # Strip leading / to get the node name as process identifier
            proc_name=$(echo "$node" | sed 's|^/||')
            pkill -9 -f "$proc_name" 2>/dev/null || true
        done
        sleep 0.5
    fi

    # Phase 3: Kill orphaned ros2 CLI processes (topic pub, topic echo, etc.)
    pkill -9 -f "ros2 topic" 2>/dev/null || true
    pkill -9 -f "ros2 service" 2>/dev/null || true
    pkill -9 -f "ros2 bag" 2>/dev/null || true

    # Clean up Gazebo lock files that can prevent restart
    rm -rf /tmp/*gazebo* /tmp/*ignition* 2>/dev/null || true
    rm -rf "$HOME/.gazebo"/*.lock 2>/dev/null || true

    # Hard reset ROS2 daemon — processes may be dead but daemon caches stale nodes
    ros2 daemon stop 2>/dev/null || true
    pkill -9 -f "ros_daemon" 2>/dev/null || true
    sleep 0.5
    ros2 daemon start 2>/dev/null || true
    sleep 2.0
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
    nuke_all_nodes

    NODE_LIST=$(ros2 node list 2>/dev/null | grep -v '^$' | grep -v 'WARNING')

    if [ -z "$NODE_LIST" ] || [ "$(echo "$NODE_LIST" | grep -c -v -E '^/_$|^/rosout$')" -eq 0 ]; then
        check_port 11345 || exit 1
        echo "PASS: No residual ROS2 nodes detected."
        exit 0
    fi

    echo "WARN: Nodes still present (attempt $((RETRY_COUNT + 1))/$MAX_RETRIES):"
    echo "$NODE_LIST"
    ((RETRY_COUNT++))
done

# Final port check even on failure
check_port 11345 || true

echo "FAIL: Nodes remain after $MAX_RETRIES cleanup attempts."
exit 1
