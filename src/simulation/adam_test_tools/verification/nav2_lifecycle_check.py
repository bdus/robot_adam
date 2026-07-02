#!/usr/bin/env python3
"""
adam_test_tools / verification / nav2_lifecycle_check.py
Check lifecycle state of all Nav2 managed nodes.

Iterates through all expected Nav2 lifecycle nodes and reports their
state (active / inactive / unknown). Returns PASS if all required nodes
are active, FAIL otherwise.

Topics:
  /test_verdict/nav2_lifecycle (std_msgs/msg/String)
    Publishes PASS / FAIL verdict with per-node state breakdown.

Gate: SPEC 02 Gate 6 — Nav2 lifecycle nodes must reach active state.

Usage:
  ros2 run adam_test_tools nav2_lifecycle_check.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_lifecycle_msgs.srv import GetState
from nav2_lifecycle_msgs.msg import State
import time


LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]

STATE_PRIMARY_ACTIVE = 3  # State.PRIMARY_STATE_ACTIVE


class Nav2LifecycleCheckNode(Node):
    """Check all Nav2 lifecycle nodes are in active state."""

    def __init__(self):
        super().__init__("test_nav2_lifecycle_check")
        self.verdict_pub = self.create_publisher(String, "/test_verdict/nav2_lifecycle", 1)
        self._check_finished = False

    def check_once(self) -> bool:
        """Check all lifecycle nodes and publish verdict. Returns True if all active."""
        all_active = True
        lines = ["Nav2 Lifecycle Check:"]

        for node_name in LIFECYCLE_NODES:
            service_name = f"/{node_name}/get_state"
            client = self.create_client(GetState, service_name)

            if not client.wait_for_service(timeout_sec=2.0):
                lines.append(f"  {node_name}: UNKNOWN (service not available)")
                all_active = False
                continue

            req = GetState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is None:
                lines.append(f"  {node_name}: UNKNOWN (no response)")
                all_active = False
                continue

            current_state = future.result().current_state.id
            if current_state == STATE_PRIMARY_ACTIVE:
                lines.append(f"  {node_name}: active [3]")
            else:
                state_names = {1: "unconfigured [1]", 2: "inactive [2]", 3: "active [3]", 4: "finalized [4]"}
                state_name = state_names.get(current_state, f"unknown [{current_state}]")
                lines.append(f"  {node_name}: {state_name}")
                all_active = False

            self.remove_client(client)

        verdict = "PASS" if all_active else "FAIL"
        lines.insert(0, f"Verdict: {verdict}")
        msg = "\n".join(lines)
        self.get_logger().info(f"\n{msg}")
        self.verdict_pub.publish(String(data=msg))
        return all_active


def main():
    rclpy.init()
    node = Nav2LifecycleCheckNode()

    try:
        node.check_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
