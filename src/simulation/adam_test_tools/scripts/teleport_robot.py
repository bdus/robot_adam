#!/usr/bin/env python3
"""
adam_test_tools / teleport_robot.py
Teleport robot to any position in Gazebo.

Services:
  /test_tools/teleport (gazebo_msgs/srv/SetEntityState)
    Sets the pose of the robot model (default: laser_2wd) to target (x, y, yaw).

Usage:
  ros2 service call /test_tools/teleport gazebo_msgs/srv/SetEntityState \
    "{state: {name: 'laser_2wd', pose: {position: {x: 5.0, y: 3.0, z: 0.0}, \
    orientation: {z: 0.707, w: 0.707}}}}"
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import math


def yaw_to_quat(yaw: float):
    """Convert yaw angle to quaternion (z, w)."""
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TeleportRobotNode(Node):

    def __init__(self):
        super().__init__("test_teleport_robot")
        self.declare_parameter("robot_name", "laser_2wd")

        self._robot_name = self.get_parameter("robot_name").value
        self._client = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/set_entity_state...")

        self.srv = self.create_service(SetEntityState, "/test_tools/teleport", self._callback)
        self.get_logger().info(
            f"TeleportRobotNode ready on /test_tools/teleport (robot: {self._robot_name})"
        )

    def _callback(self, request, response):
        state = request.state
        if not state.name:
            state.name = self._robot_name

        req = SetEntityState.Request()
        req.state = state

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response.success = future.result().success
            response.status_message = f"Teleported {state.name} to ({state.pose.position.x:.2f}, {state.pose.position.y:.2f})"
            self.get_logger().info(response.status_message)
        else:
            response.success = False
            response.status_message = "Failed to set entity state"

        return response


def main():
    rclpy.init()
    node = TeleportRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
