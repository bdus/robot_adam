#!/usr/bin/env python3
"""
adam_test_tools / apply_force.py
Apply physical impulse to robot model in Gazebo.

Services:
  /test_tools/apply_force (custom: fx, fy, fz as float, duration_ms)
    Applies an external force to the robot's base_link.

Designed for SPEC 02 Gate 2 (collision/side-impact test).
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point
import threading
import time


class ApplyForceNode(Node):

    def __init__(self):
        super().__init__("test_apply_force")
        self.declare_parameter("robot_name", "laser_2wd")
        self.declare_parameter("link_name", "base_link")
        self._robot_name = self.get_parameter("robot_name").value
        self._link_name = self.get_parameter("link_name").value

        self._client = self.create_client(ApplyBodyWrench, "/gazebo/apply_body_wrench")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/apply_body_wrench...")

        self.srv = self.create_service(Trigger, "/test_tools/apply_force", self._callback)
        self.get_logger().info(f"ApplyForceNode ready on /test_tools/apply_force")

    def _apply_wrench(self, fx: float, fy: float, fz: float,
                      duration_ms: int = 500):
        body_name = f"{self._robot_name}::{self._link_name}"
        req = ApplyBodyWrench.Request()
        req.body_name = body_name
        req.wrench = Wrench()
        req.wrench.force.x = fx
        req.wrench.force.y = fy
        req.wrench.force.z = fz
        req.duration.sec = duration_ms // 1000
        req.duration.nanosec = (duration_ms % 1000) * 1_000_000
        req.reference_frame = "world"

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if future.result() is not None:
            return future.result().success
        return False

    def _callback(self, request, response):
        # Parse fx, fy, fz, duration_ms from request.message
        msg = request.message.strip() if hasattr(request, "message") else ""
        parts = msg.split()

        try:
            if len(parts) == 4:
                fx, fy, fz = float(parts[0]), float(parts[1]), float(parts[2])
                duration_ms = int(parts[3])
            elif len(parts) == 3:
                fx, fy, fz = float(parts[0]), float(parts[1]), float(parts[2])
                duration_ms = 200
            else:
                # Default: moderate lateral push
                fx, fy, fz = 0.0, 50.0, 0.0
                duration_ms = 200
                if msg:
                    raise ValueError(f"Invalid format: '{msg}'. Use: 'fx fy fz [duration_ms]'")

            ok = self._apply_wrench(fx, fy, fz, duration_ms)
            response.success = ok
            response.message = f"Applied force ({fx}, {fy}, {fz}) for {duration_ms}ms"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"apply_force error: {e}")

        return response


def main():
    rclpy.init()
    node = ApplyForceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
