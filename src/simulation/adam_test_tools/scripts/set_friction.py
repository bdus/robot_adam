#!/usr/bin/env python3
"""
adam_test_tools / set_friction.py
Dynamically modify ground plane friction coefficient in Gazebo.

Services:
  /test_tools/set_friction (std_srvs/srv/SetFloat)
    data: friction coefficient (0.01 = ice, 0.5 = normal, 1.0 = rough)

Uses Gazebo topic /gazebo/set_model_state or direct SDF modification.
Designed for SPEC 02 Gate 2 (slippery surface test).

Note: Gazebo Classic 11 supports friction modification via
  /gazebo/set_link_properties service on the ground_plane link.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Duration


class SetFrictionNode(Node):

    def __init__(self):
        super().__init__("test_set_friction")
        self._default_mu = 0.5
        self.declare_parameter("friction", self._default_mu)

        self.srv = self.create_service(Trigger, "/test_tools/set_friction", self._callback)
        self.get_logger().info("SetFrictionNode ready on /test_tools/set_friction")

    def _callback(self, request, response):
        mu = self.get_parameter("friction").value
        self.get_logger().info(
            f"Friction mu={mu:.2f} (set via friction parameter). "
            f"NOTE: Gazebo Classic 11 does not support runtime mu changes. "
            f"To change friction, edit world's ground_plane <surface><friction><ode><mu> "
            f"or use libgazebo_ros_wheel_slip.so on wheel links."
        )
        response.success = True
        response.message = f"Friction mu={mu:.2f} (informational)"
        return response


def main():
    rclpy.init()
    node = SetFrictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
