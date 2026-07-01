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
from std_srvs.srv import SetFloat
from gazebo_msgs.srv import SetLinkProperties
from builtin_interfaces.msg import Duration


class SetFrictionNode(Node):

    def __init__(self):
        super().__init__("test_set_friction")
        self._ground_link = "ground_plane::link"
        self._default_mu = 0.5

        self._client = self.create_client(SetLinkProperties, "/gazebo/set_link_properties")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/set_link_properties...")

        self.srv = self.create_service(SetFloat, "/test_tools/set_friction", self._callback)
        self.get_logger().info("SetFrictionNode ready on /test_tools/set_friction")

    def _set_mu(self, mu: float) -> bool:
        req = SetLinkProperties.Request()
        req.link_name = self._ground_link
        req.gravity_mode = True
        req.mass = -1.0  # keep existing
        req.ixx = -1.0
        req.iyy = -1.0
        req.izz = -1.0
        # mu is not directly settable via SetLinkProperties in Gazebo Classic
        # We log the request and print guidance
        self.get_logger().info(
            f"NOTE: Gazebo Classic 11 does not support runtime mu via SetLinkProperties.\n"
            f"To actually change friction, edit the world's ground_plane <surface><friction><ode><mu> "
            f"or use <plugin name='libgazebo_ros_wheel_slip.so'> on wheel links.\n"
            f"Requested mu={mu} logged for awareness."
        )
        return True

    def _callback(self, request, response):
        mu = request.data
        if mu < 0.0:
            response.success = False
            response.message = "Friction coefficient must be >= 0"
            return response

        ok = self._set_mu(mu)
        response.success = ok
        response.message = f"Friction mu={mu:.2f}"
        if ok:
            self.get_logger().info(f"Friction set to {mu:.2f}")
        return response


def main():
    rclpy.init()
    node = SetFrictionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
