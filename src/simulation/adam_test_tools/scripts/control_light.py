#!/usr/bin/env python3
"""
adam_test_tools / control_light.py
Control Gazebo scene lighting via SetLightProperties service.

Services:
  /test_tools/toggle_light (std_srvs/srv/SetBool)
    true  = restore full diffuse intensity (light on)
    false = set diffuse to zero (light off)

Uses Gazebo built-in /gazebo/set_light_properties to control the "main_light".
Designed for dark_room_test.world (SPEC 04/05 Gate 3).
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from gazebo_msgs.srv import SetLightProperties
from std_msgs.msg import ColorRGBA


class ControlLightNode(Node):

    def __init__(self):
        super().__init__("test_control_light")
        self.declare_parameter("light_name", "main_light")
        self.declare_parameter("diffuse_on", [0.8, 0.8, 0.8, 1.0])
        self.declare_parameter("diffuse_off", [0.0, 0.0, 0.0, 1.0])

        self._light_name = self.get_parameter("light_name").value
        self._diffuse_on = self.get_parameter("diffuse_on").value
        self._diffuse_off = self.get_parameter("diffuse_off").value

        self._client = self.create_client(SetLightProperties, "/gazebo/set_light_properties")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /gazebo/set_light_properties...")

        self.srv = self.create_service(SetBool, "/test_tools/toggle_light", self._callback)
        self.get_logger().info(f"ControlLightNode ready on /test_tools/toggle_light (light: {self._light_name})")

    def _set_light_diffuse(self, rgba: list):
        req = SetLightProperties.Request()
        req.light_name = self._light_name
        req.diffuse = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Light {self._light_name} diffuse set to {rgba}")
            return True
        else:
            self.get_logger().error(f"Failed to set light {self._light_name}")
            return False

    def _callback(self, request, response):
        if request.data:
            ok = self._set_light_diffuse(self._diffuse_on)
            response.message = "Light ON"
        else:
            ok = self._set_light_diffuse(self._diffuse_off)
            response.message = "Light OFF"

        response.success = ok
        return response


def main():
    rclpy.init()
    node = ControlLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
