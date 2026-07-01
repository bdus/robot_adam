#!/usr/bin/env python3
"""
adam_test_tools / spawn_actor.py
Spawn and control a pedestrian actor in Gazebo.

Services:
  /test_tools/spawn_actor (std_srvs/srv/Trigger)
    trigger: start pedestrian crossing path

  /test_tools/stop_actor (std_srvs/srv/Trigger)
    trigger: stop/remove actor

The actor walks from left to right across the robot's forward path at ~1.0 m/s.
Designed for SPEC 03 Gate 3 (STVL dynamic pedestrian test).
"""

import os
import tempfile
import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

ACTOR_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.6">
  <actor name="{name}">
    <pose>{start_x} {start_y} 0 0 0 0</pose>
    <skin>
      <filename>file:///usr/share/gz/gazebo-11/media/skins/actor.gltf</filename>
      <scale>1.0</scale>
    </skin>
    <animation name="walk">
      <filename>file:///usr/share/gz/gazebo-11/media/animation/actor_walk.dae</filename>
      <scale>1.0</scale>
      <interpolate_x>true</interpolate_x>
    </animation>
    <script>
      <loop>false</loop>
      <delay_start>0.5</delay_start>
      <trajectory id="0" type="walking">
        <waypoint><time>0.0</time><pose>{start_x} {start_y} 0 0 0 0</pose></waypoint>
        <waypoint><time>{duration_s}</time><pose>{end_x} {end_y} 0 0 0 0</pose></waypoint>
      </trajectory>
    </script>
  </actor>
</sdf>"""


class SpawnActorNode(Node):

    def __init__(self):
        super().__init__("test_spawn_actor")
        self._actor_spawned = False
        self._actor_name = "test_pedestrian"

        self.srv_spawn = self.create_service(Trigger, "/test_tools/spawn_actor", self._spawn_callback)
        self.srv_stop = self.create_service(Trigger, "/test_tools/stop_actor", self._stop_callback)
        self.get_logger().info("SpawnActorNode ready on /test_tools/spawn_actor")

    def _spawn_actor(self, name: str, start_x: float, start_y: float,
                     end_x: float, end_y: float, duration_s: float = 10.0):
        sdf = ACTOR_SDF_TEMPLATE.format(
            name=name, start_x=start_x, start_y=start_y,
            end_x=end_x, end_y=end_y, duration_s=duration_s,
        )
        with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", delete=False) as f:
            f.write(sdf)
            sdf_path = f.name

        cmd = [
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-file", sdf_path, "-entity", name,
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        os.unlink(sdf_path)

        if result.returncode != 0:
            raise RuntimeError(f"spawn_actor failed: {result.stderr.strip()}")

    def _remove_actor(self, name: str):
        cmd = [
            "ros2", "service", "call",
            "/delete_entity",
            "gazebo_msgs/srv/DeleteEntity",
            f"{{name: {name}}}",
        ]
        subprocess.run(cmd, capture_output=True, text=True, timeout=10)

    def _spawn_callback(self, request, response):
        _ = request
        if self._actor_spawned:
            response.success = False
            response.message = "Actor already spawned, stop first"
            return response

        try:
            # Default: cross path from robot left (-4, 2) to right (4, 2) at ~1m/s
            self._spawn_actor(self._actor_name,
                              start_x=-4.0, start_y=2.0,
                              end_x=4.0, end_y=2.0,
                              duration_s=8.0)
            self._actor_spawned = True
            response.success = True
            response.message = "Pedestrian actor spawned, crossing path"
            self.get_logger().info("Pedestrian actor spawned")
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"spawn_actor error: {e}")

        return response

    def _stop_callback(self, request, response):
        _ = request
        try:
            self._remove_actor(self._actor_name)
            self._actor_spawned = False
            response.success = True
            response.message = "Actor removed"
            self.get_logger().info("Pedestrian actor removed")
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"stop_actor error: {e}")

        return response


def main():
    rclpy.init()
    node = SpawnActorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
