#!/usr/bin/env python3
"""
adam_test_tools / spawn_object.py
Dynamic obstacle spawning and deletion in Gazebo.

Services:
  /test_tools/spawn_object (custom_interfaces/srv/SpawnCommand)
    command: "spawn <model_type> <x> <y> <z>"
             "delete <entity_name>"
             "clear"

Built-in models: box (0.3m cube), cylinder (0.2m dia x 0.5m), barrier (1.0m x 0.2m x 0.5m)
"""

import os
import sys
import tempfile
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


MODEL_SDF = {
    "box": """
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="col">
        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        <material><ambient>0.9 0.2 0.2 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>""",
    "cylinder": """
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="col">
        <geometry><cylinder><radius>0.1</radius><length>0.5</length></cylinder></geometry>
      </collision>
      <visual name="vis">
        <geometry><cylinder><radius>0.1</radius><length>0.5</length></cylinder></geometry>
        <material><ambient>0.2 0.2 0.9 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>""",
    "barrier": """
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="col">
        <geometry><box><size>1.0 0.2 0.5</size></box></geometry>
      </collision>
      <visual name="vis">
        <geometry><box><size>1.0 0.2 0.5</size></box></geometry>
        <material><ambient>0.9 0.6 0.1 1</ambient></material>
      </visual>
    </link>
  </model>
</sdf>""",
}


class SpawnObjectNode(Node):

    def __init__(self):
        super().__init__("test_spawn_object")
        self._counter = 0
        self._spawned_entities = []
        self.srv = self.create_service(Trigger, "/test_tools/spawn_object", self._callback)
        self.get_logger().info("SpawnObjectNode ready on /test_tools/spawn_object")

    def _spawn_model(self, model_type: str, x: float, y: float, z: float) -> str:
        if model_type not in MODEL_SDF:
            raise ValueError(f"Unknown model type: {model_type}. Available: {list(MODEL_SDF.keys())}")

        self._counter += 1
        entity_name = f"test_object_{self._counter}"

        sdf_content = MODEL_SDF[model_type].format(name=entity_name)

        with tempfile.NamedTemporaryFile(mode="w", suffix=".sdf", delete=False) as f:
            f.write(sdf_content)
            sdf_path = f.name

        cmd = [
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-file", sdf_path,
            "-entity", entity_name,
            "-x", str(x), "-y", str(y), "-z", str(z),
        ]

        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        os.unlink(sdf_path)

        if result.returncode != 0:
            raise RuntimeError(f"spawn_entity failed: {result.stderr.strip()}")

        self._spawned_entities.append(entity_name)
        return entity_name

    def _delete_model(self, entity_name: str) -> None:
        cmd = [
            "ros2", "service", "call",
            "/delete_entity",
            "gazebo_msgs/srv/DeleteEntity",
            f"{{name: {entity_name}}}",
        ]
        subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if entity_name in self._spawned_entities:
            self._spawned_entities.remove(entity_name)

    def _clear_all(self) -> None:
        for ent in list(self._spawned_entities):
            self._delete_model(ent)

    def _callback(self, request, response):
        cmd = request.message if hasattr(request, "message") else ""
        cmd = cmd.strip()

        try:
            parts = cmd.split()
            if not parts:
                raise ValueError("Empty command")

            command = parts[0]

            if command == "spawn" and len(parts) >= 5:
                model_type = parts[1]
                x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
                name = self._spawn_model(model_type, x, y, z)
                response.success = True
                response.message = f"Spawned {name} at ({x}, {y}, {z})"

            elif command == "delete" and len(parts) >= 2:
                entity_name = parts[1]
                self._delete_model(entity_name)
                response.success = True
                response.message = f"Deleted {entity_name}"

            elif command == "clear":
                self._clear_all()
                response.success = True
                response.message = "All test objects cleared"

            else:
                raise ValueError(f"Invalid command: {cmd}")

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"spawn_object error: {e}")

        return response


def main():
    rclpy.init()
    node = SpawnObjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
