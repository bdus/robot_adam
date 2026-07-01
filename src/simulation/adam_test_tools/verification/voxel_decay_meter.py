#!/usr/bin/env python3
"""
adam_test_tools / verification / voxel_decay_meter.py
Measure STVL voxel decay time after pedestrian leaves view.

Topics:
  /test_verdict/voxel_decay (std_msgs/msg/String)
    Publishes PASS / FAIL with measured decay time.

Gate: SPEC 03 Gate 3 — voxel decay 1.0s ± 0.1s

Note: STVL does not publish a standard voxel grid topic by default.
This node monitors STVL's local costmap updates as a proxy — when
costmap values return to baseline (free space), decay is complete.

A more precise implementation would subscribe to the voxel grid topic
published by STVL when publish_voxel_map: true is set in nav2 params.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import time


class VoxelDecayMeterNode(Node):

    def __init__(self):
        super().__init__("test_voxel_decay_meter")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap_raw")
        self.declare_parameter("roi_x", 0.0)   # robot-local x of pedestrian area
        self.declare_parameter("roi_y", 2.0)   # robot-local y of pedestrian area
        self.declare_parameter("roi_radius", 1.0)
        self.declare_parameter("decay_target_s", 1.0)
        self.declare_parameter("decay_tolerance_s", 0.1)

        self._costmap_topic = self.get_parameter("costmap_topic").value
        self._roi_x = self.get_parameter("roi_x").value
        self._roi_y = self.get_parameter("roi_y").value
        self._roi_r = self.get_parameter("roi_radius").value
        self._target = self.get_parameter("decay_target_s").value
        self._tolerance = self.get_parameter("decay_tolerance_s").value

        self._last_obstacle_time = 0.0
        self._last_clear_time = 0.0
        self._was_obstructed = False
        self._decay_start = 0.0

        self._sub = self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._costmap_cb, 10)
        self._pub = self.create_publisher(String, "/test_verdict/voxel_decay", 10)
        self._timer = self.create_timer(1.0, self._report)

        self.get_logger().info(
            f"VoxelDecayMeter: monitoring {self._costmap_topic}, "
            f"decay target={self._target}s ±{self._tolerance}s"
        )

    def _costmap_cb(self, msg: OccupancyGrid):
        now = time.time()

        # Check if the ROI (pedestrian area) has obstacles
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height

        center_cx = int((self._roi_x - origin_x) / resolution)
        center_cy = int((self._roi_y - origin_y) / resolution)
        radius_px = int(self._roi_r / resolution)

        obstructed = False
        for dy in range(-radius_px, radius_px + 1):
            for dx in range(-radius_px, radius_px + 1):
                if dx * dx + dy * dy > radius_px * radius_px:
                    continue
                ix = center_cx + dx
                iy = center_cy + dy
                if 0 <= ix < width and 0 <= iy < height:
                    idx = iy * width + ix
                    if idx < len(msg.data) and msg.data[idx] > 50:
                        obstructed = True
                        break
            if obstructed:
                break

        if obstructed:
            self._last_obstacle_time = now
            self._was_obstructed = True
        elif self._was_obstructed:
            # Just became clear — start decay timer
            self._last_clear_time = now
            self._decay_start = self._last_obstacle_time
            self._was_obstructed = False

    def _report(self):
        if self._decay_start == 0:
            return

        decay_s = self._last_clear_time - self._decay_start
        lower = self._target - self._tolerance
        upper = self._target + self._tolerance

        if lower <= decay_s <= upper:
            verdict = "PASS"
        else:
            verdict = "FAIL"

        msg = String()
        msg.data = (
            f"[VOXEL_DECAY] verdict={verdict} "
            f"decay_s={decay_s:.2f}/{self._target:.1f}±{self._tolerance:.1f} "
            f"obstacle_until={self._decay_start:.1f} clear_at={self._last_clear_time:.1f}"
        )
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = VoxelDecayMeterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
