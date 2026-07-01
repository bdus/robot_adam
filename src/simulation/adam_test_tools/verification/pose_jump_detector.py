#!/usr/bin/env python3
"""
adam_test_tools / verification / pose_jump_detector.py
Detect sudden jumps in map→odom TF transformation.

Topics:
  /test_verdict/pose_jump (std_msgs/msg/String)
    Publishes PASS / FAIL when jump exceeds threshold.

Gates:
  SPEC 02 Gate 5 — relocalization jump ≤ 3cm, ≤ 2°
  SPEC 04/05 Gate 3 — dark room blind drive, jump ≤ 3cm
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time


def quat_to_yaw(q):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PoseJumpDetectorNode(Node):

    def __init__(self):
        super().__init__("test_pose_jump_detector")
        self.declare_parameter("max_jump_m", 0.03)
        self.declare_parameter("max_jump_deg", 2.0)
        self.declare_parameter("window_size", 5)

        self._max_jump_m = self.get_parameter("max_jump_m").value
        self._max_jump_deg = self.get_parameter("max_jump_deg").value
        self._window = self.get_parameter("window_size").value

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._prev_x = None
        self._prev_y = None
        self._prev_yaw = None
        self._prev_stamp = None
        self._max_delta_pos = 0.0
        self._max_delta_ang = 0.0
        self._max_jump_pos = 0.0
        self._max_jump_ang = 0.0

        self._pub = self.create_publisher(String, "/test_verdict/pose_jump", 10)
        self._timer = self.create_timer(0.1, self._check)  # 10Hz

        self.get_logger().info(
            f"PoseJumpDetector: max jump ≤ {self._max_jump_m*100:.0f}cm / {self._max_jump_deg:.0f}°"
        )

    def _check(self):
        try:
            t: TransformStamped = self._tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time())
        except Exception:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = quat_to_yaw(t.transform.rotation)
        stamp = time.time()

        if self._prev_x is not None:
            dx = x - self._prev_x
            dy = y - self._prev_y
            dyaw = abs(yaw - self._prev_yaw)

            # Normalize yaw difference
            dyaw = min(dyaw, 2 * math.pi - dyaw)

            dist = math.sqrt(dx * dx + dy * dy)

            # Track maximum smooth delta (normal TF evolution)
            self._max_delta_pos = max(self._max_delta_pos, dist)
            self._max_delta_ang = max(self._max_delta_ang, dyaw)

            # A "jump" is a per-frame delta exceeding threshold
            jump_pos = dist
            jump_ang = math.degrees(dyaw)

            if jump_pos > self._max_jump_m or jump_ang > self._max_jump_deg:
                self._max_jump_pos = max(self._max_jump_pos, jump_pos)
                self._max_jump_ang = max(self._max_jump_ang, jump_ang)
                verdict = "FAIL"
            else:
                verdict = "PASS" if self._last_verdict != "WAIT" else "WAIT"

            self._last_verdict = verdict

            msg = String()
            msg.data = (
                f"[POSE_JUMP] verdict={verdict} "
                f"jump_pos={jump_pos*100:.1f}cm/{self._max_jump_m*100:.0f}cm "
                f"jump_ang={jump_ang:.1f}°/{self._max_jump_deg:.0f}° "
                f"max_smooth_delta={self._max_delta_pos*100:.1f}cm "
                f"max_jump={self._max_jump_pos*100:.1f}cm/{math.degrees(self._max_jump_ang):.1f}°"
            )
            self._pub.publish(msg)

        else:
            self._last_verdict = "WAIT"

        self._prev_x = x
        self._prev_y = y
        self._prev_yaw = yaw
        self._prev_stamp = stamp


def main():
    rclpy.init()
    node = PoseJumpDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
