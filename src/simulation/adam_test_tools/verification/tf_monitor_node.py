#!/usr/bin/env python3
"""
adam_test_tools / verification / tf_monitor_node.py
Monitor TF broadcast frequency and loss rate.

Topics:
  /test_verdict/tf_health (std_msgs/msg/String)
    Publishes PASS / FAIL verdict with stats.

Gate: SPEC 02 Gate 2 — odom→base_link ≥ 50Hz, loss ≤ 0.1%
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import collections
import time


class TFMonitorNode(Node):

    def __init__(self):
        super().__init__("test_tf_monitor")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "base_link")
        self.declare_parameter("min_hz", 50.0)
        self.declare_parameter("max_loss_rate", 0.001)
        self.declare_parameter("window_seconds", 5.0)

        self._target = self.get_parameter("target_frame").value
        self._source = self.get_parameter("source_frame").value
        self._min_hz = self.get_parameter("min_hz").value
        self._max_loss = self.get_parameter("max_loss_rate").value
        self._window = self.get_parameter("window_seconds").value

        self._timestamps = collections.deque()
        self._total_expected = 0
        self._total_received = 0
        self._last_verdict = "WAIT"

        self._sub = self.create_subscription(TFMessage, "/tf", self._tf_callback, 10)
        self._pub = self.create_publisher(String, "/test_verdict/tf_health", 10)
        self._timer = self.create_timer(1.0, self._evaluate)

        self.get_logger().info(
            f"TFMonitor: monitoring {self._target}→{self._source}, "
            f"threshold ≥{self._min_hz}Hz, loss ≤{self._max_loss*100}%"
        )

    def _tf_callback(self, msg):
        now = time.time()
        for transform in msg.transforms:
            if (transform.header.frame_id == self._target
                    and transform.child_frame_id == self._source):
                self._timestamps.append(now)
                self._total_received += 1

        # Estimate expected frames (50Hz = 0.02s per frame)
        self._total_expected = int((now - self._timestamps[0]) * self._min_hz) if self._timestamps else 0

        # Purge old entries
        cutoff = now - self._window
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()

    def _evaluate(self):
        if len(self._timestamps) < 2:
            verdict = "WAIT"
            hz = 0.0
            loss = 0.0
        else:
            elapsed = self._timestamps[-1] - self._timestamps[0]
            if elapsed > 0:
                hz = (len(self._timestamps) - 1) / elapsed
            else:
                hz = 0.0

            if self._total_expected > 0:
                loss = 1.0 - (self._total_received / self._total_expected)
            else:
                loss = 0.0

            if hz >= self._min_hz and loss <= self._max_loss:
                verdict = "PASS"
            else:
                verdict = "FAIL"

            self._last_verdict = verdict

        msg = String()
        msg.data = (
            f"[TF_MONITOR] verdict={verdict} "
            f"frame={self._target}→{self._source} "
            f"hz={hz:.1f}/{self._min_hz} "
            f"loss={loss*100:.2f}%/{self._max_loss*100:.2f}% "
            f"samples={len(self._timestamps)}"
        )
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = TFMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
