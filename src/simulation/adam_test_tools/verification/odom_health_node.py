#!/usr/bin/env python3
"""
adam_test_tools / verification / odom_health_node.py
Monitor covariance of SLAM pose topics for degradation detection.

Topics:
  /slam_pose/cartographer (Odometry)
  /slam_pose/fast_lio     (Odometry)
  /slam_pose/orbslam3     (Odometry)
  /slam_pose/droid_slam   (Odometry)

  /test_verdict/odom_health (std_msgs/msg/String)
    Publishes HEALTHY / DEGRADED / LOST verdict.

Gates: SPEC 03 Gate 1 (FAST-LIO2), SPEC 04 Gate 3 (dark room)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time


class OdomHealthNode(Node):

    def __init__(self):
        super().__init__("test_odom_health")
        self.declare_parameter("cov_threshold_warn", 0.5)
        self.declare_parameter("cov_threshold_fail", 1.0)
        self.declare_parameter("topics", [
            "/slam_pose/cartographer",
            "/slam_pose/fast_lio",
            "/slam_pose/orbslam3",
            "/slam_pose/droid_slam",
        ])

        self._warn_thresh = self.get_parameter("cov_threshold_warn").value
        self._fail_thresh = self.get_parameter("cov_threshold_fail").value
        topics = self.get_parameter("topics").value

        self._states = {}  # topic -> {cov_max, last_seen, status}
        for t in topics:
            self._states[t] = {"cov_max": 0.0, "last_seen": 0.0, "status": "UNKNOWN"}
            self.create_subscription(Odometry, t, lambda msg, topic=t: self._odom_cb(msg, topic), 10)

        self._pub = self.create_publisher(String, "/test_verdict/odom_health", 10)
        self._timer = self.create_timer(1.0, self._evaluate)

        self.get_logger().info(
            f"OdomHealthNode: monitoring {len(topics)} topics, "
            f"warn>{self._warn_thresh} fail>{self._fail_thresh}"
        )

    def _odom_cb(self, msg: Odometry, topic: str):
        cov = msg.pose.covariance
        # Use max diagonal covariance as health indicator
        max_cov = max(abs(cov[i * 6 + i]) for i in range(6))

        self._states[topic] = {
            "cov_max": max_cov,
            "last_seen": time.time(),
            "status": self._classify(max_cov),
        }

    def _classify(self, cov_max: float) -> str:
        if cov_max > self._fail_thresh:
            return "LOST"
        elif cov_max > self._warn_thresh:
            return "DEGRADED"
        return "HEALTHY"

    def _evaluate(self):
        now = time.time()
        lines = ["[ODOM_HEALTH]"]
        overall_verdict = "HEALTHY"

        for topic, state in self._states.items():
            elapsed = now - state["last_seen"] if state["last_seen"] > 0 else 999

            # Topic not seen for 3s -> LOST
            if elapsed > 3.0:
                status = "LOST"
                cov_str = "---"
            else:
                status = state["status"]
                cov_str = f"{state['cov_max']:.3f}"

            lines.append(f"  {topic.split('/slam_pose/')[-1]}: {status} (cov={cov_str}, since={elapsed:.1f}s)")

            if status == "LOST":
                overall_verdict = "LOST"
            elif status == "DEGRADED" and overall_verdict != "LOST":
                overall_verdict = "DEGRADED"

        msg = String()
        msg.data = " | ".join(lines) + f" | overall={overall_verdict}"
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = OdomHealthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
