#!/usr/bin/env python3
"""
adam_test_tools / publish_goal.py
Publish navigation goals for Nav2 testing during mapping or localization mode.

Publishes goals via the /navigate_to_pose action (nav2_msgs/action/NavigateToPose),
which is the standard Nav2 goal interface used by the BT navigator.

Usage:
  # Single goal (x y [yaw]):
  ros2 run adam_test_tools publish_goal 5.0 3.0
  ros2 run adam_test_tools publish_goal 5.0 3.0 1.57

  # Multi-goal sequence (--goals accepts x y yaw triplets):
  ros2 run adam_test_tools publish_goal --goals 1.0 0.5 0.0  5.0 3.0 1.57  -1.0 2.0 -0.78

  # Custom frame_id (default: map):
  ros2 run adam_test_tools publish_goal 5.0 3.0 --frame-id odom

Notes:
  - Goals are sent sequentially; the next goal is sent only after the
    previous one completes or fails.
  - Yaw is in radians (0 = +X axis, pi/2 = +Y axis).
  - The action server at /navigate_to_pose must be available (Nav2 BT
    navigator running).

Designed for SPEC 02 navigation testing.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


def quaternion_from_yaw(yaw: float):
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w).

    Uses only the yaw component (roll = pitch = 0) for 2D planar goals.
    """
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavGoalPublisher(Node):
    """Action client node that sends NavigateToPose goals to Nav2.

    Connects to the /navigate_to_pose action server and sends one or more
     goals sequentially, waiting for each to complete before proceeding.
    """

    def __init__(self):
        super().__init__("test_publish_goal")
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._goal_handle = None
        self._result_future = None
        self._feedback_received = False

    def send_goal(self, x: float, y: float, yaw: float, frame_id: str = "map"):
        """Send a single navigation goal and block until it completes.

        Args:
            x: X coordinate in frame_id.
            y: Y coordinate in frame_id.
            yaw: Orientation yaw in radians.
            frame_id: TF frame for the goal pose (default 'map').

        Returns:
            True if the goal completed successfully, False otherwise.
        """
        # --- wait for the action server -----------------------------------
        self.get_logger().info(f"Waiting for /navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available after 5 s")
            return False

        # --- build and send the goal --------------------------------------
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg) "
            f"in frame '{frame_id}'"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        if not send_goal_future.result() or not send_goal_future.result().accepted:
            self.get_logger().error("Goal was rejected by the action server")
            return False

        self._goal_handle = send_goal_future.result()
        self.get_logger().info("Goal accepted, tracking progress...")

        # Wait for result
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future)

        result = self._result_future.result()
        if result is None:
            self.get_logger().error("Goal result was empty")
            return False

        status = result.status
        # Status codes from action_msgs/msg/GoalStatus:
        #   4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info("Goal SUCCEEDED -- navigation target reached")
            return True
        elif status == 5:
            self.get_logger().warning("Goal CANCELED")
            return False
        elif status == 6:
            self.get_logger().warning("Goal ABORTED -- navigation failed")
            return False
        else:
            self.get_logger().warning(f"Goal finished with unexpected status code: {status}")
            return False

    def _feedback_callback(self, feedback_msg):
        """Log feedback from the action server (only once per goal to avoid spam)."""
        feedback = feedback_msg.feedback
        if not self._feedback_received:
            self.get_logger().info(
                f"Navigating... {feedback.distance_remaining:.2f} m remaining"
            )
            self._feedback_received = True

        # Update the flag on each new goal
        self._feedback_received = True


def parse_goal_triplets(values):
    """Parse a flat list of floats into (x, y, yaw) triplets.

    Args:
        values: Flat list of floats, e.g. [x1, y1, y1_yaw, x2, y2, y2_yaw, ...]

    Returns:
        List of (x, y, yaw) tuples.
    """
    triplets = []
    for i in range(0, len(values), 3):
        if i + 2 >= len(values):
            print(
                f"Warning: ignoring trailing value(s) at index {i}, "
                f"--goals expects x y yaw triplets",
                file=sys.stderr,
            )
            break
        triplets.append((values[i], values[i + 1], values[i + 2]))
    return triplets


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Publish navigation goals for Nav2 testing via the "
            "/navigate_to_pose action."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  %(prog)s 5.0 3.0            # single goal (yaw=0)\n"
            "  %(prog)s 5.0 3.0 1.57       # single goal with yaw\n"
            "  %(prog)s --goals 0 0 0  5 3 1.57  -1 2 -0.78\n"
            "                                # three-goal sequence\n"
            "  %(prog)s 5 3 --frame-id odom  # custom coordinate frame\n"
        ),
    )

    # Positional: single goal mode
    parser.add_argument(
        "x", type=float, nargs="?",
        help="X coordinate (single goal mode)",
    )
    parser.add_argument(
        "y", type=float, nargs="?",
        help="Y coordinate (single goal mode)",
    )
    parser.add_argument(
        "yaw", type=float, nargs="?", default=0.0,
        help="Yaw angle in radians (single goal mode, default: 0)",
    )

    # Named: multi-goal mode and options
    parser.add_argument(
        "--goals", type=float, nargs="+", metavar="X Y YAW",
        help="Multi-goal sequence: space-separated x y yaw triplets",
    )
    parser.add_argument(
        "--frame-id", default="map",
        help="Coordinate frame for goals (default: map)",
    )

    args = parser.parse_args()

    # --- resolve goals ----------------------------------------------------
    goals = []

    if args.goals is not None:
        # Multi-goal mode
        goals = parse_goal_triplets(args.goals)
        if not goals:
            print("Error: --goals requires at least one x y yaw triplet", file=sys.stderr)
            sys.exit(1)
    elif args.x is not None and args.y is not None:
        # Single goal mode (yaw defaults to 0.0)
        goals.append((args.x, args.y, args.yaw if args.yaw is not None else 0.0))
    else:
        parser.print_help()
        print(
            "\nError: provide either 'x y [yaw]' or '--goals x y yaw ...'",
            file=sys.stderr,
        )
        sys.exit(1)

    # --- execute ----------------------------------------------------------
    rclpy.init()

    node = NavGoalPublisher()

    success_count = 0
    for idx, (x, y, yaw) in enumerate(goals):
        print(f"\n{'='*60}")
        print(f" Goal {idx + 1}/{len(goals)}:  x={x:.2f}  y={y:.2f}  "
              f"yaw={yaw:.3f} rad ({math.degrees(yaw):.1f} deg)")
        print(f"{'='*60}")

        ok = node.send_goal(x, y, yaw, frame_id=args.frame_id)
        if ok:
            success_count += 1
            print(f"  -> Goal {idx + 1} SUCCEEDED")
        else:
            print(f"  -> Goal {idx + 1} FAILED")
            # Stop the sequence on failure
            break

    node.destroy_node()
    rclpy.shutdown()

    print(f"\n{'='*60}")
    print(f" Finished: {success_count}/{len(goals)} goals succeeded")
    print(f"{'='*60}")

    sys.exit(0 if success_count == len(goals) else 1)


if __name__ == "__main__":
    main()
