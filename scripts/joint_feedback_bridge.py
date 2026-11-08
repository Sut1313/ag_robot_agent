#!/usr/bin/env python3
"""Republish /joint_states as /joint_states_feedback for the grasp executor.

The real grasp executor waits for gripper feedback on /joint_states_feedback,
which the simulation does not publish. This bridge supplies the same
JointState messages so the simulation can exercise the real execution path.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FeedbackBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_feedback_bridge")
        self.publisher = self.create_publisher(JointState, "/joint_states_feedback", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        self.publisher.publish(msg)


def main() -> int:
    rclpy.init()
    node = FeedbackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
