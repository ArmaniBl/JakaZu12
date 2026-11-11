#!/usr/bin/env python3
"""Stamp received Isaac states with wall time when the scene has no /clock."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState


class JointStateClock(Node):
    def __init__(self):
        super().__init__("isaac_joint_state_clock")
        self.publisher = self.create_publisher(JointState, "/ur10/joint_states", 10)
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.receive, qos_profile_sensor_data
        )

    def receive(self, message):
        if len(message.name) != len(message.position) or not all(
            math.isfinite(p) for p in message.position
        ):
            self.get_logger().error("Ignoring malformed joint state")
            return
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = JointStateClock()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
