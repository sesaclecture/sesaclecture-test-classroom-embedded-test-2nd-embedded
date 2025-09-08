#!/usr/bin/env python3
# pylint: disable=import-error, too-few-public-methods
"""A simple ROS 2 talker node that publishes messages to a topic."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    """A simple ROS 2 talker node."""

    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(0.5, self.on_timer)  # 2Hz
        self.count = 0

    def on_timer(self):
        """Callback function for the timer."""
        self.count += 1
        msg = String()
        msg.data = f"hello #{self.count}"
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main():
    """Main function to run the talker node."""
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()
