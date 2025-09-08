#!/usr/bin/env python3
"""
A simple ROS2 listener node that subscribes to the 'chatter' topic and
logs received messages.
"""
# pylint: disable=import-error, too-few-public-methods
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    """Listener node that subscribes to the 'chatter' topic."""

    def __init__(self):
        super().__init__("listener")
        self.sub = self.create_subscription(String, "chatter", self.on_msg, 10)

    def on_msg(self, msg: String):
        """Callback function to handle incoming messages."""
        self.get_logger().info(f"I heard: {msg.data}")


def main():
    """Main function to initialize and run the listener node."""
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()
