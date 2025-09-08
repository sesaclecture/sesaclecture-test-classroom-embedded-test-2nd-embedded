#!/usr/bin/env python3
# pylint: disable=import-error,too-few-public-methods
"""for controlling the turtle in turtlesim using ROS2"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleDriver(Node):
    """turtle driver class"""

    def __init__(self):
        super().__init__("turtle_driver")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.t = 0.0
        self.timer = self.create_timer(0.1, self.on_timer)  # 10Hz

    def on_timer(self):
        """timer callback"""
        self.t += 0.1
        msg = Twist()
        # 예: 원형 경로
        msg.linear.x = 1.5
        msg.angular.z = 1.0
        self.pub.publish(msg)


def main():
    """main function"""
    rclpy.init()
    rclpy.spin(TurtleDriver())
    rclpy.shutdown()
