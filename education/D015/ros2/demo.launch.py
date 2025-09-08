#!/usr/bin/env python3
# pylint: disable=import-error
"""Launch file to start multiple ROS2 nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""
    return LaunchDescription(
        [
            Node(
                package="my_pkg",
                executable="turtle_driver",
                name="turtle_driver",
            ),
            Node(package="my_pkg", executable="talker", name="talker"),
            Node(package="my_pkg", executable="listener", name="listener"),
        ]
    )
