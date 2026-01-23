#!/usr/bin/env python3
"""
Launch all available ROS2ISOBUS nodes:
 - CAN bridge
 - Address manager
 - NMEA2000 client
 - TECU client (Class3 by default)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="ros2_isobus",
            executable="can_bridge_node",
            name="can_bridge_node",
            output="screen",
        ),
        Node(
            package="ros2_isobus",
            executable="address_manager_node",
            name="address_manager_node",
            output="screen",
        ),
        Node(
            package="ros2_isobus",
            executable="nmea2000_node",
            name="nmea2000_node",
            output="screen",
        ),
        Node(
            package="ros2_isobus",
            executable="tecu_node",
            name="tecu_class3_node",
            output="screen",
            arguments=["--class3"],
        ),
    ]

    return LaunchDescription(nodes)
