#!/usr/bin/env python3
"""
Launch ROS2ISOBUS nodes with TIM enabled:
 - CAN bridge
 - Address manager
 - Diagnostics (ISO 11783-12 Annex B minimum)
 - NMEA2000 client
 - TECU client (Class2)
 - TIM client

Test panel is intended to be started separately in an interactive terminal.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    common_params = os.path.join(
        get_package_share_directory("ros2_isobus"),
        "config",
        "all_nodes_tim_params.yaml",
    )

    nodes = [
        Node(
            package="ros2_isobus",
            executable="can_bridge_node",
            name="can_bridge_node",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="ros2_isobus",
            executable="address_manager_node",
            name="address_manager_node",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="ros2_isobus",
            executable="diagnostics_node",
            name="diagnostics_node",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="ros2_isobus",
            executable="nmea2000_node",
            name="nmea2000_node",
            output="screen",
            parameters=[common_params],
        ),
        Node(
            package="ros2_isobus",
            executable="tecu_node",
            name="tecu_class2_node",
            output="screen",
            arguments=["--class2"],
            parameters=[common_params],
        ),
        Node(
            package="ros2_isobus",
            executable="tim_client_node",
            name="tim_client_node",
            output="screen",
            parameters=[common_params],
        ),
    ]

    return LaunchDescription(nodes)
