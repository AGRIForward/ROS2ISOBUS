#!/usr/bin/env python3
"""
Launch ROS2ISOBUS nodes for VT client use:
 - CAN bridge
 - Address manager
 - Diagnostics
 - VT client
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    common_params = os.path.join(
        get_package_share_directory("ros2_isobus"),
        "config",
        "all_nodes_vt_params.yaml",
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
            executable="vt_client_node",
            name="vt_client_node",
            output="screen",
            parameters=[common_params],
        ),
    ]

    return LaunchDescription(nodes)
