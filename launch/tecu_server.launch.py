#!/usr/bin/env python3
"""Start the tractor-side ISO 11783 Class 1, 2 or 3 T-ECU server stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start CAN bridge, AddressManager and the configurable T-ECU server."""
    config = (
        Path(get_package_share_directory("ros2_isobus"))
        / "config"
        / "tecu_server.yaml"
    )
    interface = LaunchConfiguration("can_interface")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface used by the T-ECU server",
            ),
            Node(
                package="ros2_isobus",
                executable="can_bridge_node",
                name="can_bridge_node",
                output="screen",
                parameters=[str(config), {"interface": interface}],
            ),
            Node(
                package="ros2_isobus",
                executable="address_manager_node",
                name="address_manager_node",
                output="screen",
                parameters=[str(config)],
            ),
            Node(
                package="ros2_isobus",
                executable="tecu_server_node",
                name="tecu_server_node",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
