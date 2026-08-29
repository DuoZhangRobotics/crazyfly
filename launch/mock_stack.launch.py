"""Hardware-free mock OptiTrack, Crazyswarm services, and safety gateway."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("crazyfly")
    fleet = os.path.join(share, "config", "mock_crazyflies.yaml")
    safety = os.path.join(share, "config", "mock_safety.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("fleet_config_file", default_value=fleet),
            DeclareLaunchArgument("safety_config_file", default_value=safety),
            Node(
                package="crazyfly",
                executable="crazyfly_mock_stack",
                name="crazyfly_mock_stack",
                output="screen",
                parameters=[{"fleet_config_file": LaunchConfiguration("fleet_config_file")}],
            ),
            Node(
                package="crazyfly",
                executable="crazyfly_safety_gateway",
                name="crazyfly_safety_gateway",
                output="screen",
                parameters=[
                    {
                        "fleet_config_file": LaunchConfiguration("fleet_config_file"),
                        "safety_config_file": LaunchConfiguration("safety_config_file"),
                    }
                ],
            ),
        ]
    )
