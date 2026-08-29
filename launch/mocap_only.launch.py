"""Start only OptiTrack reception; never starts a Crazyflie server."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from crazyfly.config_validator import prepare_motion_capture_parameters


def _nodes(context):
    allow_network = LaunchConfiguration("allow_network").perform(context).lower() == "true"
    if not allow_network:
        return []
    motion_path = LaunchConfiguration("motion_capture_yaml_file").perform(context)
    fleet_path = LaunchConfiguration("fleet_config_file").perform(context)
    parameters = prepare_motion_capture_parameters(
        motion_path,
        fleet_path,
        require_motive=True,
    )
    return [
        Node(
            package="motion_capture_tracking",
            executable="motion_capture_tracking_node",
            name="motion_capture_tracking",
            output="screen",
            parameters=[parameters],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("crazyfly")
    default_motion = os.path.join(share, "config", "motion_capture.yaml")
    default_fleet = os.path.join(share, "config", "crazyflies.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_network", default_value="false"),
            DeclareLaunchArgument("fleet_config_file", default_value=default_fleet),
            DeclareLaunchArgument("motion_capture_yaml_file", default_value=default_motion),
            OpaqueFunction(function=_nodes),
        ]
    )
