"""Hardware-gated Crazyswarm2 launch with the mandatory safety gateway."""

import os

from ament_index_python.packages import get_package_share_directory
from crazyfly.config_validator import validate_hardware_configuration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_before_hardware(context):
    """Abort before starting upstream nodes unless every hardware guard passes."""
    if LaunchConfiguration("allow_hardware").perform(context).lower() != "true":
        return []
    validate_hardware_configuration(
        fleet_path=LaunchConfiguration("fleet_config_file").perform(context),
        safety_path=LaunchConfiguration("safety_config_file").perform(context),
        motion_capture_path=LaunchConfiguration("motion_capture_yaml_file").perform(context),
    )
    return []


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("crazyfly")
    upstream = get_package_share_directory("crazyflie")
    fleet = os.path.join(share, "config", "crazyflies.yaml")
    safety = os.path.join(share, "config", "safety.yaml")
    motion = os.path.join(share, "config", "motion_capture.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_hardware", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("fleet_config_file", default_value=fleet),
            DeclareLaunchArgument("safety_config_file", default_value=safety),
            DeclareLaunchArgument("motion_capture_yaml_file", default_value=motion),
            OpaqueFunction(function=_validate_before_hardware),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(upstream, "launch", "launch.py")),
                condition=IfCondition(LaunchConfiguration("allow_hardware")),
                launch_arguments={
                    "crazyflies_yaml_file": LaunchConfiguration("fleet_config_file"),
                    "motion_capture_yaml_file": LaunchConfiguration("motion_capture_yaml_file"),
                    "backend": "cpp",
                    "mocap": "True",
                    "teleop": "False",
                    "gui": "False",
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
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
