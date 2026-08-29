"""Hardware-gated Crazyswarm2 launch with the mandatory safety gateway."""

import os

from ament_index_python.packages import get_package_share_directory
from crazyfly.config import load_yaml
from crazyfly.config_validator import (
    load_server_parameters,
    prepare_motion_capture_parameters,
    validate_hardware_configuration,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _hardware_nodes(context):
    """Validate every guard, then construct the physical Crazyswarm2 nodes."""
    if LaunchConfiguration("allow_hardware").perform(context).lower() != "true":
        return []
    fleet_path = LaunchConfiguration("fleet_config_file").perform(context)
    safety_path = LaunchConfiguration("safety_config_file").perform(context)
    motion_path = LaunchConfiguration("motion_capture_yaml_file").perform(context)
    server_path = LaunchConfiguration("server_config_file").perform(context)
    validate_hardware_configuration(
        fleet_path=fleet_path,
        safety_path=safety_path,
        motion_capture_path=motion_path,
    )

    upstream = get_package_share_directory("crazyflie")
    motion_parameters = prepare_motion_capture_parameters(
        motion_path,
        fleet_path,
        require_motive=True,
    )
    server_parameters = load_server_parameters(server_path)
    server_parameters["poses_qos_deadline"] = motion_parameters["topics"]["poses"][
        "qos"
    ]["deadline"]
    with open(
        os.path.join(upstream, "urdf", "crazyflie_description.urdf"),
        encoding="utf-8",
    ) as stream:
        server_parameters["robot_description"] = stream.read()

    nodes = [
        Node(
            package="motion_capture_tracking",
            executable="motion_capture_tracking_node",
            name="motion_capture_tracking",
            output="screen",
            parameters=[motion_parameters],
        ),
        Node(
            package="crazyflie",
            executable="crazyflie_server",
            name="crazyflie_server",
            output="screen",
            parameters=[load_yaml(fleet_path), server_parameters],
        ),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", os.path.join(upstream, "config", "config.rviz")],
                parameters=[{"use_sim_time": False}],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("crazyfly")
    fleet = os.path.join(share, "config", "crazyflies.yaml")
    safety = os.path.join(share, "config", "safety.yaml")
    motion = os.path.join(share, "config", "motion_capture.yaml")
    server = os.path.join(share, "config", "server.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_hardware", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("fleet_config_file", default_value=fleet),
            DeclareLaunchArgument("safety_config_file", default_value=safety),
            DeclareLaunchArgument("motion_capture_yaml_file", default_value=motion),
            DeclareLaunchArgument("server_config_file", default_value=server),
            OpaqueFunction(function=_hardware_nodes),
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
