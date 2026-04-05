#!/usr/bin/env python3
"""Launch the cddp_mpc PX4 offboard MPC node with YAML parameters."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_default_params(package_name: str, filename: str):
    current_file = Path(__file__).resolve()
    for ancestor in current_file.parents:
        if ancestor.name == "src":
            candidate = ancestor / package_name / "config" / filename
            if candidate.is_file():
                return str(candidate)
        if ancestor.name == "install":
            candidate = ancestor.parent / "src" / package_name / "config" / filename
            if candidate.is_file():
                return str(candidate)

    return PathJoinSubstitution(
        [FindPackageShare(package_name), "config", filename]
    )


def generate_launch_description():
    package_name = "cddp_mpc"
    default_params = _resolve_default_params(package_name, "mpc_sitl.yaml")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to ROS 2 parameter YAML for the cddp_mpc node",
    )

    px4_mpc_node = Node(
        package=package_name,
        executable="px4_mpc_node",
        name="cddp_mpc",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, px4_mpc_node])
