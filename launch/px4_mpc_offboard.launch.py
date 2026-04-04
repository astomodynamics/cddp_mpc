#!/usr/bin/env python3
"""Backward-compatible alias for the main offboard MPC launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("cddp_mpc")
    default_params = PathJoinSubstitution([package_share, "config", "mpc_sitl.yaml"])
    offboard_launch = PathJoinSubstitution([package_share, "launch", "mpc_offboard.launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to ROS 2 parameter YAML for px4_mpc_node",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(offboard_launch),
                launch_arguments={"params_file": LaunchConfiguration("params_file")}.items(),
            )
        ]
    )
