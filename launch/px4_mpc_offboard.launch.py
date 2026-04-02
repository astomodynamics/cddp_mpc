#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "cddp_mpc"
    default_params = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "mpc_sitl.yaml"]
    )

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to the PX4 MPC parameter YAML file.",
    )

    px4_mpc_node = Node(
        package=package_name,
        executable="px4_mpc_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_arg, px4_mpc_node])
