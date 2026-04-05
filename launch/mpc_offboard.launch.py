#!/usr/bin/env python3
"""Launch the cddp_mpc PX4 offboard MPC node with YAML parameters."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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


def _resolve_share_file(package_name: str, directory: str, filename: str):
    current_file = Path(__file__).resolve()
    for ancestor in current_file.parents:
        if ancestor.name == "src":
            candidate = ancestor / package_name / directory / filename
            if candidate.is_file():
                return str(candidate)
        if ancestor.name == "install":
            candidate = ancestor.parent / "src" / package_name / directory / filename
            if candidate.is_file():
                return str(candidate)

    return PathJoinSubstitution([FindPackageShare(package_name), directory, filename])


def generate_launch_description():
    package_name = "cddp_mpc"
    default_params = _resolve_default_params(package_name, "mpc_sitl.yaml")
    default_rviz_config = _resolve_share_file(package_name, "rviz", "px4_visualizer.rviz")

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Path to ROS 2 parameter YAML for the cddp_mpc node",
    )
    visualizer_arg = DeclareLaunchArgument(
        "launch_visualizer",
        default_value="false",
        description="Launch the px4_visualizer ROS helper node.",
    )
    rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz with the px4_visualizer display config.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config used when launch_rviz is true.",
    )

    px4_mpc_node = Node(
        package=package_name,
        executable="px4_mpc_node",
        name="cddp_mpc",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )
    px4_visualizer = Node(
        package=package_name,
        executable="px4_visualizer",
        name="px4_visualizer",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("launch_visualizer"),
                    "'",
                    " == 'true' or ",
                    "'",
                    LaunchConfiguration("launch_rviz"),
                    "'",
                    " == 'true'",
                ]
            )
        ),
    )
    rviz = ExecuteProcess(
        cmd=["rviz2", "-d", LaunchConfiguration("rviz_config")],
        output="screen",
        name="rviz2",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [params_arg, visualizer_arg, rviz_arg, rviz_config_arg, px4_mpc_node, px4_visualizer, rviz]
    )
