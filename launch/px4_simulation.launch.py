#!/usr/bin/env python3
"""Launch PX4 SITL simulation with Gazebo, Micro XRCE-DDS, and optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_px4_sitl(context, *args, **kwargs):
    del args, kwargs
    vehicle = LaunchConfiguration("vehicle").perform(context)
    world = LaunchConfiguration("world").perform(context)
    return [
        ExecuteProcess(
            cmd=["make", "px4_sitl", vehicle],
            cwd="/opt/PX4-Autopilot",
            additional_env={"PX4_GZ_WORLD": world},
            output="screen",
            name="px4_sitl_gazebo",
            on_exit=LogInfo(msg="PX4 SITL terminated"),
        )
    ]


def generate_launch_description():
    vehicle_arg = DeclareLaunchArgument(
        "vehicle",
        default_value="gz_x500",
        description="PX4 vehicle model to simulate",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="default",
        description="Gazebo world to load",
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
        default_value=PathJoinSubstitution(
            [FindPackageShare("cddp_mpc"), "rviz", "px4_visualizer.rviz"]
        ),
        description="Path to the RViz config used when launch_rviz is true.",
    )

    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_dds_agent",
        on_exit=LogInfo(msg="Micro XRCE-DDS Agent terminated"),
    )

    px4_sitl = OpaqueFunction(function=launch_px4_sitl)
    px4_visualizer = Node(
        package="cddp_mpc",
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
        [
            vehicle_arg,
            world_arg,
            visualizer_arg,
            rviz_arg,
            rviz_config_arg,
            LogInfo(msg="Starting PX4 SITL Simulation..."),
            micro_xrce_agent,
            px4_sitl,
            px4_visualizer,
            rviz,
        ]
    )
