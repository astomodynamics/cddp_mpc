#!/usr/bin/env python3
"""Launch PX4 SITL simulation with Gazebo and Micro XRCE-DDS."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_px4_sitl(context, *args, **kwargs):
    del args, kwargs
    vehicle = LaunchConfiguration("vehicle").perform(context)
    world = LaunchConfiguration("world").perform(context)
    return [
        ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                f"cd /opt/PX4-Autopilot && PX4_GZ_WORLD={world} make px4_sitl {vehicle}",
            ],
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

    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_dds_agent",
        on_exit=LogInfo(msg="Micro XRCE-DDS Agent terminated"),
    )

    px4_sitl = OpaqueFunction(function=launch_px4_sitl)

    return LaunchDescription(
        [
            vehicle_arg,
            world_arg,
            LogInfo(msg="Starting PX4 SITL Simulation..."),
            micro_xrce_agent,
            px4_sitl,
        ]
    )
