#!/usr/bin/env python3
"""Minimal launch file for PX4 SITL simulation."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_px4_sitl(context, *args, **kwargs):
    del args, kwargs
    vehicle = LaunchConfiguration("vehicle").perform(context)
    px4_dir = os.environ.get("PX4_HOME", "/opt/PX4-Autopilot")
    return [
        ExecuteProcess(
            cmd=["bash", "-c", f"cd {px4_dir} && make px4_sitl {vehicle}"],
            output="screen",
            name="px4_sitl_gazebo",
            additional_env={"PX4_HOME": px4_dir},
        )
    ]


def generate_launch_description():
    vehicle_arg = DeclareLaunchArgument(
        "vehicle",
        default_value="gz_x500",
        description="PX4 vehicle model to simulate",
    )

    px4_dir = os.environ.get("PX4_HOME", "/opt/PX4-Autopilot")

    micro_xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_dds_agent",
    )

    px4_sitl = OpaqueFunction(function=launch_px4_sitl)

    return LaunchDescription(
        [
            vehicle_arg,
            LogInfo(msg="=== Starting PX4 SITL Simulation ==="),
            LogInfo(msg=f"PX4 Directory: {px4_dir}"),
            micro_xrce_agent,
            px4_sitl,
        ]
    )
