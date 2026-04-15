#!/usr/bin/env python3
"""Launch multiple cddp_mpc offboard controller instances."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _build_actions(context, *args, **kwargs):
    del args, kwargs
    package_share = Path(FindPackageShare("cddp_mpc").perform(context))
    offboard_launch = package_share / "launch" / "mpc_offboard.launch.py"
    params_file = LaunchConfiguration("params_file").perform(context)
    vehicle_count = int(LaunchConfiguration("vehicle_count").perform(context))
    instance_start = int(LaunchConfiguration("instance_start").perform(context))
    launch_visualizer = LaunchConfiguration("launch_visualizer").perform(context)
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context))
    rviz_instance = int(LaunchConfiguration("rviz_instance").perform(context))

    actions = [
        LogInfo(
            msg=(
                f"Launching {vehicle_count} offboard controller instances "
                f"starting at PX4 instance {instance_start}."
            )
        )
    ]

    for offset in range(vehicle_count):
        instance = instance_start + offset
        namespace = f"px4_{instance}"
        overlay_path = package_share / "config" / "fleet" / f"{namespace}.yaml"
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(offboard_launch)),
                launch_arguments={
                    "namespace": namespace,
                    "params_file": params_file,
                    "params_overlay": str(overlay_path) if overlay_path.is_file() else "",
                    "fmu_prefix": f"/{namespace}/fmu",
                    "controller_prefix": f"/{namespace}/cddp_mpc",
                    "visualizer_prefix": f"/{namespace}/px4_visualizer",
                    "target_system": str(instance + 1),
                    "target_component": "1",
                    "source_system": str(instance + 1),
                    "source_component": "1",
                    "launch_visualizer": launch_visualizer,
                    "launch_rviz": "true" if launch_rviz and instance == rviz_instance else "false",
                }.items(),
            )
        )

    return actions


def generate_launch_description():
    package_share = FindPackageShare("cddp_mpc")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([package_share, "config", "mpc_sitl.yaml"]),
                description="Base parameter YAML shared by all controller instances.",
            ),
            DeclareLaunchArgument("vehicle_count", default_value="2"),
            DeclareLaunchArgument("instance_start", default_value="0"),
            DeclareLaunchArgument("launch_visualizer", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_instance", default_value="0"),
            OpaqueFunction(function=_build_actions),
        ]
    )
