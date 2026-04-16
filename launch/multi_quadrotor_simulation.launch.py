#!/usr/bin/env python3
"""Launch multiple PX4 SITL instances with Gazebo and optional visualizers."""

from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _rewrite_rviz_config(source_path: str, controller_prefix: str, visualizer_prefix: str) -> str:
    text = Path(source_path).read_text()
    text = text.replace("/cddp_mpc", controller_prefix)
    text = text.replace("/px4_visualizer", visualizer_prefix)
    stem = controller_prefix.strip("/").replace("/", "_") or "default"
    destination = Path(tempfile.gettempdir()) / f"cddp_mpc_{stem}_rviz.rviz"
    destination.write_text(text)
    return str(destination)


def _build_actions(context, *args, **kwargs):
    del args, kwargs
    px4_root = Path(LaunchConfiguration("px4_root").perform(context))
    px4_bin = px4_root / "build" / "px4_sitl_default" / "bin" / "px4"
    vehicle = LaunchConfiguration("vehicle").perform(context)
    world = LaunchConfiguration("world").perform(context)
    px4_autostart = LaunchConfiguration("px4_autostart").perform(context)
    vehicle_count = int(LaunchConfiguration("vehicle_count").perform(context))
    instance_start = int(LaunchConfiguration("instance_start").perform(context))
    instance_spacing_m = float(LaunchConfiguration("instance_spacing_m").perform(context))
    launch_visualizer = _as_bool(LaunchConfiguration("launch_visualizer").perform(context))
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context))
    rviz_instance = int(LaunchConfiguration("rviz_instance").perform(context))

    actions = [
        LogInfo(
            msg=(
                f"Starting {vehicle_count} PX4 SITL instances from {px4_bin} "
                f"with world='{world}' and vehicle='{vehicle}'."
            )
        ),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            output="screen",
            name="micro_xrce_dds_agent",
        ),
    ]

    for offset in range(vehicle_count):
        instance = instance_start + offset
        namespace = f"px4_{instance}"
        env = {
            "PX4_GZ_WORLD": world,
            "PX4_SIM_MODEL": vehicle,
            "PX4_SYS_AUTOSTART": px4_autostart,
            "PX4_UXRCE_DDS_NS": namespace,
        }
        if offset > 0:
            env["PX4_GZ_STANDALONE"] = "1"
            env["PX4_GZ_MODEL_POSE"] = f"0,{offset * instance_spacing_m:.3f},0"

        actions.append(
            ExecuteProcess(
                cmd=[str(px4_bin), "-i", str(instance)],
                cwd=str(px4_root),
                additional_env=env,
                output="screen",
                name=f"px4_sitl_{namespace}",
            )
        )

        if launch_visualizer or (launch_rviz and instance == rviz_instance):
            actions.append(
                Node(
                    package="cddp_mpc",
                    executable="px4_visualizer",
                    namespace=namespace,
                    name="px4_visualizer",
                    output="screen",
                    parameters=[
                        {
                            "fmu_prefix": f"/{namespace}/fmu",
                            "controller_prefix": f"/{namespace}/cddp_mpc",
                            "visualizer_prefix": f"/{namespace}/px4_visualizer",
                        }
                    ],
                )
            )

        if launch_rviz and instance == rviz_instance:
            rviz_config = _rewrite_rviz_config(
                LaunchConfiguration("rviz_config").perform(context),
                f"/{namespace}/cddp_mpc",
                f"/{namespace}/px4_visualizer",
            )
            actions.append(
                ExecuteProcess(
                    cmd=["rviz2", "-d", rviz_config],
                    output="screen",
                    name=f"rviz2_{namespace}",
                )
            )

    return actions


def generate_launch_description():
    package_share = FindPackageShare("cddp_mpc")
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_root", default_value="/opt/PX4-Autopilot"),
            DeclareLaunchArgument("vehicle", default_value="gz_x500"),
            DeclareLaunchArgument("world", default_value="default"),
            DeclareLaunchArgument("px4_autostart", default_value="4001"),
            DeclareLaunchArgument("vehicle_count", default_value="2"),
            DeclareLaunchArgument("instance_start", default_value="0"),
            DeclareLaunchArgument("instance_spacing_m", default_value="2.0"),
            DeclareLaunchArgument("launch_visualizer", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_instance", default_value="0"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([package_share, "rviz", "px4_visualizer.rviz"]),
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
