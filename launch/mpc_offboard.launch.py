#!/usr/bin/env python3
"""Launch the cddp_mpc PX4 offboard MPC node with YAML parameters."""

from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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

    return PathJoinSubstitution([FindPackageShare(package_name), "config", filename])


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


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _clean_namespace(value: str) -> str:
    return value.strip().strip("/")


def _normalize_prefix(value: str) -> str:
    value = value.strip()
    if not value:
        return ""
    if value != "/":
        value = value.rstrip("/")
    return value


def _default_prefix(namespace: str, explicit_prefix: str, leaf: str) -> str:
    explicit_prefix = _normalize_prefix(explicit_prefix)
    if explicit_prefix:
        return explicit_prefix
    if namespace:
        return f"/{namespace}/{leaf}"
    return f"/{leaf}"


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
    package_name = "cddp_mpc"
    namespace = _clean_namespace(LaunchConfiguration("namespace").perform(context))
    fmu_prefix = _default_prefix(
        namespace, LaunchConfiguration("fmu_prefix").perform(context), "fmu"
    )
    controller_prefix = _default_prefix(
        namespace, LaunchConfiguration("controller_prefix").perform(context), "cddp_mpc"
    )
    visualizer_prefix = _default_prefix(
        namespace, LaunchConfiguration("visualizer_prefix").perform(context), "px4_visualizer"
    )
    params_overlay = LaunchConfiguration("params_overlay").perform(context).strip()
    launch_visualizer = _as_bool(LaunchConfiguration("launch_visualizer").perform(context))
    launch_rviz = _as_bool(LaunchConfiguration("launch_rviz").perform(context))

    controller_parameters = [LaunchConfiguration("params_file")]
    if params_overlay:
        controller_parameters.append(params_overlay)
    controller_parameters.append(
        {
            "fmu_prefix": fmu_prefix,
            "controller_prefix": controller_prefix,
            "goal_pose_topic": f"{controller_prefix}/goal_pose",
            "target_system": int(LaunchConfiguration("target_system").perform(context)),
            "target_component": int(LaunchConfiguration("target_component").perform(context)),
            "source_system": int(LaunchConfiguration("source_system").perform(context)),
            "source_component": int(LaunchConfiguration("source_component").perform(context)),
        }
    )

    actions = [
        Node(
            package=package_name,
            executable="px4_mpc_node",
            namespace=namespace,
            name="cddp_mpc",
            output="screen",
            parameters=controller_parameters,
        )
    ]

    if launch_visualizer or launch_rviz:
        actions.append(
            Node(
                package=package_name,
                executable="px4_visualizer",
                namespace=namespace,
                name="px4_visualizer",
                output="screen",
                parameters=[
                    {
                        "fmu_prefix": fmu_prefix,
                        "controller_prefix": controller_prefix,
                        "visualizer_prefix": visualizer_prefix,
                    }
                ],
            )
        )

    if launch_rviz:
        rviz_config = _rewrite_rviz_config(
            LaunchConfiguration("rviz_config").perform(context),
            controller_prefix,
            visualizer_prefix,
        )
        actions.append(
            ExecuteProcess(
                cmd=["rviz2", "-d", rviz_config],
                output="screen",
                name=f"rviz2_{namespace or 'default'}",
            )
        )

    return actions


def generate_launch_description():
    package_name = "cddp_mpc"
    default_params = _resolve_default_params(package_name, "mpc_sitl.yaml")
    default_rviz_config = _resolve_share_file(package_name, "rviz", "px4_visualizer.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to ROS 2 parameter YAML for the cddp_mpc node",
            ),
            DeclareLaunchArgument(
                "params_overlay",
                default_value="",
                description="Optional overlay parameter YAML applied after params_file.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace for this controller instance.",
            ),
            DeclareLaunchArgument(
                "fmu_prefix",
                default_value="",
                description="Override the PX4 topic prefix. Defaults to /<namespace>/fmu or /fmu.",
            ),
            DeclareLaunchArgument(
                "controller_prefix",
                default_value="",
                description=(
                    "Override the controller topic prefix. "
                    "Defaults to /<namespace>/cddp_mpc or /cddp_mpc."
                ),
            ),
            DeclareLaunchArgument(
                "visualizer_prefix",
                default_value="",
                description=(
                    "Override the visualizer topic prefix. "
                    "Defaults to /<namespace>/px4_visualizer or /px4_visualizer."
                ),
            ),
            DeclareLaunchArgument("target_system", default_value="1"),
            DeclareLaunchArgument("target_component", default_value="1"),
            DeclareLaunchArgument("source_system", default_value="1"),
            DeclareLaunchArgument("source_component", default_value="1"),
            DeclareLaunchArgument(
                "launch_visualizer",
                default_value="false",
                description="Launch the px4_visualizer ROS helper node.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz with a prefix-adjusted px4_visualizer display config.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Path to the RViz config used when launch_rviz is true.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
