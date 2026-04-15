#!/usr/bin/env python3
"""Launch the hardware validation mission sequence for cddp_mpc."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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


def _build_control_actions(context, *args, **kwargs):
    del args, kwargs
    control_mode = LaunchConfiguration("control_mode").perform(context).strip().lower()
    namespace = _clean_namespace(LaunchConfiguration("namespace").perform(context))
    fmu_prefix = _default_prefix(
        namespace, LaunchConfiguration("fmu_prefix").perform(context), "fmu"
    )
    controller_prefix = _default_prefix(
        namespace, LaunchConfiguration("controller_prefix").perform(context), "cddp_mpc"
    )
    start_service = f"{controller_prefix}/start_mission"
    package_share = FindPackageShare("cddp_mpc").perform(context)
    offboard_launch = PathJoinSubstitution([package_share, "launch", "mpc_offboard.launch.py"])

    if control_mode == "offboard":
        return [
            LogInfo(msg="Launching offboard MPC hardware validation node..."),
            LogInfo(
                msg=(
                    "When ready, start the mission with: "
                    f"ros2 service call {start_service} std_srvs/srv/Trigger {{}}"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(offboard_launch),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "namespace": LaunchConfiguration("namespace"),
                    "fmu_prefix": fmu_prefix,
                    "controller_prefix": controller_prefix,
                    "visualizer_prefix": LaunchConfiguration("visualizer_prefix"),
                    "target_system": LaunchConfiguration("target_system"),
                    "target_component": LaunchConfiguration("target_component"),
                    "source_system": LaunchConfiguration("source_system"),
                    "source_component": LaunchConfiguration("source_component"),
                }.items(),
            ),
        ]

    if control_mode == "onboard":
        return [
            LogInfo(
                msg=(
                    "Hardware validation launch is running in onboard monitoring mode; "
                    "no offboard MPC node will be started."
                )
            )
        ]

    raise ValueError(
        f"Unsupported control_mode='{control_mode}'. Expected 'offboard' or 'onboard'."
    )


def _build_validator_notice(context, *args, **kwargs):
    del args, kwargs
    launch_validator = LaunchConfiguration("launch_validator").perform(context).strip().lower()
    if launch_validator not in {"1", "true", "yes", "on"}:
        return []

    namespace = _clean_namespace(LaunchConfiguration("namespace").perform(context))
    fmu_prefix = _default_prefix(
        namespace, LaunchConfiguration("fmu_prefix").perform(context), "fmu"
    )
    controller_prefix = _default_prefix(
        namespace, LaunchConfiguration("controller_prefix").perform(context), "cddp_mpc"
    )
    package_share = FindPackageShare("cddp_mpc").perform(context)
    validator_script = PathJoinSubstitution([package_share, "examples", "validate_takeoff_hover.py"])
    cmd = [
        "python3",
        validator_script,
        "--fmu-prefix",
        fmu_prefix,
        "--controller-prefix",
        controller_prefix,
        "--validation-mode",
        LaunchConfiguration("control_mode").perform(context),
        "--target-z",
        LaunchConfiguration("target_z").perform(context),
        "--settle-tolerance",
        LaunchConfiguration("settle_tolerance").perform(context),
        "--hold-duration-sec",
        LaunchConfiguration("hold_duration_sec").perform(context),
        "--timeout-sec",
        LaunchConfiguration("timeout_sec").perform(context),
        "--min-climb-m",
        LaunchConfiguration("min_climb_m").perform(context),
        "--climb-timeout-sec",
        LaunchConfiguration("climb_timeout_sec").perform(context),
        "--max-downward-speed-mps",
        LaunchConfiguration("max_downward_speed_mps").perform(context),
        "--max-drop-from-start-m",
        LaunchConfiguration("max_drop_from_start_m").perform(context),
        "--max-xy-error-m",
        LaunchConfiguration("max_xy_error_m").perform(context),
        "--max-xy-drift-from-start-m",
        LaunchConfiguration("max_xy_drift_from_start_m").perform(context),
        "--landing-tolerance-m",
        LaunchConfiguration("landing_tolerance_m").perform(context),
    ]

    require_hover_done = LaunchConfiguration("require_hover_done").perform(context).strip().lower()
    require_landing = LaunchConfiguration("require_landing").perform(context).strip().lower()
    shutdown_on_validator_exit = (
        LaunchConfiguration("shutdown_on_validator_exit").perform(context).strip().lower()
    )
    if require_hover_done in {"1", "true", "yes", "on"}:
        cmd.append("--require-hover-done")
    if require_landing in {"1", "true", "yes", "on"}:
        cmd.append("--require-landing")

    on_exit = None
    if shutdown_on_validator_exit in {"1", "true", "yes", "on"}:
        on_exit = [Shutdown(reason="hardware mission validator exited")]

    return [
        LogInfo(msg="Starting hardware mission validator..."),
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            name="hardware_mission_validator",
            additional_env={"PYTHONUNBUFFERED": "1"},
            on_exit=on_exit,
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("cddp_mpc")
    default_params = PathJoinSubstitution([package_share, "config", "mpc_hardware.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the hardware ROS 2 parameter YAML for px4_mpc_node",
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("fmu_prefix", default_value=""),
            DeclareLaunchArgument("controller_prefix", default_value=""),
            DeclareLaunchArgument("visualizer_prefix", default_value=""),
            DeclareLaunchArgument("target_system", default_value="1"),
            DeclareLaunchArgument("target_component", default_value="1"),
            DeclareLaunchArgument("source_system", default_value="1"),
            DeclareLaunchArgument("source_component", default_value="1"),
            DeclareLaunchArgument(
                "control_mode",
                default_value="offboard",
                description="Mission control path: offboard launches px4_mpc_node, onboard is monitor-only.",
            ),
            DeclareLaunchArgument(
                "launch_validator",
                default_value="true",
                description="Launch the hardware mission validator process.",
            ),
            DeclareLaunchArgument(
                "shutdown_on_validator_exit",
                default_value="false",
                description="Shutdown the launch when the validator exits.",
            ),
            DeclareLaunchArgument("target_z", default_value="-3.0"),
            DeclareLaunchArgument("settle_tolerance", default_value="0.3"),
            DeclareLaunchArgument("hold_duration_sec", default_value="20.0"),
            DeclareLaunchArgument("timeout_sec", default_value="120.0"),
            DeclareLaunchArgument("min_climb_m", default_value="0.6"),
            DeclareLaunchArgument("climb_timeout_sec", default_value="25.0"),
            DeclareLaunchArgument("max_downward_speed_mps", default_value="2.5"),
            DeclareLaunchArgument("max_drop_from_start_m", default_value="0.0"),
            DeclareLaunchArgument("max_xy_error_m", default_value="0.0"),
            DeclareLaunchArgument("max_xy_drift_from_start_m", default_value="0.0"),
            DeclareLaunchArgument("require_hover_done", default_value="false"),
            DeclareLaunchArgument("require_landing", default_value="true"),
            DeclareLaunchArgument("landing_tolerance_m", default_value="0.3"),
            LogInfo(msg="Starting hardware validation sequence..."),
            LogInfo(
                msg=(
                    "This launch is ROS-side only; PX4 hardware transport must already be running."
                )
            ),
            OpaqueFunction(function=_build_control_actions),
            OpaqueFunction(function=_build_validator_notice),
        ]
    )
