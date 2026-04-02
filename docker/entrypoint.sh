#!/bin/bash
set -e

_configure_ros_logging() {
    local requested_ros_home="${ROS_HOME:-$HOME/.ros}"
    local active_ros_home="$requested_ros_home"

    mkdir -p "$requested_ros_home" 2>/dev/null || true
    if [ ! -w "$requested_ros_home" ]; then
        active_ros_home="/tmp/ros"
        mkdir -p "$active_ros_home"
        echo "[entrypoint] WARN: ${requested_ros_home} is not writable; using ${active_ros_home}"
    fi

    export ROS_HOME="$active_ros_home"
    export ROS_LOG_DIR="${ROS_LOG_DIR:-${ROS_HOME}/log}"
    mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
}

_workspace_root() {
    if [ -d "/home/developer/ws" ]; then
        echo "/home/developer/ws"
        return
    fi
    if [ -d "/app/ros_ws" ]; then
        echo "/app/ros_ws"
        return
    fi
    echo ""
}

_repo_root() {
    local ws_root
    ws_root="$(_workspace_root)"
    if [ -n "$ws_root" ] && [ -d "${ws_root}/src/cddp_mpc" ]; then
        echo "${ws_root}/src/cddp_mpc"
        return
    fi
    echo ""
}

_install_dev_shell_helpers() {
    local repo_dir rc_file
    repo_dir="$(_repo_root)"
    rc_file="$HOME/.bashrc"

    if [ -z "$repo_dir" ] || [ ! -f "$rc_file" ]; then
        return
    fi
    if grep -q "CDDP-MPC helper aliases" "$rc_file"; then
        return
    fi

    cat >> "$rc_file" <<EOF

# CDDP-MPC helper aliases
alias cw='cd ${repo_dir}'
alias cb='cd $(_workspace_root) && colcon build --packages-select cddp_mpc --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF'
sb() {
    source /opt/ros/\${ROS_DISTRO}/setup.bash
    if [ -f $(_workspace_root)/install/setup.bash ]; then
        source $(_workspace_root)/install/setup.bash
    fi
}
EOF
}

_source_workspace() {
    local ws_root
    ws_root="$(_workspace_root)"
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    if [ -n "$ws_root" ] && [ -f "${ws_root}/install/setup.bash" ]; then
        source "${ws_root}/install/setup.bash"
    fi
}

_print_environment_banner() {
    echo "=========================================="
    echo "CDDP-MPC Container Environment"
    echo "=========================================="
    echo "ROS Distro: ${ROS_DISTRO}"
    echo "Workspace: $(_workspace_root)"
    echo "Repository: $(_repo_root)"
    echo "PX4 Autopilot: ${PX4_HOME}"
    echo "MicroXRCEAgent: $(which MicroXRCEAgent || echo 'Not found')"
    echo "Gazebo: $(gz sim --version 2>/dev/null || echo 'Not available')"
    echo "ROS_HOME: ${ROS_HOME}"
    echo "ROS_LOG_DIR: ${ROS_LOG_DIR}"
    echo "=========================================="
}

_configure_ros_logging
_install_dev_shell_helpers
_source_workspace

export PX4_HOME=/opt/PX4-Autopilot
export PATH="${PX4_HOME}/Tools:${PATH}"
export GZ_SIM_RESOURCE_PATH="${PX4_HOME}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${PX4_HOME}/build/px4_sitl_default/build_gz:${GZ_SIM_SYSTEM_PLUGIN_PATH}"

_print_environment_banner

exec "$@"
