# CDDP MPC Package

A ROS 2 PX4 offboard MPC package built on `cddp-cpp`.

## Description

This package provides a single PX4-oriented controller path:

- `px4_mpc_node` uses the `QuadrotorRate` model from [CDDP C++](https://github.com/astomodynamics/cddp-cpp)
- It subscribes to PX4 state topics and publishes `VehicleRatesSetpoint`, `OffboardControlMode`, and `VehicleCommand`
- It supports SITL defaults and a hardware-gated launch mode through separate YAML files

## Prerequisites

- ROS 2 (tested on ROS 2 Humble)
- C++17 or later
- CMake 3.8 or later
- `px4_msgs` for PX4 offboard integration
- [Optional] local sibling checkout of `cddp-cpp` at `../cddp-cpp` for offline builds

## Installation

### Build cddp_mpc

1. Create a ROS workspace and navigate to it:
```bash
mkdir -p ros_ws/src
cd ros_ws/src
```

2. Create the source directory and clone the repository:
```bash
git clone https://github.com/astomodynamics/cddp_mpc
```

3. Return to the workspace root and build the package:
```bash
cd ..
colcon build --packages-select cddp_mpc
```

### Optional local `cddp-cpp`

If you want offline builds or tighter local iteration, place `cddp-cpp` next to this repo:

```bash
cd src
git clone https://github.com/astomodynamics/cddp-cpp
```

`cddp_mpc` will prefer `../cddp-cpp` during CMake configure.

## Usage

After building the package, source your ROS workspace:
```bash
source ~/ros_ws/install/setup.bash
```

### Docker Workflow

The repo now ships the same broad PX4 dev/sim container capability as `naiten-mpc`, adapted for `cddp_mpc`:

- Multi-stage Docker image in [`docker/Dockerfile`](docker/Dockerfile)
- Dev and sim services in [`docker-compose.yml`](docker-compose.yml)
- PX4 SITL tooling, Gazebo Garden, Micro XRCE-DDS Agent, and a prebuilt `px4_msgs` workspace overlay

Build the development image:

```bash
docker compose build cddp-mpc-dev
```

Open a development shell:

```bash
docker compose run --rm cddp-mpc-dev bash
```

Open the lighter simulation image:

```bash
docker compose run --rm cddp-mpc-sim bash
```

Inside either container, the ROS and PX4 environment is already sourced. In the dev container, `cw` jumps to the repo and `cb` rebuilds `cddp_mpc`.

### Running the PX4 Controller

The package ships a PX4-focused offboard node and launch/config layout modeled after `naiten-mpc`, but using the `cddp-cpp` quadrotor-rate solver.

1. Start PX4 SITL, Micro XRCE-DDS, and your simulator as usual.
2. Launch the controller:

```bash
ros2 launch cddp_mpc px4_mpc_offboard.launch.py
```

3. For hardware validation, use the hardware-oriented parameter file:

```bash
ros2 launch cddp_mpc px4_mpc_offboard.launch.py \
  params_file:=$(ros2 pkg prefix cddp_mpc)/share/cddp_mpc/config/mpc_hardware.yaml
```

4. If `auto_start_sequence` is `false`, start the mission manually:

```bash
ros2 service call /cddp_mpc/start_mission std_srvs/srv/Trigger {}
```

The PX4 node prefers `VehicleOdometry` when it is available and frame-valid, and falls back to local position plus attitude otherwise.

The PX4 node subscribes to:

- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_status`

and publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/vehicle_rates_setpoint`
- `/fmu/in/vehicle_command`

Parameter files are in [`config/mpc_sitl.yaml`](config/mpc_sitl.yaml) and [`config/mpc_hardware.yaml`](config/mpc_hardware.yaml).

## Contributing
Contributions are welcome! Please feel free to submit pull requests.

## Citing
If you use this work in an academic context, please cite this repository.

## License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
