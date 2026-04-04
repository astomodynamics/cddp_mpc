# CDDP MPC Package

A ROS 2 PX4 offboard MPC package built on `cddp-cpp`.

## Description

This package provides a single PX4-oriented controller path:

- `px4_mpc_node` uses the `Quadrotor` thrust model from [CDDP C++](https://github.com/astomodynamics/cddp-cpp)
- It subscribes to PX4 state topics and publishes `VehicleThrustSetpoint`, `VehicleTorqueSetpoint`, `OffboardControlMode`, and `VehicleCommand`
- It supports SITL defaults and a hardware-gated launch mode through separate YAML files

## Prerequisites

- ROS 2 (tested on ROS 2 Humble)
- C++17 or later
- CMake 3.8 or later
- `px4_msgs` for PX4 offboard integration
- [Optional] local sibling checkout of `cddp-cpp` at `../cddp-cpp` for offline builds
- For Docker: NVIDIA Container Toolkit and X11 access if you want Gazebo GUI

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
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
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

or with the host-side helper:

```bash
./scripts/docker-build.sh
```

Open a development shell:

```bash
docker compose run --rm cddp-mpc-dev bash
```

Recommended persistent workflow:

```bash
./scripts/docker-run.sh bash
```

Open additional terminals into the same running dev container:

```bash
./scripts/docker-exec.sh bash
```

Open the lighter simulation image:

```bash
docker compose run --rm cddp-mpc-sim bash
```

Inside either container, the ROS and PX4 environment is already sourced. In the dev container, `cw` jumps to the repo, `cb` rebuilds `cddp_mpc`, and `sb` sources the active overlay.

Important boundary:

- `docker compose` provides the PX4-ready environment
- it does not currently orchestrate PX4 SITL, Gazebo, Micro XRCE-DDS, and the controller in one launch command
- you still start the simulator stack and the controller explicitly inside the container

### Running PX4 Simulation

The launch sequence now matches `naiten-mpc`:

```bash
# Shell 1 inside the persistent dev container: start PX4 SITL + Gazebo + XRCE-DDS
sb
ros2 launch cddp_mpc px4_simulation.launch.py

# Shell 2 inside the same container: run the CDDP offboard MPC node with SITL defaults
sb
ros2 launch cddp_mpc mpc_offboard.launch.py

# Optional: override the parameter file
ros2 launch cddp_mpc mpc_offboard.launch.py params_file:=/path/to/params.yaml
```

`minimal_simulation.launch.py` is also available for the same CLI-friendly flow, just like in `naiten-mpc`:

```bash
sb
ros2 launch cddp_mpc minimal_simulation.launch.py
```

### Hardware Validation Launch

For a ROS-side hardware validation sequence against an already running PX4 vehicle:

```bash
sb
ros2 launch cddp_mpc hardware_validation.launch.py
```

This uses [config/mpc_hardware.yaml](config/mpc_hardware.yaml) by default and waits in `READY` until the operator explicitly starts the mission:

```bash
ros2 service call /cddp_mpc/start_mission std_srvs/srv/Trigger {}
```

Use monitor-only mode for an external onboard controller:

```bash
sb
ros2 launch cddp_mpc hardware_validation.launch.py control_mode:=onboard
```

Notes:

- `hardware_validation.launch.py` now accepts the same `control_mode:=offboard|onboard` launch pattern as `naiten-mpc`.
- `launch_validator:=true` now launches [`examples/validate_takeoff_hover.py`](examples/validate_takeoff_hover.py) by default, using the same high-level flow as `naiten-mpc`.
- `config/mpc_hardware.yaml` is intentionally close to the SITL config, but it adds the `READY` hold by setting `auto_start_sequence: false`.

### Manual Launch (for debugging)

If you want the underlying processes separately instead of the `ros2 launch` wrappers:

```bash
# Terminal 1: XRCE-DDS bridge
MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL
cd /opt/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 3: Monitor topics
sb
ros2 topic echo /fmu/out/vehicle_odometry
```

### Verify Setup

```bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_odometry
ros2 interface list | grep px4_msgs
```

### Lift-Off + Hover Validation

Use this flow to validate autonomous takeoff and hover driven by the CDDP solver.

```bash
# Terminal 1
sb
ros2 launch cddp_mpc minimal_simulation.launch.py

# Terminal 2
sb
ros2 launch cddp_mpc mpc_offboard.launch.py

# Terminal 3
sb
python examples/validate_takeoff_hover.py \
  --target-z -3.0 \
  --settle-tolerance 0.3 \
  --timeout-sec 90 \
  --min-climb-m 0.6 \
  --climb-timeout-sec 25 \
  --max-downward-speed-mps 2.5
```

Expected result: the script exits `0` and prints `PASS`.

If the validator times out or the vehicle stalls below target altitude, run hover calibration while MPC is active in hover or takeoff:

```bash
sb
python examples/calibrate_hover_mass.py \
  --duration-sec 25 \
  --min-samples 30 \
  --vz-threshold-mps 0.2 \
  --max-thrust-n 20.0
```

Then set the reported `recommended_hover_thrust_n` into `config/mpc_sitl.yaml` or `config/mpc_hardware.yaml` and rebuild with `cb`.

### Standalone Data Collection

For synchronized PX4 dataset capture without package installation, use:

```bash
python standalone/px4_data_collector.py --duration 60 --dt 0.02 --output flight_data.npz
```

This writes synchronized `timestamps`, `positions`, `velocities`, `quaternions`, `angular_velocities`, `accelerations`, and `controls` arrays into a single `.npz` file.

### Stability Tuning

Tune in this order:

1. Fix thrust calibration and normalization.
2. Reduce reference aggressiveness if needed.
3. Smooth the published command behavior.
4. Adjust MPC weights last.

#### 1. Thrust Calibration First

The most important parameters in `config/mpc_sitl.yaml` and `config/mpc_hardware.yaml` are:

- `mass_kg`
- `hover_thrust_n`
- `max_thrust_n`
- `thrust_norm_scale`
- `takeoff_min_thrust_margin_n`

If the vehicle will not lift, overshoots badly, or hovers at the wrong command level, fix the thrust map first.

#### 2. Reduce Reference Aggressiveness

If thrust is roughly correct but the mission is still too aggressive, reduce:

- `takeoff_altitude_m` during early tuning
- `settle_tolerance_m`
- `hover_entry_vz_threshold_mps`
- `hover_entry_rate_threshold_rad_s`
- `hover_entry_dwell_s`

If it falls out of hover too easily, increase:

- `hover_reenter_error_m`
- `hover_reenter_dwell_s`

#### 3. Smooth Command Behavior

The main command-smoothing knobs are:

- `thrust_slew_rate_n_per_s`
- `body_torque_slew_nm_per_s`
- `max_roll_torque_nm`
- `max_pitch_torque_nm`
- `max_yaw_torque_nm`
- `min_flight_thrust_ratio`
- `hover_min_thrust_ratio`

#### 4. Adjust MPC Weights Last

Once thrust and reference behavior look believable:

- increase `vel_weight` and `tilt_weight` for a calmer vehicle
- increase `rate_weight` to discourage aggressive body-torque usage
- increase `terminal_pos_weight` if the vehicle stays too loose near target
- reduce `pos_weight` slightly if position tracking drives excessive aggression

### TUI Monitoring

For live terminal monitoring while the validator runs:

```bash
watch -n 0.5 'ros2 topic echo --once /cddp_mpc/status'
watch -n 0.5 'ros2 topic echo --once /fmu/out/vehicle_status'
watch -n 0.5 'ros2 topic echo --once /fmu/out/vehicle_odometry'
watch -n 1.0 'ros2 topic hz /fmu/in/vehicle_thrust_setpoint'
watch -n 1.0 'ros2 topic hz /fmu/in/vehicle_torque_setpoint'
```

Expected mode progression in `/cddp_mpc/status`: `INIT` -> `TAKEOFF` -> `HOVER` -> `LAND` -> `LAND_DONE`.

Available launch files:

- `ros2 launch cddp_mpc px4_simulation.launch.py`
- `ros2 launch cddp_mpc minimal_simulation.launch.py`
- `ros2 launch cddp_mpc mpc_offboard.launch.py`
- `ros2 launch cddp_mpc hardware_validation.launch.py`
- `ros2 launch cddp_mpc px4_mpc_offboard.launch.py`
  This remains as a backward-compatible alias to `mpc_offboard.launch.py`, including `params_file:=...` overrides.

The PX4 node prefers `VehicleOdometry` when it is available and frame-valid, and falls back to local position plus attitude otherwise.

The PX4 node subscribes to:

- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_status`

and publishes:

- `/fmu/in/offboard_control_mode`
- `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/in/vehicle_torque_setpoint`
- `/fmu/in/vehicle_command`

Parameter files are in [`config/mpc_sitl.yaml`](config/mpc_sitl.yaml) and [`config/mpc_hardware.yaml`](config/mpc_hardware.yaml).

## Contributing
Contributions are welcome! Please feel free to submit pull requests.

## Citing
If you use this work in an academic context, please cite this repository.

## License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
