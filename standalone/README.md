# Standalone Tools

This directory contains standalone utilities that do not require installing `cddp_mpc` as a Python package.

## PX4 Data Collector

`px4_data_collector.py` is a single-file ROS 2 node for collecting synchronized PX4 flight data.

Requirements:
- ROS 2 Humble or later
- `px4_msgs` available in the sourced workspace
- Python packages: `numpy`, `scipy`

Usage:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 standalone/px4_data_collector.py --duration 60 --dt 0.02 --output flight_data.npz
```

The output `.npz` file contains synchronized arrays for:
- `timestamps`
- `positions`
- `velocities`
- `quaternions`
- `angular_velocities`
- `accelerations`
- `controls`

The collector prefers `VehicleOdometry` when available, but will fall back to combining local position, attitude, and angular-rate topics when needed.
