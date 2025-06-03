# CDDP MPC Package

A ROS package implementing Constrained Differential Dynamic Programming (CDDP) for Model Predictive Control (MPC).

<img src="video/cddp_mpc_demo.gif" width="500" alt="">

## Description

This package provides a ROS implementation of CDDP-based Model Predictive Control. It integrates with the [CDDP C++ library](https://github.com/astomodynamics/cddp-cpp) to enable real-time optimal control for robotic systems.

## Prerequisites

- ROS 2 (tested on ROS 2 Humble)
- C++17 or later
- CMake 3.8 or later
- [Optional] CDDP C++ Library

## Installation

### Build cddp_mpc_pkg

1. Create a ROS workspace and navigate to it:
```bash
mkdir ros_ws && cd ros_ws```
```
2. Create the source directory and clone the repository:
```bash
mkdir src && cd src
git clone https://github.com/astomodynamics/cddp_mpc
```
3. Return to the workspace root and build the package:
```bash
cd ../ # to ~/ros_ws
colcon build --packages-select cddp_mpc
```
### [Optional] Build CDDP C++ Library
If you want to use the standalone CDDP C++ library:

1. Clone the CDDP C++ repository:
```bash
git clone https://github.com/astomodynamics/cddp-cpp
```

2. Build the library:
```bash
cd cddp-cpp
mkdir build && cd build
cmake ..
make
```
## Usage
After building the package, source your ROS workspace:
```bash
source ~/ros_ws/install/setup.bash
```
### Running the Controller

To test the CDDP MPC controller, you'll need to run commands in three separate terminals.

1. Launch the Robot Node
```bash
ros2 launch cddp_mpc simple_robot_bringup.launch.py
```
2. Launch the MPC Node

In your first terminal, start the MPC node:
```bash
ros2 run cddp_mpc mpc_node
```

3. Select the robot goal at Rviz2 


## Contributing
Contributions are welcome! Please feel free to submit pull requests.

## Citing
If you use this work in an academic context, please cite this repository.

## License
This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
