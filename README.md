# cddp_mpc_pkg


## Build CDDP Package
```bash
git clone https://github.com/astomodynamics/CDDP-cpp
cd CDDP-cpp
mkdir build && cd build
cmake ..
make
```

## Build cddp_mpc_pkg
```bash
mkdir ros_ws && cd ros_ws
mkdir src && cd src
git clone https://github.com/astomodynamics/cddp_mpc_pkg
cd ../ # to ~/ros_ws
colcon build --packages-select cddp_mpc_pkg
```
