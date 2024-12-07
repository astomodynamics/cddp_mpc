# cddp_mpc_pkg

## Build cddp_mpc_pkg
```bash
mkdir ros_ws && cd ros_ws
mkdir src && cd src
git clone https://github.com/astomodynamics/cddp_mpc
cd ../ # to ~/ros_ws
colcon build --packages-select cddp_mpc
```


## [Optional] Build CDDP Package
```bash
git clone https://github.com/astomodynamics/cddp-cpp
cd cddp-cpp
mkdir build && cd build
cmake ..
make
```
