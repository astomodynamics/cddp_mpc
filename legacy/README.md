# Legacy Ground-Robot Demo

These files preserve the pre-PX4 unicycle and car MPC examples that were still present at the merged `#13` state of this repository.

They are intentionally archived:

- Build them only with `-DBUILD_LEGACY_GROUND_DEMOS=ON`
- Use the `legacy_*` executables and `simple_robot_bringup.launch.py`
- Treat them as historical examples, not the primary supported controller path

Build example:

```bash
colcon build --packages-select cddp_mpc --cmake-args -DBUILD_LEGACY_GROUND_DEMOS=ON
```

Bring up the map, RViz, and simple unicycle simulator:

```bash
ros2 launch cddp_mpc simple_robot_bringup.launch.py
```

Optional manual teleop:

```bash
ros2 run cddp_mpc legacy_teleop_keyboard_node --ros-args -p robot_id:=robot_1
```

Archived MPC nodes:

```bash
ros2 run cddp_mpc legacy_mpc_node
ros2 run cddp_mpc legacy_car_mpc_node
```

Important limitation:

- The archived MPC nodes still expect a path on `/robot_1/global_path`
- The bringup launch does not provide a path planner
- The GIF and sources are kept for reference and experimentation, not as a maintained end-to-end demo
