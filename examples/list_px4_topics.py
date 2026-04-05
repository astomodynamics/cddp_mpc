#!/usr/bin/env python3
"""List PX4 topics visible to ROS 2."""

import rclpy
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("topic_lister")

    print("\n" + "=" * 60)
    print("PX4 Topics Available")
    print("=" * 60)

    topic_list = node.get_topic_names_and_types()
    px4_topics = [(name, types) for name, types in topic_list if name.startswith("/fmu")]

    if not px4_topics:
        print("\nNo PX4 topics found.")
        print("  Make sure:")
        print("  1. PX4 SITL is running (ros2 launch cddp_mpc px4_simulation.launch.py)")
        print("  2. MicroXRCEAgent is reachable")
    else:
        print(f"\nFound {len(px4_topics)} PX4 topics:\n")

        input_topics = [t for t in px4_topics if "/fmu/in/" in t[0]]
        output_topics = [t for t in px4_topics if "/fmu/out/" in t[0]]

        print("Output Topics (from PX4):")
        print("-" * 60)
        for name, types in sorted(output_topics):
            print(f"  {name}")
            for msg_type in types:
                print(f"    - {msg_type}")
        print()

        print("Input Topics (to PX4):")
        print("-" * 60)
        for name, types in sorted(input_topics):
            print(f"  {name}")
            for msg_type in types:
                print(f"    - {msg_type}")

    print("\n" + "=" * 60)
    print("\nCommon topics to monitor:")
    print("  - /fmu/out/vehicle_odometry")
    print("  - /fmu/out/vehicle_status")
    print("  - /fmu/out/vehicle_local_position")
    print("  - /cddp_mpc/status")
    print("\nCommon topics to publish:")
    print("  - /fmu/in/offboard_control_mode")
    print("  - /fmu/in/vehicle_thrust_setpoint")
    print("  - /fmu/in/vehicle_torque_setpoint")
    print("  - /fmu/in/vehicle_command")
    print()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
