#!/usr/bin/env python3
"""Test script to verify PX4 connection via Micro XRCE-DDS."""

import rclpy
from px4_msgs.msg import SensorCombined, VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class PX4ConnectionTest(Node):
    """Simple node to verify that PX4 topics are flowing."""

    def __init__(self):
        super().__init__("px4_connection_test")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odometry_count = 0
        self.status_count = 0
        self.sensor_count = 0

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odometry_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.status_callback,
            qos_profile,
        )
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self.sensor_callback,
            qos_profile,
        )

        self.create_timer(2.0, self.print_status)

        self.get_logger().info("PX4 Connection Test initialized")
        self.get_logger().info("Listening for PX4 topics...")

    def odometry_callback(self, msg: VehicleOdometry) -> None:
        self.odometry_count += 1
        if self.odometry_count == 1:
            pos = msg.position
            self.get_logger().info("Receiving vehicle_odometry")
            self.get_logger().info(
                f"  Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            )

    def status_callback(self, msg: VehicleStatus) -> None:
        self.status_count += 1
        if self.status_count == 1:
            self.get_logger().info("Receiving vehicle_status")
            self.get_logger().info(f"  Arming state: {msg.arming_state}")
            self.get_logger().info(f"  Nav state: {msg.nav_state}")

    def sensor_callback(self, _msg: SensorCombined) -> None:
        self.sensor_count += 1
        if self.sensor_count == 1:
            self.get_logger().info("Receiving sensor_combined")

    def print_status(self) -> None:
        self.get_logger().info("=" * 50)
        self.get_logger().info("Messages received in last 2 seconds:")
        self.get_logger().info(f"  vehicle_odometry:  {self.odometry_count}")
        self.get_logger().info(f"  vehicle_status:    {self.status_count}")
        self.get_logger().info(f"  sensor_combined:   {self.sensor_count}")
        self.get_logger().info("=" * 50)
        self.odometry_count = 0
        self.status_count = 0
        self.sensor_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PX4ConnectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
