#include <cmath>

#include <gtest/gtest.h>

#include "cddp_mpc/px4_utils.hpp"

TEST(Px4UtilsTest, NedToEnuSwapsAxesAndFlipsZ) {
  const Eigen::Vector3d ned(1.0, 2.0, -3.0);
  const Eigen::Vector3d enu = cddp_mpc::nedToEnu(ned);

  EXPECT_DOUBLE_EQ(enu.x(), 2.0);
  EXPECT_DOUBLE_EQ(enu.y(), 1.0);
  EXPECT_DOUBLE_EQ(enu.z(), 3.0);
}

TEST(Px4UtilsTest, IdentityQuaternionConvertsToExpectedEnuFrame) {
  const Eigen::Quaterniond q_enu =
      cddp_mpc::quatNedToEnuWxyz(Eigen::Quaterniond::Identity());

  EXPECT_NEAR(q_enu.norm(), 1.0, 1e-9);
  EXPECT_NEAR(q_enu.w(), std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(q_enu.z(), std::sqrt(0.5), 1e-9);
}

TEST(Px4UtilsTest, ThrustCommandMapsToNegativeBodyZ) {
  const auto thrust_body =
      cddp_mpc::thrustBodyFromCommand(10.0, 20.0);

  EXPECT_FLOAT_EQ(thrust_body[0], 0.0F);
  EXPECT_FLOAT_EQ(thrust_body[1], 0.0F);
  EXPECT_FLOAT_EQ(thrust_body[2], -0.5F);
}

TEST(Px4UtilsTest, NormalizeOdometryAutoDetectsXyzwQuaternionOrder) {
  cddp_mpc::FrameAdapterConfig config;
  config.quaternion_order = "auto";
  config.odom_body_frame = "frd";

  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), 1, 1, config);

  EXPECT_TRUE(odom.frame_ok);
  EXPECT_EQ(odom.quat_order_used, "xyzw");
  EXPECT_NEAR(odom.attitude_wxyz.w(), 1.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.x(), 0.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.y(), 0.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.z(), 0.0, 1e-9);
}

TEST(Px4UtilsTest, NormalizeOdometryConvertsFluBodyRatesToFrd) {
  cddp_mpc::FrameAdapterConfig config;
  config.quaternion_order = "xyzw";
  config.odom_body_frame = "flu";

  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.1, 0.2, -0.3), Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), 1, 1,
      config);

  EXPECT_TRUE(odom.frame_ok);
  EXPECT_EQ(odom.body_frame_used, "flu");
  EXPECT_NEAR(odom.angular_velocity.x(), 0.1, 1e-9);
  EXPECT_NEAR(odom.angular_velocity.y(), -0.2, 1e-9);
  EXPECT_NEAR(odom.angular_velocity.z(), 0.3, 1e-9);
}

TEST(Px4UtilsTest, NormalizeOdometryRejectsUnexpectedFrameMetadata) {
  cddp_mpc::FrameAdapterConfig config;
  config.expected_pose_frame_ned = 1;
  config.expected_velocity_frame_ned = 1;

  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), 2, 1, config);

  EXPECT_FALSE(odom.frame_ok);
  EXPECT_EQ(odom.pose_frame, 2);
  EXPECT_EQ(odom.velocity_frame, 1);
}
