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

TEST(Px4UtilsTest, YawNedToQuaternionEnuMatchesNorthFacingLevelAttitude) {
  const Eigen::Quaterniond q_enu = cddp_mpc::yawNedToQuaternionEnu(0.0);

  EXPECT_NEAR(q_enu.norm(), 1.0, 1e-9);
  EXPECT_NEAR(q_enu.w(), std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(q_enu.x(), 0.0, 1e-9);
  EXPECT_NEAR(q_enu.y(), 0.0, 1e-9);
  EXPECT_NEAR(q_enu.z(), std::sqrt(0.5), 1e-9);
}

TEST(Px4UtilsTest, ThrustCommandMapsToNegativeBodyZ) {
  const auto thrust_body =
      cddp_mpc::thrustBodyFromCommand(10.0, 20.0);

  EXPECT_FLOAT_EQ(thrust_body[0], 0.0F);
  EXPECT_FLOAT_EQ(thrust_body[1], 0.0F);
  EXPECT_FLOAT_EQ(thrust_body[2], -0.5F);
}

TEST(Px4UtilsTest, TorqueCommandUsesProvidedScalePerAxis) {
  const Eigen::Vector3d torque_nm(0.2, -0.15, 0.08);
  const Eigen::Vector3d scale(0.5, 2.0, 4.0);
  const auto torque_body = cddp_mpc::torqueBodyFromCommand(torque_nm, scale);

  EXPECT_FLOAT_EQ(torque_body[0], 0.1F);
  EXPECT_FLOAT_EQ(torque_body[1], -0.3F);
  EXPECT_FLOAT_EQ(torque_body[2], 0.32F);
}

TEST(Px4UtilsTest, TorqueCommandWithoutScaleFallsBackToZero) {
  const Eigen::Vector3d torque_nm(0.2, -0.15, 0.08);
  const auto torque_body =
      cddp_mpc::torqueBodyFromCommand(torque_nm, Eigen::Vector3d::Zero());

  EXPECT_FLOAT_EQ(torque_body[0], 0.0F);
  EXPECT_FLOAT_EQ(torque_body[1], 0.0F);
  EXPECT_FLOAT_EQ(torque_body[2], 0.0F);
}

TEST(Px4UtilsTest, NormalizeOdometryAutoDefaultsToWxyzQuaternionOrder) {
  cddp_mpc::FrameAdapterConfig config;
  config.quaternion_order = "auto";
  config.odom_body_frame = "frd";

  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d(1.0, 2.0, -3.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), 1, 1, config);

  EXPECT_TRUE(odom.frame_ok);
  EXPECT_EQ(odom.quat_order_used, "wxyz");
  EXPECT_NEAR(odom.attitude_wxyz.w(), 0.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.x(), 0.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.y(), 0.0, 1e-9);
  EXPECT_NEAR(odom.attitude_wxyz.z(), 1.0, 1e-9);
}

TEST(Px4UtilsTest, NormalizeOdometrySupportsExplicitXyzwQuaternionOrder) {
  cddp_mpc::FrameAdapterConfig config;
  config.quaternion_order = "xyzw";
  config.odom_body_frame = "frd";

  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), 1, 1, config);

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

TEST(Px4UtilsTest, NormalizeOdometryLeavesFrdBodyRatesUnchanged) {
  cddp_mpc::FrameAdapterConfig config;
  config.quaternion_order = "xyzw";
  config.odom_body_frame = "frd";

  const Eigen::Vector3d body_rates_frd(0.1, -0.2, 0.3);
  const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), body_rates_frd,
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0), 1, 1, config);

  EXPECT_TRUE(odom.frame_ok);
  EXPECT_EQ(odom.body_frame_used, "frd");
  EXPECT_NEAR(odom.angular_velocity.x(), body_rates_frd.x(), 1e-9);
  EXPECT_NEAR(odom.angular_velocity.y(), body_rates_frd.y(), 1e-9);
  EXPECT_NEAR(odom.angular_velocity.z(), body_rates_frd.z(), 1e-9);
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

TEST(Px4UtilsTest, MotorForceAllocationMatchesExpectedMoments) {
  const Eigen::Vector3d torque(0.2, -0.15, 0.08);
  const Eigen::Vector4d motors = cddp_mpc::motorForcesFromThrustTorque(
      12.0, torque, 0.25, cddp_mpc::kQuadrotorModelYawMomentCoefficient);

  EXPECT_NEAR(motors.sum(), 12.0, 1e-9);
  EXPECT_NEAR(0.25 * (motors(0) - motors(2)), torque.x(), 1e-9);
  EXPECT_NEAR(0.25 * (motors(1) - motors(3)), torque.y(), 1e-9);
  EXPECT_NEAR(cddp_mpc::kQuadrotorModelYawMomentCoefficient *
                  (motors(0) - motors(1) + motors(2) - motors(3)),
              torque.z(), 1e-9);
}

TEST(Px4UtilsTest, ProjectThrustTorqueToMotorLimitsPreservesFeasibleThrust) {
  const Eigen::Vector4d command(16.0, 2.0, -2.0, 1.0);
  const Eigen::Vector4d projected = cddp_mpc::projectThrustTorqueToMotorLimits(
      command, 0.0, 5.0, 0.25, cddp_mpc::kQuadrotorModelYawMomentCoefficient);
  const Eigen::Vector4d motors = cddp_mpc::motorForcesFromThrustTorque(
      projected(0), projected.tail<3>(), 0.25,
      cddp_mpc::kQuadrotorModelYawMomentCoefficient);

  EXPECT_NEAR(projected(0), 16.0, 1e-9);
  EXPECT_TRUE((motors.array() >= -1e-9).all());
  EXPECT_TRUE((motors.array() <= 5.0 + 1e-9).all());
  EXPECT_LT(projected.tail<3>().norm(), command.tail<3>().norm());
}
