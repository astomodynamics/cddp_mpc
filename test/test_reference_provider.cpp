#include <gtest/gtest.h>

#include "cddp_mpc/reference_provider.hpp"

TEST(ReferenceProviderTest, DrivesPositionErrorTowardZeroAcrossAxes) {
  cddp_mpc::ReferenceConfig config;
  config.horizon_steps = 20;
  config.mpc_dt = 0.04;
  config.hover_thrust_n = 13.2;
  config.min_thrust_n = 0.0;
  config.max_thrust_n = 20.0;
  config.max_axis_speed_mps = 1.5;
  config.max_axis_accel_mps2 = 2.0;
  config.smoothing_tau_s = 0.5;
  config.target_yaw_rad = 0.3;

  cddp_mpc::PositionYawReferenceProvider provider(config);

  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(13);
  initial_state(0) = 2.0;
  initial_state(1) = -1.0;
  initial_state(2) = 1.5;
  initial_state(7) = 0.2;
  initial_state(8) = -0.1;
  initial_state(9) = 0.3;

  const cddp_mpc::ReferenceTrajectory reference = provider.build(initial_state);

  ASSERT_EQ(reference.states.size(), 21U);
  ASSERT_EQ(reference.controls.size(), 21U);
  EXPECT_LT(std::abs(reference.states.back()(0)), std::abs(initial_state(0)));
  EXPECT_LT(std::abs(reference.states.back()(1)), std::abs(initial_state(1)));
  EXPECT_LT(std::abs(reference.states.back()(2)), std::abs(initial_state(2)));
  EXPECT_TRUE(reference.controls.front().allFinite());
  EXPECT_GE(reference.controls.front()(0), 0.0);
}

TEST(ReferenceProviderTest, CircleModeStartsContinuouslyAndRespectsSpeedLimit) {
  cddp_mpc::ReferenceConfig config;
  config.horizon_steps = 20;
  config.mpc_dt = 0.05;
  config.hover_thrust_n = 13.2;
  config.min_thrust_n = 0.0;
  config.max_thrust_n = 20.0;
  config.max_axis_speed_mps = 1.0;
  config.max_axis_accel_mps2 = 2.0;
  config.max_yaw_rate_rad_s = 0.4;
  config.circle_radius_m = 0.5;
  config.trajectory_period_s = 8.0;
  config.mode = "circle";

  cddp_mpc::PositionYawReferenceProvider provider(config);
  const Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(13);
  const cddp_mpc::ReferenceTrajectory reference = provider.build(initial_state);

  ASSERT_EQ(reference.states.size(), 21U);
  EXPECT_NEAR(reference.states.front()(0), 0.0, 1e-9);
  EXPECT_NEAR(reference.states.front()(1), 0.0, 1e-9);
  for (const auto &state : reference.states) {
    EXPECT_LE(state.segment<3>(7).cwiseAbs().maxCoeff(), 1.0 + 1e-9);
  }
}

TEST(ReferenceProviderTest, FigureEightModeProducesBoundedLateralMotion) {
  cddp_mpc::ReferenceConfig config;
  config.horizon_steps = 20;
  config.mpc_dt = 0.05;
  config.hover_thrust_n = 13.2;
  config.min_thrust_n = 0.0;
  config.max_thrust_n = 20.0;
  config.max_axis_speed_mps = 1.2;
  config.max_axis_accel_mps2 = 2.0;
  config.figure_eight_amplitude_m = 0.7;
  config.trajectory_period_s = 10.0;
  config.mode = "figure_eight";

  cddp_mpc::PositionYawReferenceProvider provider(config);
  const Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(13);
  const cddp_mpc::ReferenceTrajectory reference = provider.build(initial_state);

  ASSERT_EQ(reference.states.size(), 21U);
  double max_lateral_error = 0.0;
  for (const auto &state : reference.states) {
    max_lateral_error = std::max(max_lateral_error, std::abs(state(1)));
    EXPECT_LE(state.segment<3>(7).cwiseAbs().maxCoeff(), 1.2 + 1e-9);
  }
  EXPECT_GT(max_lateral_error, 0.0);
}
