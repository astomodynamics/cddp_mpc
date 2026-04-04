#include <gtest/gtest.h>

#include "cddp_mpc/error_state_quadrotor_rate.hpp"
#include "cddp_mpc/px4_utils.hpp"

TEST(ErrorStateQuadrotorRateTest, HoverEquilibriumStaysNearZeroError) {
  const double mass_kg = 1.35;
  const double gravity_mps2 = 9.81;
  const double hover_thrust_n = mass_kg * gravity_mps2;

  cddp_mpc::ErrorStateEnuQuadrotorRate system(
      0.1, mass_kg, 20.0, 0.8, Eigen::Vector3d(2.0, -1.0, 3.0), "rk4");

  Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
  const Eigen::Quaterniond quat_enu =
      cddp_mpc::quatNedToEnuWxyz(Eigen::Quaterniond::Identity());
  state(6) = quat_enu.w();
  state(7) = quat_enu.x();
  state(8) = quat_enu.y();
  state(9) = quat_enu.z();

  Eigen::VectorXd control = Eigen::VectorXd::Zero(4);
  control(0) = hover_thrust_n;

  const Eigen::VectorXd state_dot = system.getContinuousDynamics(state, control, 0.0);
  const Eigen::VectorXd next_state = system.getDiscreteDynamics(state, control, 0.0);

  EXPECT_NEAR(state_dot.segment<3>(0).norm(), 0.0, 1e-9);
  EXPECT_NEAR(state_dot.segment<3>(3).norm(), 0.0, 1e-9);
  EXPECT_NEAR(next_state.segment<6>(0).norm(), 0.0, 1e-9);
  EXPECT_NEAR(next_state.segment<4>(6).norm(), 1.0, 1e-9);
}

TEST(ErrorStateQuadrotorRateTest, ForwardPitchProducesForwardEnuAcceleration) {
  const double mass_kg = 1.35;
  const double gravity_mps2 = 9.81;
  const double hover_thrust_n = mass_kg * gravity_mps2;

  cddp_mpc::ErrorStateEnuQuadrotorRate system(
      0.1, mass_kg, 20.0, 0.8, Eigen::Vector3d::Zero(), "rk4");

  Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
  const Eigen::AngleAxisd pitch_enu(-0.15, Eigen::Vector3d::UnitY());
  const Eigen::Quaterniond quat_enu(pitch_enu);
  state(6) = quat_enu.w();
  state(7) = quat_enu.x();
  state(8) = quat_enu.y();
  state(9) = quat_enu.z();

  Eigen::VectorXd control = Eigen::VectorXd::Zero(4);
  control(0) = hover_thrust_n;

  const Eigen::VectorXd state_dot = system.getContinuousDynamics(state, control, 0.0);

  EXPECT_GT(state_dot(3), 0.0);
}
