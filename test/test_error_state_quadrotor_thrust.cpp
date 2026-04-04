#include <gtest/gtest.h>

#include "cddp_mpc/error_state_quadrotor_thrust.hpp"
#include "cddp_mpc/px4_utils.hpp"

TEST(ErrorStateQuadrotorThrustTest, HoverEquilibriumStaysNearZeroError) {
  const double mass_kg = 1.35;
  const double gravity_mps2 = 9.81;
  const double hover_thrust_n = mass_kg * gravity_mps2;

  cddp_mpc::ErrorStateEnuQuadrotorThrust system(
      0.1, mass_kg, Eigen::Vector3d(0.029, 0.029, 0.055).asDiagonal(), 0.25,
      Eigen::Vector3d(2.0, -1.0, 3.0), "rk4");

  Eigen::VectorXd state = Eigen::VectorXd::Zero(13);
  const Eigen::Quaterniond quat_enu =
      cddp_mpc::quatNedToEnuWxyz(Eigen::Quaterniond::Identity());
  state(3) = quat_enu.w();
  state(4) = quat_enu.x();
  state(5) = quat_enu.y();
  state(6) = quat_enu.z();

  Eigen::VectorXd control = Eigen::VectorXd::Zero(4);
  control(0) = hover_thrust_n;

  const Eigen::VectorXd state_dot = system.getContinuousDynamics(state, control, 0.0);
  const Eigen::VectorXd next_state = system.getDiscreteDynamics(state, control, 0.0);

  EXPECT_NEAR(state_dot.segment<3>(0).norm(), 0.0, 1e-9);
  EXPECT_NEAR(state_dot.segment<3>(7).norm(), 0.0, 1e-9);
  EXPECT_NEAR(state_dot.segment<3>(10).norm(), 0.0, 1e-9);
  EXPECT_NEAR(next_state.segment<3>(0).norm(), 0.0, 1e-9);
  EXPECT_NEAR(next_state.segment<4>(3).norm(), 1.0, 1e-9);
}

TEST(ErrorStateQuadrotorThrustTest, RollTorqueProducesAngularAcceleration) {
  const double mass_kg = 1.35;

  cddp_mpc::ErrorStateEnuQuadrotorThrust system(
      0.1, mass_kg, Eigen::Vector3d(0.029, 0.029, 0.055).asDiagonal(), 0.25,
      Eigen::Vector3d::Zero(), "rk4");

  Eigen::VectorXd state = Eigen::VectorXd::Zero(13);
  const Eigen::Quaterniond quat_enu =
      cddp_mpc::quatNedToEnuWxyz(Eigen::Quaterniond::Identity());
  state(3) = quat_enu.w();
  state(4) = quat_enu.x();
  state(5) = quat_enu.y();
  state(6) = quat_enu.z();

  Eigen::VectorXd control = Eigen::VectorXd::Zero(4);
  control(0) = mass_kg * 9.81;
  control(1) = 0.15;

  const Eigen::VectorXd state_dot = system.getContinuousDynamics(state, control, 0.0);

  EXPECT_GT(state_dot(10), 0.0);
}
