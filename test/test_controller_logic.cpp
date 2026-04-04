#include <gtest/gtest.h>

#include <limits>

#include "cddp_mpc/controller_logic.hpp"

TEST(ControllerLogicTest, SelectCommandSourceMatchesActiveFlightRecoveryFlow) {
  EXPECT_EQ(cddp_mpc::selectCommandSource("READY", false, 0, 5),
            cddp_mpc::CommandSource::ReadyWait);
  EXPECT_EQ(cddp_mpc::selectCommandSource("LAND_DONE", false, 0, 5),
            cddp_mpc::CommandSource::LandDoneIdle);
  EXPECT_EQ(cddp_mpc::selectCommandSource("TAKEOFF", false, 3, 5),
            cddp_mpc::CommandSource::RecoveryHover);
  EXPECT_EQ(cddp_mpc::selectCommandSource("LAND", false, 3, 5),
            cddp_mpc::CommandSource::RecoveryDescent);
  EXPECT_EQ(cddp_mpc::selectCommandSource("INIT", false, 2, 5),
            cddp_mpc::CommandSource::LastGoodHold);
  EXPECT_EQ(cddp_mpc::selectCommandSource("INIT", false, 8, 5),
            cddp_mpc::CommandSource::FallbackHover);
  EXPECT_EQ(cddp_mpc::selectCommandSource("HOVER", true, 8, 5),
            cddp_mpc::CommandSource::Solver);
}

TEST(ControllerLogicTest, UpdateTakeoffSetpointPreservesExistingXYTarget) {
  const Eigen::Vector3d current_position_enu(4.0, -2.0, 0.8);
  const Eigen::Vector3d previous_setpoint_enu(1.5, 2.5, -3.0);

  const Eigen::Vector3d updated = cddp_mpc::updateTakeoffSetpoint(
      current_position_enu, 5.0, previous_setpoint_enu);

  EXPECT_NEAR(updated.x(), previous_setpoint_enu.x(), 1e-9);
  EXPECT_NEAR(updated.y(), previous_setpoint_enu.y(), 1e-9);
  EXPECT_NEAR(updated.z(), -5.0, 1e-9);
}

TEST(ControllerLogicTest, UpdateTakeoffSetpointInitializesFromCurrentPose) {
  const Eigen::Vector3d current_position_enu(4.0, -2.0, 0.8);

  const Eigen::Vector3d updated =
      cddp_mpc::updateTakeoffSetpoint(current_position_enu, 5.0, std::nullopt);

  EXPECT_NEAR(updated.x(), current_position_enu.x(), 1e-9);
  EXPECT_NEAR(updated.y(), current_position_enu.y(), 1e-9);
  EXPECT_NEAR(updated.z(), -5.0, 1e-9);
}

TEST(ControllerLogicTest, TakeoffMinimumThrustRespectsReferenceFloor) {
  const double minimum = cddp_mpc::computeTakeoffMinimumThrust(
      -1.5, -3.0, 13.24, 1.0, 0.3, 0.55, 0.0, 20.0, 15.27);

  EXPECT_NEAR(minimum, 15.27, 1e-9);
}

TEST(ControllerLogicTest, TakeoffMinimumThrustDropsToBaseFloorPastTarget) {
  const double minimum = cddp_mpc::computeTakeoffMinimumThrust(
      -3.5, -3.0, 13.24, 1.0, 0.3, 0.55, 0.0, 20.0, 0.0);

  EXPECT_NEAR(minimum, 13.24 * 0.55, 1e-9);
}

TEST(ControllerLogicTest, ShapeSolverCommandBlendsOutsideTakeoff) {
  Eigen::Vector4d command;
  command << 16.0, 0.5, -0.2, 0.1;
  Eigen::Vector4d previous;
  previous << 14.0, 0.1, 0.1, 0.0;

  const Eigen::Vector4d shaped = cddp_mpc::shapeSolverCommand(
      command, previous, cddp_mpc::CommandSource::Solver, "HOVER", 0.2, 0.6, 0.8,
      13.24, 0.55, 1.0, 0.8);

  EXPECT_NEAR(shaped(0), 14.4, 1e-9);
  EXPECT_NEAR(shaped(1), 0.18, 1e-9);
  EXPECT_NEAR(shaped(2), 0.04, 1e-9);
  EXPECT_NEAR(shaped(3), 0.02, 1e-9);
}

TEST(ControllerLogicTest, ShapeSolverCommandLimitsHoverRatesWithSmallPositionError) {
  Eigen::Vector4d command;
  command << 13.0, 0.8, -0.8, 0.8;
  const Eigen::Vector4d previous = Eigen::Vector4d::Zero();

  const Eigen::Vector4d shaped = cddp_mpc::shapeSolverCommand(
      command, previous, cddp_mpc::CommandSource::Solver, "HOVER", 1.0, 0.6, 0.8,
      13.24, 0.55, 1.0, 0.2);

  EXPECT_NEAR(shaped(1), 0.12, 1e-9);
  EXPECT_NEAR(shaped(2), -0.12, 1e-9);
  EXPECT_NEAR(shaped(3), 0.12, 1e-9);
}

TEST(ControllerLogicTest, ApplyFlightThrustFloorProtectsHoverAndTakeoff) {
  Eigen::Vector4d command;
  command << 10.0, 0.0, 0.0, 0.0;

  const Eigen::Vector4d takeoff_guarded = cddp_mpc::applyFlightThrustFloor(
      command, "TAKEOFF", 13.24, 0.55, 1.0, 0.3, 15.27,
      std::numeric_limits<double>::quiet_NaN(), 1.5, 0.0, 20.0);
  const Eigen::Vector4d hover_handoff_guarded = cddp_mpc::applyFlightThrustFloor(
      command, "HOVER", 13.24, 0.55, 1.0, 0.3, 15.27, 0.0, 1.5, 0.0, 20.0);
  const Eigen::Vector4d hover_settled_guarded = cddp_mpc::applyFlightThrustFloor(
      command, "HOVER", 13.24, 0.55, 1.0, 0.3, 15.27, 2.0, 1.5, 0.0, 20.0);

  EXPECT_NEAR(takeoff_guarded(0), 15.27, 1e-9);
  EXPECT_NEAR(hover_handoff_guarded(0), 15.27, 1e-9);
  EXPECT_NEAR(hover_settled_guarded(0), 13.24, 1e-9);
}

TEST(ControllerLogicTest, HoverMinimumThrustDecaysFromTakeoffFloor) {
  const double immediate = cddp_mpc::computeHoverMinimumThrust(
      13.24, 0.55, 1.0, 15.27, 0.0, 1.5, 0.0, 20.0);
  const double decayed = cddp_mpc::computeHoverMinimumThrust(
      13.24, 0.55, 1.0, 15.27, 0.75, 1.5, 0.0, 20.0);
  const double settled = cddp_mpc::computeHoverMinimumThrust(
      13.24, 0.55, 1.0, 15.27, 2.0, 1.5, 0.0, 20.0);

  EXPECT_NEAR(immediate, 15.27, 1e-9);
  EXPECT_NEAR(decayed, 14.255, 1e-9);
  EXPECT_NEAR(settled, 13.24, 1e-9);
}

TEST(ControllerLogicTest, HoverVerticalCorrectionAddsUpwardThrustBelowTarget) {
  Eigen::Vector4d command;
  command << 13.24, 0.0, 0.0, 0.0;

  const Eigen::Vector4d corrected = cddp_mpc::applyHoverVerticalCorrection(
      command, "HOVER", 0.2, 0.0, 2.0, 0.8, 1.5, 0.0, 20.0);

  EXPECT_NEAR(corrected(0), 13.64, 1e-9);
}

TEST(ControllerLogicTest, HoverVerticalCorrectionDampsClimbRate) {
  Eigen::Vector4d command;
  command << 14.5, 0.0, 0.0, 0.0;

  const Eigen::Vector4d corrected = cddp_mpc::applyHoverVerticalCorrection(
      command, "HOVER", 0.0, 0.5, 2.0, 0.8, 1.5, 0.0, 20.0);

  EXPECT_NEAR(corrected(0), 14.1, 1e-9);
}

TEST(ControllerLogicTest, HoverLateralCorrectionAddsPitchBackForForwardError) {
  Eigen::Vector4d command;
  command << 13.24, 0.0, 0.0, 0.0;

  const Eigen::Vector4d corrected = cddp_mpc::applyHoverLateralCorrection(
      command, "HOVER", 0.4, 0.0, 0.0, 0.0, 0.25, 0.12, 0.25, 0.8);

  EXPECT_NEAR(corrected(1), 0.0, 1e-9);
  EXPECT_NEAR(corrected(2), -0.1, 1e-9);
}

TEST(ControllerLogicTest, HoverLateralCorrectionAddsRollForRightError) {
  Eigen::Vector4d command;
  command << 13.24, 0.0, 0.0, 0.0;

  const Eigen::Vector4d corrected = cddp_mpc::applyHoverLateralCorrection(
      command, "HOVER", 0.0, 0.4, 0.0, 0.0, 0.25, 0.12, 0.25, 0.8);

  EXPECT_NEAR(corrected(1), 0.1, 1e-9);
  EXPECT_NEAR(corrected(2), 0.0, 1e-9);
}

TEST(ControllerLogicTest, WarmStartResetTriggersOnStateMismatch) {
  Eigen::VectorXd initial = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(10);

  initial(0) = 2.0;
  EXPECT_TRUE(cddp_mpc::shouldResetWarmStart(initial, expected, 0.1, 1.0, 1.5));

  initial.setZero();
  expected.setZero();
  expected(6) = 1.0;
  initial(6) = 0.0;
  initial(9) = 1.0;
  EXPECT_TRUE(cddp_mpc::shouldResetWarmStart(initial, expected, 0.1, 1.0, 1.5));
}

TEST(ControllerLogicTest, WarmStartResetAllowsNearbyState) {
  Eigen::VectorXd initial = Eigen::VectorXd::Zero(10);
  Eigen::VectorXd expected = Eigen::VectorXd::Zero(10);
  initial(6) = 1.0;
  expected(6) = 1.0;
  initial(0) = 0.1;
  expected(0) = 0.05;
  initial(3) = 0.2;
  expected(3) = 0.1;

  EXPECT_FALSE(
      cddp_mpc::shouldResetWarmStart(initial, expected, 0.1, 1.0, 1.5));
}
