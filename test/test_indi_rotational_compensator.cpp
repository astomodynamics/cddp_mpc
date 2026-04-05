#include <gtest/gtest.h>

#include "cddp_mpc/indi_rotational_compensator.hpp"

TEST(IndiRotationalCompensatorTest, DisabledModeReturnsNominalTorque) {
  cddp_mpc::IndiRotationalCompensator compensator(
      Eigen::Vector3d(0.029, 0.029, 0.055).asDiagonal());
  cddp_mpc::IndiRotationalCompensator::Config config;
  config.enabled = false;
  config.blend_alpha = 0.5;
  compensator.setConfig(config);
  compensator.updateMeasurement(0.0, Eigen::Vector3d::Zero());

  const Eigen::Vector3d nominal(0.1, -0.08, 0.03);
  const Eigen::Vector3d output = compensator.computeTorqueCommand(
      0.01, nominal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  EXPECT_NEAR((output - nominal).norm(), 0.0, 1e-9);
}

TEST(IndiRotationalCompensatorTest, CorrectionIsClamped) {
  cddp_mpc::IndiRotationalCompensator compensator(
      Eigen::Vector3d(0.029, 0.029, 0.055).asDiagonal());
  cddp_mpc::IndiRotationalCompensator::Config config;
  config.enabled = true;
  config.blend_alpha = 1.0;
  config.rate_lpf_cutoff_hz = 50.0;
  config.accel_lpf_cutoff_hz = 50.0;
  config.torque_correction_limit_nm = 0.05;
  compensator.setConfig(config);
  compensator.updateMeasurement(0.0, Eigen::Vector3d::Zero());
  compensator.updateMeasurement(0.01, Eigen::Vector3d(4.0, -4.0, 2.0));

  const Eigen::Vector3d nominal(0.1, 0.1, 0.02);
  const Eigen::Vector3d corrected = compensator.computeTorqueCommand(
      0.02, nominal, Eigen::Vector3d(4.0, -4.0, 2.0), Eigen::Vector3d::Zero());

  const Eigen::Vector3d delta = corrected - nominal;
  EXPECT_LE(delta.cwiseAbs().maxCoeff(), 0.05 + 1e-9);
  EXPECT_TRUE(compensator.status().active);
}
