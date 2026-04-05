#include "cddp_mpc/indi_rotational_compensator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cddp_mpc {

IndiRotationalCompensator::IndiRotationalCompensator(
    const Eigen::Matrix3d &inertia_matrix)
    : inertia_matrix_(inertia_matrix),
      inertia_inverse_(inertia_matrix.inverse()) {}

void IndiRotationalCompensator::setConfig(const Config &config) { config_ = config; }

void IndiRotationalCompensator::reset() {
  status_ = Status{};
  last_measurement_timestamp_s_.reset();
}

double IndiRotationalCompensator::lowPassAlpha(double cutoff_hz, double dt_s) {
  constexpr double kPi = 3.14159265358979323846;
  const double clamped_cutoff = std::max(cutoff_hz, 1e-3);
  const double clamped_dt = std::max(dt_s, 1e-6);
  const double tau = 1.0 / (2.0 * kPi * clamped_cutoff);
  return clamped_dt / (tau + clamped_dt);
}

void IndiRotationalCompensator::updateMeasurement(double timestamp_s,
                                                  const Eigen::Vector3d &body_rates) {
  if (!std::isfinite(timestamp_s) || !body_rates.allFinite()) {
    return;
  }

  if (!last_measurement_timestamp_s_.has_value()) {
    last_measurement_timestamp_s_ = timestamp_s;
    status_.has_measurement = true;
    status_.filtered_rates = body_rates;
    status_.filtered_accel.setZero();
    status_.measurement_age_s = 0.0;
    status_.update_rate_hz = 0.0;
    return;
  }

  const double dt_s = timestamp_s - *last_measurement_timestamp_s_;
  if (dt_s <= 1e-6) {
    return;
  }

  const Eigen::Vector3d previous_filtered_rates = status_.filtered_rates;
  const double rate_alpha = lowPassAlpha(config_.rate_lpf_cutoff_hz, dt_s);
  status_.filtered_rates += rate_alpha * (body_rates - status_.filtered_rates);

  const Eigen::Vector3d raw_accel =
      (status_.filtered_rates - previous_filtered_rates) / dt_s;
  const double accel_alpha = lowPassAlpha(config_.accel_lpf_cutoff_hz, dt_s);
  status_.filtered_accel += accel_alpha * (raw_accel - status_.filtered_accel);

  last_measurement_timestamp_s_ = timestamp_s;
  status_.has_measurement = true;
  status_.measurement_age_s = 0.0;
  status_.update_rate_hz = 1.0 / dt_s;
}

Eigen::Vector3d IndiRotationalCompensator::computeTorqueCommand(
    double timestamp_s, const Eigen::Vector3d &nominal_torque,
    const Eigen::Vector3d &body_rates,
    const Eigen::Vector3d &estimated_realized_torque) {
  status_.active = false;
  status_.desired_accel.setZero();
  status_.torque_estimate = estimated_realized_torque;
  status_.torque_correction.setZero();
  status_.corrected_torque = nominal_torque;
  status_.using_rpm_feedback = false;
  status_.using_command_feedback_fallback = !config_.use_rpm_feedback;
  if (last_measurement_timestamp_s_.has_value()) {
    status_.measurement_age_s =
        std::max(0.0, timestamp_s - *last_measurement_timestamp_s_);
  } else {
    status_.measurement_age_s = std::numeric_limits<double>::infinity();
  }

  if (!config_.enabled || !status_.has_measurement || !nominal_torque.allFinite() ||
      !body_rates.allFinite()) {
    return nominal_torque;
  }

  const Eigen::Vector3d gyroscopic_term =
      body_rates.cross(inertia_matrix_ * body_rates);
  status_.desired_accel = inertia_inverse_ * (nominal_torque - gyroscopic_term);

  Eigen::Vector3d corrected =
      estimated_realized_torque +
      inertia_matrix_ * (status_.desired_accel - status_.filtered_accel);
  corrected =
      nominal_torque + std::clamp(config_.blend_alpha, 0.0, 1.0) * (corrected - nominal_torque);

  const double max_correction = std::max(0.0, config_.torque_correction_limit_nm);
  const Eigen::Vector3d raw_correction = corrected - nominal_torque;
  Eigen::Vector3d limited_correction = raw_correction;
  for (int axis = 0; axis < 3; ++axis) {
    limited_correction(axis) =
        std::clamp(limited_correction(axis), -max_correction, max_correction);
  }

  status_.active = true;
  status_.torque_correction = limited_correction;
  status_.corrected_torque = nominal_torque + limited_correction;
  return status_.corrected_torque;
}

const IndiRotationalCompensator::Status &IndiRotationalCompensator::status() const {
  return status_;
}

} // namespace cddp_mpc
