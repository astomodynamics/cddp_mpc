#ifndef CDDP_MPC_CONTROLLER_LOGIC_HPP
#define CDDP_MPC_CONTROLLER_LOGIC_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Dense>

#include "cddp_mpc/px4_utils.hpp"

namespace cddp_mpc {

enum class CommandSource {
  ReadyWait,
  LandDoneIdle,
  Solver,
  RecoveryHover,
  RecoveryDescent,
  LastGoodHold,
  FallbackHover,
  FallbackDescent,
};

inline const char *commandSourceLabel(CommandSource source) {
  switch (source) {
  case CommandSource::ReadyWait:
    return "ready_wait";
  case CommandSource::LandDoneIdle:
    return "land_done_idle";
  case CommandSource::Solver:
    return "solver";
  case CommandSource::RecoveryHover:
    return "recovery_hover";
  case CommandSource::RecoveryDescent:
    return "recovery_descent";
  case CommandSource::LastGoodHold:
    return "last_good_hold";
  case CommandSource::FallbackHover:
    return "fallback_hover";
  case CommandSource::FallbackDescent:
    return "fallback_descent";
  }
  return "unknown";
}

inline CommandSource selectCommandSource(const std::string &mode,
                                         bool usable_solution,
                                         int solve_fail_streak,
                                         int fallback_hold_cycles) {
  if (mode == "READY") {
    return CommandSource::ReadyWait;
  }
  if (mode == "LAND_DONE") {
    return CommandSource::LandDoneIdle;
  }
  if (usable_solution) {
    return CommandSource::Solver;
  }
  if (mode == "TAKEOFF" || mode == "HOVER" || mode == "LAND") {
    return mode == "LAND" ? CommandSource::RecoveryDescent
                          : CommandSource::RecoveryHover;
  }
  if (solve_fail_streak <= std::max(0, fallback_hold_cycles)) {
    return CommandSource::LastGoodHold;
  }
  return mode == "LAND" ? CommandSource::FallbackDescent
                        : CommandSource::FallbackHover;
}

inline Eigen::Vector3d updateTakeoffSetpoint(
    const Eigen::Vector3d &current_position_enu, double takeoff_altitude_m,
    const std::optional<Eigen::Vector3d> &existing_setpoint_enu) {
  if (!existing_setpoint_enu.has_value()) {
    return Eigen::Vector3d(current_position_enu.x(), current_position_enu.y(),
                           -takeoff_altitude_m);
  }

  Eigen::Vector3d updated = *existing_setpoint_enu;
  updated.z() = -takeoff_altitude_m;
  return updated;
}

inline double computeTakeoffMinimumThrust(
    double current_z_ned, double target_z_ned, double hover_thrust_n,
    double takeoff_min_thrust_margin_n, double settle_tolerance_m,
    double min_flight_thrust_ratio, double min_thrust_n, double max_thrust_n,
    double reference_thrust_n) {
  const double distance = std::abs(current_z_ned - target_z_ned);
  const double ramp_range = std::max(settle_tolerance_m * 3.0, 0.3);

  double floor_frac =
      std::clamp((distance - settle_tolerance_m) / ramp_range, 0.0, 1.0);
  if (current_z_ned < target_z_ned) {
    floor_frac = 0.0;
  }

  const double margin = takeoff_min_thrust_margin_n * floor_frac;
  const double base_takeoff_floor =
      hover_thrust_n * std::max(0.0, min_flight_thrust_ratio);
  const double min_thrust =
      floor_frac > 0.0 ? (hover_thrust_n + margin) : base_takeoff_floor;
  return std::clamp(std::max(min_thrust, reference_thrust_n), min_thrust_n,
                    max_thrust_n);
}

inline double computeHoverMinimumThrust(
    double hover_thrust_n, double min_flight_thrust_ratio,
    double hover_min_thrust_ratio, double takeoff_minimum_thrust,
    double hover_elapsed_s, double hover_handoff_duration_s, double min_thrust_n,
    double max_thrust_n) {
  const double base_hover_thrust = std::clamp(
      hover_thrust_n *
          std::max({1.0, min_flight_thrust_ratio, hover_min_thrust_ratio}),
      min_thrust_n, max_thrust_n);
  if (!std::isfinite(hover_elapsed_s) || hover_handoff_duration_s <= 1e-6) {
    return base_hover_thrust;
  }

  const double handoff_target =
      std::clamp(std::max(base_hover_thrust, takeoff_minimum_thrust),
                 min_thrust_n, max_thrust_n);
  const double alpha = std::clamp(
      1.0 - std::max(0.0, hover_elapsed_s) / hover_handoff_duration_s, 0.0, 1.0);
  return std::clamp(base_hover_thrust + alpha * (handoff_target - base_hover_thrust),
                    min_thrust_n, max_thrust_n);
}

inline Eigen::Vector4d shapeSolverCommand(
    const Eigen::Vector4d &command, const Eigen::Vector4d &last_published_command,
    CommandSource source, const std::string &mode, double blend_alpha,
    double hover_rate_limit_rad_s, double max_body_rate_rad_s,
    double hover_thrust_n, double min_flight_thrust_ratio,
    double hover_min_thrust_ratio, double position_error_norm) {
  Eigen::Vector4d shaped = command;
  if (source == CommandSource::Solver && mode != "TAKEOFF") {
    const double alpha = std::clamp(blend_alpha, 0.0, 1.0);
    shaped = last_published_command + alpha * (shaped - last_published_command);
  }

  if (mode == "HOVER" || mode == "LAND") {
    if (mode == "HOVER") {
      const double min_hover_thrust = std::clamp(
          hover_thrust_n *
              std::max({1.0, min_flight_thrust_ratio, hover_min_thrust_ratio}),
          0.0, std::numeric_limits<double>::infinity());
      shaped(0) = std::max(shaped(0), min_hover_thrust);
    }

    const double rate_limit_base =
        std::clamp(hover_rate_limit_rad_s, 0.0, max_body_rate_rad_s);
    const double error_scale = std::clamp(position_error_norm / 1.0, 0.15, 1.0);
    const double effective_rate_limit =
        std::clamp(rate_limit_base * error_scale, 0.0, max_body_rate_rad_s);
    for (int i = 1; i < 4; ++i) {
      shaped(i) =
          std::clamp(shaped(i), -effective_rate_limit, effective_rate_limit);
    }
  }
  return shaped;
}

inline Eigen::Vector4d applyFlightThrustFloor(
    const Eigen::Vector4d &command, const std::string &mode,
    double hover_thrust_n, double min_flight_thrust_ratio,
    double hover_min_thrust_ratio, double landing_min_thrust_ratio,
    double takeoff_minimum_thrust, double hover_elapsed_s,
    double hover_handoff_duration_s, double min_thrust_n, double max_thrust_n) {
  Eigen::Vector4d guarded = command;
  if (mode == "TAKEOFF") {
    guarded(0) = std::max(guarded(0), takeoff_minimum_thrust);
  }
  if (!(mode == "TAKEOFF" || mode == "HOVER" || mode == "LAND")) {
    return guarded;
  }

  double flight_ratio =
      mode == "LAND" ? std::max(0.0, landing_min_thrust_ratio)
                     : std::max(0.0, min_flight_thrust_ratio);
  if (mode == "HOVER") {
    const double min_hover_thrust = computeHoverMinimumThrust(
        hover_thrust_n, flight_ratio, hover_min_thrust_ratio,
        takeoff_minimum_thrust, hover_elapsed_s, hover_handoff_duration_s,
        min_thrust_n, max_thrust_n);
    guarded(0) = std::max(guarded(0), min_hover_thrust);
    return guarded;
  }
  const double min_flight_thrust =
      std::clamp(hover_thrust_n * flight_ratio, min_thrust_n, max_thrust_n);
  guarded(0) = std::max(guarded(0), min_flight_thrust);
  return guarded;
}

inline double computeHoverVerticalCorrection(double position_error_enu_z,
                                            double velocity_enu_z,
                                            double proportional_gain,
                                            double damping_gain,
                                            double max_correction_n) {
  const double raw_correction =
      proportional_gain * position_error_enu_z - damping_gain * velocity_enu_z;
  return std::clamp(raw_correction, -std::abs(max_correction_n),
                    std::abs(max_correction_n));
}

inline Eigen::Vector4d applyHoverVerticalCorrection(
    const Eigen::Vector4d &command, const std::string &mode,
    double position_error_enu_z, double velocity_enu_z,
    double proportional_gain, double damping_gain, double max_correction_n,
    double min_thrust_n, double max_thrust_n) {
  if (mode != "HOVER") {
    return command;
  }

  Eigen::Vector4d corrected = command;
  corrected(0) = std::clamp(
      corrected(0) + computeHoverVerticalCorrection(
                          position_error_enu_z, velocity_enu_z, proportional_gain,
                          damping_gain, max_correction_n),
      min_thrust_n, max_thrust_n);
  return corrected;
}

inline Eigen::Vector4d applyHoverLateralCorrection(
    const Eigen::Vector4d &command, const std::string &mode,
    double body_forward_error_m, double body_right_error_m,
    double body_forward_velocity_mps, double body_right_velocity_mps,
    double proportional_gain, double damping_gain, double max_rate_correction_rad_s,
    double max_body_rate_rad_s) {
  if (mode != "HOVER") {
    return command;
  }

  const double forward_correction =
      proportional_gain * body_forward_error_m -
      damping_gain * body_forward_velocity_mps;
  const double right_correction =
      proportional_gain * body_right_error_m -
      damping_gain * body_right_velocity_mps;

  Eigen::Vector4d corrected = command;
  corrected(1) = std::clamp(
      corrected(1) +
          std::clamp(right_correction, -std::abs(max_rate_correction_rad_s),
                     std::abs(max_rate_correction_rad_s)),
      -max_body_rate_rad_s, max_body_rate_rad_s);
  corrected(2) = std::clamp(
      corrected(2) -
          std::clamp(forward_correction, -std::abs(max_rate_correction_rad_s),
                     std::abs(max_rate_correction_rad_s)),
      -max_body_rate_rad_s, max_body_rate_rad_s);
  return corrected;
}

inline Eigen::Vector4d shapeSolverThrustCommand(
    const Eigen::Vector4d &command, const Eigen::Vector4d &last_published_command,
    CommandSource source, const std::string &mode, double blend_alpha,
    const Eigen::Vector3d &hover_torque_limit_nm,
    const Eigen::Vector3d &max_body_torque_nm, double hover_thrust_n,
    double min_flight_thrust_ratio, double hover_min_thrust_ratio,
    double position_error_norm) {
  Eigen::Vector4d shaped = command;
  if (source == CommandSource::Solver && mode != "TAKEOFF") {
    const double alpha = std::clamp(blend_alpha, 0.0, 1.0);
    shaped = last_published_command + alpha * (shaped - last_published_command);
  }

  if (mode == "HOVER" || mode == "LAND") {
    if (mode == "HOVER") {
      const double min_hover_thrust = std::clamp(
          hover_thrust_n *
              std::max({1.0, min_flight_thrust_ratio, hover_min_thrust_ratio}),
          0.0, std::numeric_limits<double>::infinity());
      shaped(0) = std::max(shaped(0), min_hover_thrust);
    }

    const double error_scale = std::clamp(position_error_norm / 1.0, 0.4, 1.0);
    const Eigen::Vector3d effective_limit =
        hover_torque_limit_nm.cwiseAbs().cwiseMin(max_body_torque_nm.cwiseAbs()) *
        error_scale;
    for (int i = 0; i < 3; ++i) {
      shaped(i + 1) =
          std::clamp(shaped(i + 1), -effective_limit(i), effective_limit(i));
    }
  }

  return shaped;
}

inline Eigen::Vector4d applyHoverLateralTorqueCorrection(
    const Eigen::Vector4d &command, const std::string &mode,
    double body_forward_error_m, double body_right_error_m,
    double body_forward_velocity_mps, double body_right_velocity_mps,
    double proportional_gain, double damping_gain,
    const Eigen::Vector3d &max_torque_correction_nm,
    const Eigen::Vector3d &max_body_torque_nm) {
  if (mode != "HOVER") {
    return command;
  }

  const double forward_correction =
      proportional_gain * body_forward_error_m -
      damping_gain * body_forward_velocity_mps;
  const double right_correction =
      proportional_gain * body_right_error_m -
      damping_gain * body_right_velocity_mps;

  Eigen::Vector4d corrected = command;
  corrected(1) = std::clamp(
      corrected(1) +
          std::clamp(right_correction, -std::abs(max_torque_correction_nm.x()),
                     std::abs(max_torque_correction_nm.x())),
      -std::abs(max_body_torque_nm.x()), std::abs(max_body_torque_nm.x()));
  corrected(2) = std::clamp(
      corrected(2) -
          std::clamp(forward_correction, -std::abs(max_torque_correction_nm.y()),
                     std::abs(max_torque_correction_nm.y())),
      -std::abs(max_body_torque_nm.y()), std::abs(max_body_torque_nm.y()));
  corrected(3) = std::clamp(corrected(3), -std::abs(max_body_torque_nm.z()),
                            std::abs(max_body_torque_nm.z()));
  return corrected;
}

inline bool shouldResetWarmStartThrustModel(
    const Eigen::VectorXd &initial_state, const Eigen::VectorXd &expected_state,
    double mpc_dt, double reference_max_climb_rate_mps,
    double reference_max_climb_accel_mps2) {
  if (!expected_state.allFinite() || !initial_state.allFinite()) {
    return true;
  }

  const double position_error =
      (initial_state.segment<3>(0) - expected_state.segment<3>(0)).norm();
  const double velocity_error =
      (initial_state.segment<3>(7) - expected_state.segment<3>(7)).norm();
  const double angular_rate_error =
      (initial_state.segment<3>(10) - expected_state.segment<3>(10)).norm();

  const Eigen::Quaterniond quat_now =
      normalizeQuaternionWxyz(initial_state.segment<4>(3));
  const Eigen::Quaterniond quat_expected =
      normalizeQuaternionWxyz(expected_state.segment<4>(3));
  const double attitude_alignment = std::abs(quat_now.w() * quat_expected.w() +
                                             quat_now.x() * quat_expected.x() +
                                             quat_now.y() * quat_expected.y() +
                                             quat_now.z() * quat_expected.z());

  const double position_reset_threshold =
      std::max(0.75, 5.0 * mpc_dt * reference_max_climb_rate_mps);
  const double velocity_reset_threshold =
      std::max(1.5, 2.0 * reference_max_climb_accel_mps2);
  const double angular_rate_reset_threshold = 0.6;

  return position_error > position_reset_threshold ||
         velocity_error > velocity_reset_threshold ||
         angular_rate_error > angular_rate_reset_threshold ||
         attitude_alignment < 0.9;
}

inline bool shouldResetWarmStart(const Eigen::VectorXd &initial_state,
                                 const Eigen::VectorXd &expected_state,
                                 double mpc_dt,
                                 double reference_max_climb_rate_mps,
                                 double reference_max_climb_accel_mps2) {
  if (!expected_state.allFinite() || !initial_state.allFinite()) {
    return true;
  }

  const double position_error =
      (initial_state.segment<3>(0) - expected_state.segment<3>(0)).norm();
  const double velocity_error =
      (initial_state.segment<3>(3) - expected_state.segment<3>(3)).norm();

  const Eigen::Quaterniond quat_now = normalizeQuaternionWxyz(initial_state.segment<4>(6));
  const Eigen::Quaterniond quat_expected =
      normalizeQuaternionWxyz(expected_state.segment<4>(6));
  const double attitude_alignment = std::abs(quat_now.w() * quat_expected.w() +
                                             quat_now.x() * quat_expected.x() +
                                             quat_now.y() * quat_expected.y() +
                                             quat_now.z() * quat_expected.z());

  const double position_reset_threshold =
      std::max(0.75, 5.0 * mpc_dt * reference_max_climb_rate_mps);
  const double velocity_reset_threshold =
      std::max(1.5, 2.0 * reference_max_climb_accel_mps2);

  return position_error > position_reset_threshold ||
         velocity_error > velocity_reset_threshold ||
         attitude_alignment < 0.9;
}

} // namespace cddp_mpc

#endif
