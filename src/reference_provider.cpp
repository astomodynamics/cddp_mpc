#include "cddp_mpc/reference_provider.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include "cddp_mpc/px4_utils.hpp"

namespace cddp_mpc {

namespace {

double wrapAngle(double angle_rad) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

} // namespace

PositionYawReferenceProvider::PositionYawReferenceProvider(ReferenceConfig config)
    : config_(std::move(config)) {}

PositionYawReferenceProvider::AxisProfile
PositionYawReferenceProvider::buildAxisProfile(double position_error, double velocity,
                                               double target_position_error) const {
  AxisProfile profile;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::max(0, config_.horizon_steps) + 1);
  profile.positions.reserve(sample_count);
  profile.velocities.reserve(sample_count);
  profile.accelerations.reserve(sample_count);

  const double rate_limit = std::max(config_.max_axis_speed_mps, 0.05);
  const double accel_limit = std::max(config_.max_axis_accel_mps2, 0.1);
  const double dt = std::max(config_.mpc_dt, 1e-6);
  const double tau = std::max(config_.smoothing_tau_s, dt);

  double current_position = position_error;
  double current_velocity = velocity;
  profile.positions.push_back(current_position);
  profile.velocities.push_back(std::clamp(current_velocity, -rate_limit, rate_limit));
  profile.accelerations.push_back(0.0);

  for (int i = 0; i < config_.horizon_steps; ++i) {
    const double pos_error = target_position_error - current_position;
    const double desired_velocity = std::clamp(pos_error / tau, -rate_limit, rate_limit);
    const double applied_accel =
        std::clamp((desired_velocity - current_velocity) / dt, -accel_limit, accel_limit);

    double next_velocity = std::clamp(current_velocity + applied_accel * dt, -rate_limit,
                                      rate_limit);
    double next_position = current_position + next_velocity * dt;
    const bool crossed_target =
        (target_position_error - current_position) * (target_position_error - next_position) <=
        0.0;
    if (!config_.allow_reference_jump && crossed_target && std::abs(pos_error) > 1e-9) {
      next_position = target_position_error;
      next_velocity = 0.0;
    }

    current_position = next_position;
    current_velocity = next_velocity;
    profile.positions.push_back(current_position);
    profile.velocities.push_back(current_velocity);
    profile.accelerations.push_back(applied_accel);
  }

  return profile;
}

double PositionYawReferenceProvider::clampYawStep(double previous_yaw,
                                                  double desired_yaw) const {
  const double dt = std::max(config_.mpc_dt, 1e-6);
  const double max_step = std::max(config_.max_yaw_rate_rad_s, 0.05) * dt;
  const double delta = wrapAngle(desired_yaw - previous_yaw);
  return wrapAngle(previous_yaw + std::clamp(delta, -max_step, max_step));
}

PositionYawReferenceProvider::TrajectoryPoint
PositionYawReferenceProvider::buildPointFromMode(double time_s,
                                                 double previous_yaw) const {
  TrajectoryPoint point;
  const double safe_period = std::max(config_.trajectory_period_s, config_.mpc_dt);
  const double omega = 2.0 * 3.14159265358979323846 / safe_period;
  const double radius = std::max(0.0, config_.circle_radius_m);
  const double amplitude = std::max(0.0, config_.figure_eight_amplitude_m);
  const double vertical_amplitude = std::max(0.0, config_.vertical_amplitude_m);

  if (config_.mode == "circle") {
    point.position_error.x() = radius * (std::cos(omega * time_s) - 1.0);
    point.position_error.y() = radius * std::sin(omega * time_s);
    point.position_error.z() = vertical_amplitude * std::sin(omega * time_s);
    point.velocity.x() = -radius * omega * std::sin(omega * time_s);
    point.velocity.y() = radius * omega * std::cos(omega * time_s);
    point.velocity.z() = vertical_amplitude * omega * std::cos(omega * time_s);
    point.acceleration.x() = -radius * omega * omega * std::cos(omega * time_s);
    point.acceleration.y() = -radius * omega * omega * std::sin(omega * time_s);
    point.acceleration.z() =
        -vertical_amplitude * omega * omega * std::sin(omega * time_s);
  } else if (config_.mode == "figure_eight") {
    point.position_error.x() = amplitude * std::sin(omega * time_s);
    point.position_error.y() = 0.5 * amplitude * std::sin(2.0 * omega * time_s);
    point.position_error.z() = vertical_amplitude * std::sin(omega * time_s);
    point.velocity.x() = amplitude * omega * std::cos(omega * time_s);
    point.velocity.y() = amplitude * omega * std::cos(2.0 * omega * time_s);
    point.velocity.z() = vertical_amplitude * omega * std::cos(omega * time_s);
    point.acceleration.x() = -amplitude * omega * omega * std::sin(omega * time_s);
    point.acceleration.y() =
        -2.0 * amplitude * omega * omega * std::sin(2.0 * omega * time_s);
    point.acceleration.z() =
        -vertical_amplitude * omega * omega * std::sin(omega * time_s);
  }

  point.velocity =
      point.velocity.cwiseMax(Eigen::Vector3d::Constant(-config_.max_axis_speed_mps))
          .cwiseMin(Eigen::Vector3d::Constant(config_.max_axis_speed_mps));
  point.acceleration =
      point.acceleration.cwiseMax(Eigen::Vector3d::Constant(-config_.max_axis_accel_mps2))
          .cwiseMin(Eigen::Vector3d::Constant(config_.max_axis_accel_mps2));

  point.yaw_rad = clampYawStep(previous_yaw, config_.target_yaw_rad);
  return point;
}

PositionYawReferenceProvider::TrajectoryPoint
PositionYawReferenceProvider::buildPointFromProfiles(const AxisProfile &x_profile,
                                                     const AxisProfile &y_profile,
                                                     const AxisProfile &z_profile,
                                                     std::size_t idx,
                                                     double previous_yaw) const {
  TrajectoryPoint point;
  point.position_error.x() = x_profile.positions[idx];
  point.position_error.y() = y_profile.positions[idx];
  point.position_error.z() = z_profile.positions[idx];
  point.velocity.x() = x_profile.velocities[idx];
  point.velocity.y() = y_profile.velocities[idx];
  point.velocity.z() = z_profile.velocities[idx];
  point.acceleration.x() = x_profile.accelerations[idx];
  point.acceleration.y() = y_profile.accelerations[idx];
  point.acceleration.z() = z_profile.accelerations[idx];
  point.velocity =
      point.velocity.cwiseMax(Eigen::Vector3d::Constant(-config_.max_axis_speed_mps))
          .cwiseMin(Eigen::Vector3d::Constant(config_.max_axis_speed_mps));
  point.acceleration =
      point.acceleration.cwiseMax(Eigen::Vector3d::Constant(-config_.max_axis_accel_mps2))
          .cwiseMin(Eigen::Vector3d::Constant(config_.max_axis_accel_mps2));

  point.yaw_rad = clampYawStep(previous_yaw, config_.target_yaw_rad);
  return point;
}

ReferenceTrajectory
PositionYawReferenceProvider::build(const Eigen::VectorXd &initial_state) const {
  ReferenceTrajectory trajectory;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::max(0, config_.horizon_steps) + 1);
  trajectory.states.reserve(sample_count);
  trajectory.controls.reserve(sample_count);

  const bool uses_axis_profiles =
      config_.mode != "circle" && config_.mode != "figure_eight";
  AxisProfile x_profile;
  AxisProfile y_profile;
  AxisProfile z_profile;
  if (uses_axis_profiles) {
    x_profile = buildAxisProfile(initial_state(0), initial_state(7), 0.0);
    y_profile = buildAxisProfile(initial_state(1), initial_state(8), 0.0);
    z_profile = buildAxisProfile(initial_state(2), initial_state(9), 0.0);
  }

  double yaw_rad = config_.target_yaw_rad;
  for (int i = 0; i <= config_.horizon_steps; ++i) {
    const double time_s = static_cast<double>(i) * std::max(config_.mpc_dt, 1e-6);
    const TrajectoryPoint point =
        uses_axis_profiles
            ? buildPointFromProfiles(x_profile, y_profile, z_profile,
                                     static_cast<std::size_t>(i), yaw_rad)
            : buildPointFromMode(time_s, yaw_rad);
    yaw_rad = point.yaw_rad;
    const Eigen::Quaterniond q_ref = yawNedToQuaternionEnu(yaw_rad);

    Eigen::VectorXd state = Eigen::VectorXd::Zero(13);
    state.segment<3>(0) = point.position_error;
    state(3) = q_ref.w();
    state(4) = q_ref.x();
    state(5) = q_ref.y();
    state(6) = q_ref.z();
    state.segment<3>(7) = point.velocity;
    trajectory.states.push_back(state);

    Eigen::VectorXd control = Eigen::VectorXd::Zero(4);
    control(0) = std::clamp(config_.hover_thrust_n +
                                point.acceleration.z() *
                                    std::max(config_.hover_thrust_n, 1.0) / 9.81,
                            config_.min_thrust_n, config_.max_thrust_n);
    trajectory.controls.push_back(control);
  }

  return trajectory;
}

} // namespace cddp_mpc
