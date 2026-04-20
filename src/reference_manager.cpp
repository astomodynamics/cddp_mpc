#include "cddp_mpc/reference_manager.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

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

double clampAbs(double value, double limit) {
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

const char *sourceLabel(ReferenceSource source) {
  switch (source) {
  case ReferenceSource::Mission:
    return "mission";
  case ReferenceSource::GoalPose:
    return "goal_pose";
  case ReferenceSource::Teleop:
    return "teleop";
  }
  return "unknown";
}

double advanceAxis(double current_position, double *current_velocity,
                   double desired_position, double tau_s, double max_speed,
                   double max_accel, double dt_s) {
  if (dt_s <= 0.0) {
    return current_position;
  }

  const double desired_velocity = clampAbs(
      (desired_position - current_position) / std::max(tau_s, dt_s), max_speed);
  const double velocity_error = desired_velocity - *current_velocity;
  *current_velocity += clampAbs(velocity_error, max_accel * dt_s);
  *current_velocity = clampAbs(*current_velocity, max_speed);

  const double next_position = current_position + (*current_velocity) * dt_s;
  const bool crossed_target =
      (desired_position - current_position) * (desired_position - next_position) <= 0.0;
  if (crossed_target || std::abs(desired_position - next_position) <= 1e-4) {
    *current_velocity = 0.0;
    return desired_position;
  }
  return next_position;
}

} // namespace

ReferenceManager::ReferenceManager(ReferenceManagerConfig config)
    : config_(std::move(config)) {}

void ReferenceManager::setConfig(const ReferenceManagerConfig &config) {
  config_ = config;
}

void ReferenceManager::updateTeleopCommand(double time_s,
                                           const Eigen::Vector3d &linear_velocity,
                                           double yaw_rate_rad_s,
                                           TeleopFrame frame) {
  TeleopCommand command;
  command.time_s = time_s;
  command.linear_velocity.x() = clampAbs(linear_velocity.x(), config_.teleop_max_xy_speed_mps);
  command.linear_velocity.y() = clampAbs(linear_velocity.y(), config_.teleop_max_xy_speed_mps);
  command.linear_velocity.z() = clampAbs(linear_velocity.z(), config_.teleop_max_z_speed_mps);
  command.yaw_rate_rad_s =
      clampAbs(yaw_rate_rad_s, config_.teleop_max_yaw_rate_rad_s);
  command.frame = frame;
  latest_teleop_command_ = command;
}

void ReferenceManager::updateDeadmanState(double time_s, bool pressed) {
  static_cast<void>(time_s);
  deadman_pressed_ = pressed;
}

void ReferenceManager::updateGoalPose(double time_s,
                                      const Eigen::Vector3d &position_enu,
                                      double yaw_rad) {
  GoalPose goal;
  goal.time_s = time_s;
  goal.position_enu = position_enu;
  goal.yaw_rad = wrapAngle(yaw_rad);
  latest_goal_pose_ = goal;
}

ReferenceStatus ReferenceManager::update(double time_s,
                                         const Eigen::Vector3d &current_position_enu,
                                         double current_yaw_rad,
                                         const MissionReference &mission_reference) {
  const bool teleop_fresh =
      latest_teleop_command_.has_value() &&
      (time_s - latest_teleop_command_->time_s) <= std::max(config_.teleop_timeout_s, 0.0);
  const bool teleop_deadman_ok =
      !config_.teleop_require_deadman || deadman_pressed_;
  const bool teleop_active = teleop_fresh && teleop_deadman_ok;
  const bool goal_fresh = latest_goal_pose_.has_value() &&
                          (time_s - latest_goal_pose_->time_s) <=
                              std::max(config_.goal_timeout_s, 0.0);
  const bool goal_active = goal_fresh;

  const double dt_s = last_update_time_s_.has_value()
                          ? std::clamp(time_s - *last_update_time_s_, 0.0, 0.5)
                          : 0.0;
  last_update_time_s_ = time_s;

  Eigen::Vector3d desired_position_enu = mission_reference.target_position_enu;
  double desired_yaw_rad = wrapAngle(mission_reference.target_yaw_rad);
  ReferenceSource source = ReferenceSource::Mission;

  if (teleop_active) {
    source = ReferenceSource::Teleop;
    const Eigen::Vector3d reference_origin =
        initialized_ ? active_target_position_enu_ : current_position_enu;
    desired_position_enu = reference_origin + teleopVelocityWorld(current_yaw_rad) * dt_s;
    const double yaw_origin = initialized_ ? active_target_yaw_rad_ : current_yaw_rad;
    desired_yaw_rad = wrapAngle(
        yaw_origin + latest_teleop_command_->yaw_rate_rad_s * dt_s);
  } else if (goal_active) {
    source = ReferenceSource::GoalPose;
    desired_position_enu = latest_goal_pose_->position_enu;
    desired_yaw_rad = latest_goal_pose_->yaw_rad;
  }

  bool geofence_clamped = false;
  desired_position_enu = clampToGeofence(desired_position_enu, &geofence_clamped);

  if (!initialized_) {
    initializeTarget(desired_position_enu, desired_yaw_rad);
  } else {
    advanceTarget(desired_position_enu, desired_yaw_rad, dt_s);
  }

  ReferenceStatus status;
  status.source = source;
  status.source_label = sourceLabel(source);
  status.target_position_enu = active_target_position_enu_;
  status.target_yaw_rad = active_target_yaw_rad_;
  status.teleop_active = teleop_active;
  status.teleop_deadman_pressed = deadman_pressed_;
  status.goal_fresh = goal_fresh;
  status.geofence_clamped = geofence_clamped;
  return status;
}

void ReferenceManager::initializeTarget(const Eigen::Vector3d &position_enu,
                                        double yaw_rad) {
  active_target_position_enu_ = position_enu;
  active_target_velocity_enu_.setZero();
  active_target_yaw_rad_ = wrapAngle(yaw_rad);
  initialized_ = true;
}

Eigen::Vector3d ReferenceManager::clampToGeofence(const Eigen::Vector3d &position_enu,
                                                  bool *clamped) const {
  Eigen::Vector3d clamped_position = position_enu;
  clamped_position.x() = std::clamp(position_enu.x(), -config_.geofence_half_extent_x_m,
                                    config_.geofence_half_extent_x_m);
  clamped_position.y() = std::clamp(position_enu.y(), -config_.geofence_half_extent_y_m,
                                    config_.geofence_half_extent_y_m);
  clamped_position.z() = std::clamp(position_enu.z(), config_.geofence_min_z_m,
                                    config_.geofence_max_z_m);
  if (clamped != nullptr) {
    *clamped = !clamped_position.isApprox(position_enu, 1e-9);
  }
  return clamped_position;
}

Eigen::Vector3d ReferenceManager::teleopVelocityWorld(double current_yaw_rad) const {
  if (!latest_teleop_command_.has_value()) {
    return Eigen::Vector3d::Zero();
  }

  if (latest_teleop_command_->frame == TeleopFrame::World) {
    return latest_teleop_command_->linear_velocity;
  }

  const double cos_yaw = std::cos(current_yaw_rad);
  const double sin_yaw = std::sin(current_yaw_rad);
  const Eigen::Vector3d &body_velocity = latest_teleop_command_->linear_velocity;

  Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
  world_velocity.x() = cos_yaw * body_velocity.x() - sin_yaw * body_velocity.y();
  world_velocity.y() = sin_yaw * body_velocity.x() + cos_yaw * body_velocity.y();
  world_velocity.z() = body_velocity.z();
  return world_velocity;
}

void ReferenceManager::advanceTarget(const Eigen::Vector3d &desired_position_enu,
                                     double desired_yaw_rad, double dt_s) {
  const double tau_s = std::max(config_.smoothing_tau_s, 1e-3);
  const double xy_speed = std::max(config_.max_xy_speed_mps, 0.0);
  const double z_speed = std::max(config_.max_z_speed_mps, 0.0);
  const double xy_accel = std::max(config_.max_xy_accel_mps2, 0.0);
  const double z_accel = std::max(config_.max_z_accel_mps2, 0.0);

  active_target_position_enu_.x() = advanceAxis(
      active_target_position_enu_.x(), &active_target_velocity_enu_.x(),
      desired_position_enu.x(), tau_s, xy_speed, xy_accel, dt_s);
  active_target_position_enu_.y() = advanceAxis(
      active_target_position_enu_.y(), &active_target_velocity_enu_.y(),
      desired_position_enu.y(), tau_s, xy_speed, xy_accel, dt_s);
  active_target_position_enu_.z() = advanceAxis(
      active_target_position_enu_.z(), &active_target_velocity_enu_.z(),
      desired_position_enu.z(), tau_s, z_speed, z_accel, dt_s);

  if (dt_s <= 0.0) {
    active_target_yaw_rad_ = wrapAngle(desired_yaw_rad);
    return;
  }

  const double max_yaw_step = std::max(config_.max_yaw_rate_rad_s, 0.0) * dt_s;
  const double delta = wrapAngle(desired_yaw_rad - active_target_yaw_rad_);
  active_target_yaw_rad_ = wrapAngle(active_target_yaw_rad_ +
                                     std::clamp(delta, -max_yaw_step, max_yaw_step));
}

} // namespace cddp_mpc
