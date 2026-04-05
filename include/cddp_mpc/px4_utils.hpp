#ifndef CDDP_MPC_PX4_UTILS_HPP
#define CDDP_MPC_PX4_UTILS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>

#include <Eigen/Dense>

namespace cddp_mpc {

inline constexpr double kQuadrotorModelYawMomentCoefficient = 0.1;

struct FrameAdapterConfig {
  std::string quaternion_order{"auto"};
  std::string odom_body_frame{"auto"};
  bool require_ned_pose_frame{true};
  bool require_ned_velocity_frame{true};
  int expected_pose_frame_ned{1};
  int expected_velocity_frame_ned{1};
};

struct NormalizedOdometry {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_wxyz{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  bool frame_ok{true};
  int pose_frame{0};
  int velocity_frame{0};
  std::string quat_order_used{"wxyz"};
  std::string body_frame_used{"frd"};
  double thrust_axis_world_z{0.0};
};

inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d &ned) {
  return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
}

inline Eigen::Quaterniond normalizeQuaternionWxyz(const Eigen::Vector4d &quat_wxyz) {
  Eigen::Vector4d q = quat_wxyz;
  const double norm = q.norm();
  if (norm < 1e-8) {
    return Eigen::Quaterniond::Identity();
  }
  q /= norm;
  return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
}

inline Eigen::Quaterniond quatNedToEnuWxyz(const Eigen::Quaterniond &q_ned) {
  constexpr double kSqrtHalf = 0.70710678118654757;
  const double qw = q_ned.w();
  const double qx = q_ned.x();
  const double qy = q_ned.y();
  const double qz = q_ned.z();

  Eigen::Vector4d q_enu_wxyz;
  q_enu_wxyz << kSqrtHalf * (qw + qz), kSqrtHalf * (qx + qy),
      kSqrtHalf * (qx - qy), kSqrtHalf * (qw - qz);
  return normalizeQuaternionWxyz(q_enu_wxyz);
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw_rad) {
  const double half = 0.5 * yaw_rad;
  return Eigen::Quaterniond(std::cos(half), 0.0, 0.0, std::sin(half));
}

inline Eigen::Quaterniond yawNedToQuaternionEnu(double yaw_ned_rad) {
  const double half = 0.5 * yaw_ned_rad;
  constexpr double kSqrtHalf = 0.70710678118654757;
  Eigen::Vector4d quat_wxyz;
  quat_wxyz << kSqrtHalf * (std::cos(half) + std::sin(half)), 0.0, 0.0,
      kSqrtHalf * (std::cos(half) - std::sin(half));
  return normalizeQuaternionWxyz(quat_wxyz);
}

inline Eigen::Quaterniond quaternionMultiplyWxyz(const Eigen::Quaterniond &a,
                                                 const Eigen::Quaterniond &b) {
  return Eigen::Quaterniond(
      a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z(),
      a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y(),
      a.w() * b.y() - a.x() * b.z() + a.y() * b.w() + a.z() * b.x(),
      a.w() * b.z() + a.x() * b.y() - a.y() * b.x() + a.z() * b.w());
}

inline Eigen::Vector3d bodyVectorFluToFrd(const Eigen::Vector3d &vec) {
  return Eigen::Vector3d(vec.x(), -vec.y(), -vec.z());
}

inline Eigen::Quaterniond convertQuaternionOrder(const Eigen::Vector4d &raw_q,
                                                 const std::string &requested_order,
                                                 std::string *used_order) {
  const Eigen::Quaterniond wxyz_candidate = normalizeQuaternionWxyz(raw_q);
  const Eigen::Quaterniond xyzw_candidate =
      normalizeQuaternionWxyz(Eigen::Vector4d(raw_q(3), raw_q(0), raw_q(1), raw_q(2)));

  if (requested_order == "wxyz") {
    if (used_order != nullptr) {
      *used_order = "wxyz";
    }
    return wxyz_candidate;
  }
  if (requested_order == "xyzw") {
    if (used_order != nullptr) {
      *used_order = "xyzw";
    }
    return xyzw_candidate;
  }

  if (std::abs(wxyz_candidate.w()) >= std::abs(xyzw_candidate.w())) {
    if (used_order != nullptr) {
      *used_order = "wxyz";
    }
    return wxyz_candidate;
  }
  if (used_order != nullptr) {
    *used_order = "xyzw";
  }
  return xyzw_candidate;
}

inline Eigen::Quaterniond convertBodyFrameToFrd(const Eigen::Quaterniond &quat_wxyz,
                                                const std::string &requested_body_frame,
                                                std::string *used_body_frame) {
  const Eigen::Quaterniond q_frd = quat_wxyz.normalized();
  const Eigen::Quaterniond q_flu_to_frd(0.0, 1.0, 0.0, 0.0);
  const Eigen::Quaterniond q_from_flu =
      quaternionMultiplyWxyz(q_frd, q_flu_to_frd).normalized();

  if (requested_body_frame == "frd") {
    if (used_body_frame != nullptr) {
      *used_body_frame = "frd";
    }
    return q_frd;
  }
  if (requested_body_frame == "flu") {
    if (used_body_frame != nullptr) {
      *used_body_frame = "flu";
    }
    return q_from_flu;
  }

  const Eigen::Vector3d thrust_axis_frd =
      q_frd.toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);
  const Eigen::Vector3d thrust_axis_flu =
      q_from_flu.toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);

  if (thrust_axis_flu.z() < thrust_axis_frd.z()) {
    if (used_body_frame != nullptr) {
      *used_body_frame = "flu";
    }
    return q_from_flu;
  }
  if (used_body_frame != nullptr) {
    *used_body_frame = "frd";
  }
  return q_frd;
}

inline NormalizedOdometry normalizeOdometry(
    const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &angular_velocity, const Eigen::Vector4d &raw_q,
    std::optional<int> pose_frame, std::optional<int> velocity_frame,
    const FrameAdapterConfig &config) {
  NormalizedOdometry odom;
  odom.position = position;
  odom.velocity = velocity;
  odom.pose_frame = pose_frame.value_or(0);
  odom.velocity_frame = velocity_frame.value_or(0);

  odom.attitude_wxyz =
      convertQuaternionOrder(raw_q, config.quaternion_order, &odom.quat_order_used);
  odom.attitude_wxyz = convertBodyFrameToFrd(odom.attitude_wxyz, config.odom_body_frame,
                                             &odom.body_frame_used);

  odom.angular_velocity = angular_velocity;
  if (odom.body_frame_used == "flu") {
    odom.angular_velocity = bodyVectorFluToFrd(odom.angular_velocity);
  }

  odom.frame_ok = true;
  if (config.require_ned_pose_frame && pose_frame.has_value()) {
    odom.frame_ok = odom.frame_ok && (pose_frame.value() == config.expected_pose_frame_ned);
  }
  if (config.require_ned_velocity_frame && velocity_frame.has_value()) {
    odom.frame_ok =
        odom.frame_ok && (velocity_frame.value() == config.expected_velocity_frame_ned);
  }

  const Eigen::Vector3d thrust_axis_world =
      odom.attitude_wxyz.toRotationMatrix() * Eigen::Vector3d(0.0, 0.0, -1.0);
  odom.thrust_axis_world_z = thrust_axis_world.z();
  return odom;
}

inline std::array<float, 3> thrustBodyFromCommand(double thrust_newtons,
                                                  double max_thrust_newtons,
                                                  double thrust_scale = 0.0) {
  const double scale =
      thrust_scale > 0.0 ? thrust_scale : 1.0 / std::max(max_thrust_newtons, 1e-6);
  const double thrust_body_z =
      std::clamp(-(thrust_newtons * scale), -1.0, 0.0);
  return {0.0F, 0.0F, static_cast<float>(thrust_body_z)};
}

inline Eigen::Vector4d motorForcesFromThrustTorque(double collective_thrust_newtons,
                                                   const Eigen::Vector3d &body_torque_nm,
                                                   double arm_length_m,
                                                   double yaw_moment_coefficient) {
  const double safe_arm_length = std::max(std::abs(arm_length_m), 1e-6);
  const double safe_yaw_coeff = std::max(std::abs(yaw_moment_coefficient), 1e-6);

  Eigen::Vector4d motor_forces;
  motor_forces(0) = 0.25 * collective_thrust_newtons +
                    0.5 * body_torque_nm.x() / safe_arm_length +
                    0.25 * body_torque_nm.z() / safe_yaw_coeff;
  motor_forces(1) = 0.25 * collective_thrust_newtons +
                    0.5 * body_torque_nm.y() / safe_arm_length -
                    0.25 * body_torque_nm.z() / safe_yaw_coeff;
  motor_forces(2) = 0.25 * collective_thrust_newtons -
                    0.5 * body_torque_nm.x() / safe_arm_length +
                    0.25 * body_torque_nm.z() / safe_yaw_coeff;
  motor_forces(3) = 0.25 * collective_thrust_newtons -
                    0.5 * body_torque_nm.y() / safe_arm_length -
                    0.25 * body_torque_nm.z() / safe_yaw_coeff;
  return motor_forces;
}

inline Eigen::Vector4d projectThrustTorqueToMotorLimits(
    const Eigen::Vector4d &command, double min_motor_thrust_n,
    double max_motor_thrust_n, double arm_length_m,
    double yaw_moment_coefficient) {
  Eigen::Vector4d projected = command;
  const double min_collective = 4.0 * std::max(0.0, min_motor_thrust_n);
  const double max_collective =
      4.0 * std::max(std::max(0.0, min_motor_thrust_n), max_motor_thrust_n);
  projected(0) = std::clamp(projected(0), min_collective, max_collective);

  const auto torque_is_feasible = [&](const Eigen::Vector3d &torque) {
    const Eigen::Vector4d motor_forces = motorForcesFromThrustTorque(
        projected(0), torque, arm_length_m, yaw_moment_coefficient);
    return (motor_forces.array() >= min_motor_thrust_n - 1e-9).all() &&
           (motor_forces.array() <= max_motor_thrust_n + 1e-9).all();
  };

  const Eigen::Vector3d torque = projected.tail<3>();
  if (torque_is_feasible(torque)) {
    return projected;
  }

  double lower = 0.0;
  double upper = 1.0;
  for (int i = 0; i < 32; ++i) {
    const double mid = 0.5 * (lower + upper);
    if (torque_is_feasible(mid * torque)) {
      lower = mid;
    } else {
      upper = mid;
    }
  }
  projected.tail<3>() *= lower;
  return projected;
}

inline std::array<float, 3> torqueBodyFromCommand(const Eigen::Vector3d &body_torque_nm,
                                                  const Eigen::Vector3d &torque_scale) {
  std::array<float, 3> torque_body{};
  for (int i = 0; i < 3; ++i) {
    const double scale = torque_scale(i) > 0.0 ? torque_scale(i) : 0.0;
    torque_body[static_cast<std::size_t>(i)] =
        static_cast<float>(std::clamp(body_torque_nm(i) * scale, -1.0, 1.0));
  }
  return torque_body;
}

} // namespace cddp_mpc

#endif
