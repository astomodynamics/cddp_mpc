#include <algorithm>
#include <array>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <Eigen/Dense>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#if __has_include(<px4_msgs/msg/vehicle_angular_velocity.hpp>)
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#define CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY 1
#else
#define CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY 0
#endif
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cddp.hpp"
#include "cddp_mpc/controller_logic.hpp"
#include "cddp_mpc/error_state_quadrotor_thrust.hpp"
#include "cddp_mpc/indi_rotational_compensator.hpp"
#include "cddp_mpc/px4_utils.hpp"
#include "cddp_mpc/reference_provider.hpp"
#include "cddp_mpc/thrust_allocation_constraint.hpp"

using namespace std::chrono_literals;

namespace {

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleThrustSetpoint;
using px4_msgs::msg::VehicleTorqueSetpoint;
using px4_msgs::msg::VehicleStatus;
#if CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY
using px4_msgs::msg::VehicleAngularVelocity;
#endif
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using cddp_mpc::ReferenceTrajectory;

std::string formatDouble(double value, int precision = 3) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

struct SolveResult {
  std::uint64_t request_id{0};
  bool success{false};
  bool usable_solution{false};
  bool converged{false};
  std::string status_message{"not_started"};
  double solve_time_ms{0.0};
  double final_objective{std::numeric_limits<double>::quiet_NaN()};
  double max_constraint_violation{std::numeric_limits<double>::quiet_NaN()};
  double raw_bound_violation{std::numeric_limits<double>::quiet_NaN()};
  double final_primal_infeasibility{std::numeric_limits<double>::quiet_NaN()};
  double final_dual_infeasibility{std::numeric_limits<double>::quiet_NaN()};
  double final_complementary_infeasibility{
      std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector4d command{Eigen::Vector4d::Zero()};
  std::vector<Eigen::VectorXd> state_trajectory;
  std::vector<Eigen::VectorXd> control_trajectory;
};

struct PendingSolve {
  std::uint64_t request_id{0};
  double start_time_s{0.0};
  bool timeout_reported{false};
  std::future<SolveResult> future;
};

struct SafetyTrace {
  std::string last_modified_layer{"none"};
  bool fallback_active{false};
  bool projection_changed_command{false};
};

class TrackingQuadraticObjective : public cddp::Objective {
public:
  TrackingQuadraticObjective(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                             const Eigen::MatrixXd &Qf,
                             const Eigen::VectorXd &reference_state,
                             std::vector<Eigen::VectorXd> reference_states,
                             std::vector<Eigen::VectorXd> reference_controls,
                             double timestep)
      : Q_(Q * timestep),
        R_(R * timestep),
        Qf_(Qf),
        timestep_(timestep),
        reference_controls_(std::move(reference_controls)) {
    reference_state_ = reference_state;
    reference_states_ = std::move(reference_states);
  }

  double evaluate(const std::vector<Eigen::VectorXd> &states,
                  const std::vector<Eigen::VectorXd> &controls) const override {
    double total_cost = 0.0;
    for (std::size_t t = 0; t + 1 < states.size(); ++t) {
      total_cost += running_cost(states[t], controls[t], static_cast<int>(t));
    }
    total_cost += terminal_cost(states.back());
    return total_cost;
  }

  double running_cost(const Eigen::VectorXd &state, const Eigen::VectorXd &control,
                      int index) const override {
    const Eigen::VectorXd state_error = state - referenceStateAt(index);
    const Eigen::VectorXd control_error = control - referenceControlAt(index);
    return ((state_error.transpose() * Q_ * state_error).value() +
            (control_error.transpose() * R_ * control_error).value());
  }

  double terminal_cost(const Eigen::VectorXd &final_state) const override {
    const Eigen::VectorXd state_error = final_state - reference_state_;
    return (state_error.transpose() * Qf_ * state_error).value();
  }

  Eigen::VectorXd getRunningCostStateGradient(const Eigen::VectorXd &state,
                                              const Eigen::VectorXd &control,
                                              int index) const override {
    return 2.0 * Q_ * (state - referenceStateAt(index));
  }

  Eigen::VectorXd getRunningCostControlGradient(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                int index) const override {
    return 2.0 * R_ * (control - referenceControlAt(index));
  }

  Eigen::VectorXd
  getFinalCostGradient(const Eigen::VectorXd &final_state) const override {
    return 2.0 * Qf_ * (final_state - reference_state_);
  }

  Eigen::MatrixXd getRunningCostStateHessian(const Eigen::VectorXd &state,
                                             const Eigen::VectorXd &control,
                                             int index) const override {
    return 2.0 * Q_;
  }

  Eigen::MatrixXd getRunningCostControlHessian(const Eigen::VectorXd &state,
                                               const Eigen::VectorXd &control,
                                               int index) const override {
    return 2.0 * R_;
  }

  Eigen::MatrixXd getRunningCostCrossHessian(const Eigen::VectorXd &state,
                                             const Eigen::VectorXd &control,
                                             int index) const override {
    return Eigen::MatrixXd::Zero(control.size(), state.size());
  }

  Eigen::MatrixXd
  getFinalCostHessian(const Eigen::VectorXd &final_state) const override {
    return 2.0 * Qf_;
  }

private:
  const Eigen::VectorXd &referenceStateAt(int index) const {
    if (reference_states_.empty()) {
      return reference_state_;
    }
    const std::size_t clamped_index = static_cast<std::size_t>(
        std::clamp(index, 0, static_cast<int>(reference_states_.size() - 1)));
    return reference_states_[clamped_index];
  }

  const Eigen::VectorXd &referenceControlAt(int index) const {
    const std::size_t clamped_index = static_cast<std::size_t>(
        std::clamp(index, 0, static_cast<int>(reference_controls_.size() - 1)));
    return reference_controls_[clamped_index];
  }

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Qf_;
  double timestep_{0.1};
  std::vector<Eigen::VectorXd> reference_controls_;
};

class PX4MPCNode : public rclcpp::Node {
public:
  explicit PX4MPCNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("px4_mpc_node", options),
        mission_start_requested_(auto_start_sequence_),
        last_solver_command_(hoverCommand(0.0)),
        last_command_(hoverCommand(0.0)),
        last_good_command_(hoverCommand(0.0)),
        last_published_command_(hoverCommand(0.0)) {
    loadParameters();
    mission_start_requested_ = auto_start_sequence_;
    hover_thrust_n_ = hover_thrust_n_ > 0.0 ? hover_thrust_n_ : mass_kg_ * gravity_mps2_;
    reference_provider_ = std::make_unique<cddp_mpc::PositionYawReferenceProvider>(
        buildReferenceConfig());
    rotational_indi_ =
        std::make_unique<cddp_mpc::IndiRotationalCompensator>(inertiaMatrix());
    rotational_indi_->setConfig(buildIndiConfig());
    last_solver_command_ = hoverCommand(0.0);
    last_command_ = last_solver_command_;
    last_good_command_ = last_solver_command_;
    last_published_command_ = last_solver_command_;
    last_nominal_command_ = last_solver_command_;
    last_corrected_command_ = last_solver_command_;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    local_position_sub_ = create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos,
        std::bind(&PX4MPCNode::localPositionCallback, this, std::placeholders::_1));
    odometry_sub_ = create_subscription<VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos,
        std::bind(&PX4MPCNode::odometryCallback, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos,
        std::bind(&PX4MPCNode::attitudeCallback, this, std::placeholders::_1));
    vehicle_status_sub_ = create_subscription<VehicleStatus>(
        "/fmu/out/vehicle_status", qos,
        std::bind(&PX4MPCNode::vehicleStatusCallback, this, std::placeholders::_1));
#if CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY
    angular_velocity_sub_ = create_subscription<VehicleAngularVelocity>(
        "/fmu/out/vehicle_angular_velocity", qos,
        std::bind(&PX4MPCNode::angularVelocityCallback, this, std::placeholders::_1));
#endif

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    vehicle_thrust_pub_ = create_publisher<VehicleThrustSetpoint>(
        "/fmu/in/vehicle_thrust_setpoint", qos);
    vehicle_torque_pub_ = create_publisher<VehicleTorqueSetpoint>(
        "/fmu/in/vehicle_torque_setpoint", qos);
    vehicle_command_pub_ = create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", qos);
    diagnostic_pub_ =
        create_publisher<DiagnosticArray>("/cddp_mpc/status", rclcpp::QoS(10));

    start_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "/cddp_mpc/start_mission",
        std::bind(&PX4MPCNode::handleStartMission, this, std::placeholders::_1,
                  std::placeholders::_2));

    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1e-3)),
        std::bind(&PX4MPCNode::controlLoop, this));
    solve_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(solve_rate_hz_, 1e-3)),
        std::bind(&PX4MPCNode::solveLoop, this));
    offboard_timer_ = create_wall_timer(
        100ms, std::bind(&PX4MPCNode::publishOffboardControlMode, this));

    RCLCPP_INFO(get_logger(),
                "Initialized PX4 CDDP MPC node (control=%.1f Hz, solve=%.1f Hz, "
                "horizon=%d, dt=%.3f s)",
                control_rate_hz_, solve_rate_hz_, horizon_steps_, mpc_dt_);
    if (!auto_start_sequence_) {
      RCLCPP_INFO(get_logger(),
                  "Mission start is gated. Call /cddp_mpc/start_mission to begin.");
    }
    if (indi_use_rpm_feedback_) {
      RCLCPP_WARN(get_logger(),
                  "indi_use_rpm_feedback=true, but ESC RPM feedback is not wired in this "
                  "node yet. Falling back to commanded torque estimation.");
    }
  }

private:
  void loadParameters() {
    control_rate_hz_ = declareOrGet("control_rate_hz", 100.0);
    solve_rate_hz_ = declareOrGet("solve_rate_hz", 20.0);
    mpc_dt_ = declareOrGet("mpc_dt", 0.05);
    horizon_steps_ = declareOrGet("horizon_steps", 20);
    stale_plan_timeout_s_ = declareOrGet("stale_plan_timeout_s", 0.25);
    solve_overrun_hover_cycles_threshold_ =
        declareOrGet("solve_overrun_hover_cycles_threshold", 3);

    takeoff_altitude_m_ = declareOrGet("takeoff_altitude_m", -3.0);
    hover_duration_s_ = declareOrGet("hover_duration_s", 20.0);
    settle_tolerance_m_ = declareOrGet("settle_tolerance_m", 0.3);
    hover_entry_vz_threshold_mps_ =
        declareOrGet("hover_entry_vz_threshold_mps", 0.35);
    hover_entry_rate_threshold_rad_s_ =
        declareOrGet("hover_entry_rate_threshold_rad_s", 0.3);
    hover_entry_dwell_s_ = declareOrGet("hover_entry_dwell_s", 0.6);
    hover_handoff_duration_s_ = declareOrGet("hover_handoff_duration_s", 1.5);
    hover_reenter_error_m_ = declareOrGet("hover_reenter_error_m", 1.2);
    hover_reenter_dwell_s_ = declareOrGet("hover_reenter_dwell_s", 2.0);
    landing_enabled_ = declareOrGet("landing_enabled", true);
    landing_descent_rate_mps_ = declareOrGet("landing_descent_rate_mps", 0.3);
    landing_touchdown_tolerance_m_ =
        declareOrGet("landing_touchdown_tolerance_m", 0.2);
    landing_touchdown_vz_threshold_mps_ =
        declareOrGet("landing_touchdown_vz_threshold_mps", 0.2);
    ready_thrust_n_ = declareOrGet("ready_thrust_n", 0.0);
    fallback_hold_cycles_ = declareOrGet("fallback_hold_cycles", 5);
    fallback_descent_rate_mps_ =
        declareOrGet("fallback_descent_rate_mps", 0.2);
    solver_command_blend_alpha_ =
        declareOrGet("solver_command_blend_alpha", 0.3);
    hover_vertical_kp_n_per_m_ =
        declareOrGet("hover_vertical_kp_n_per_m", 0.0);
    hover_vertical_kd_n_per_mps_ =
        declareOrGet("hover_vertical_kd_n_per_mps", 0.0);
    hover_vertical_correction_limit_n_ =
        declareOrGet("hover_vertical_correction_limit_n", 0.0);
    hover_lateral_kp_nm_per_m_ =
        declareOrGet("hover_lateral_kp_nm_per_m", 0.0);
    hover_lateral_kd_nm_per_mps_ =
        declareOrGet("hover_lateral_kd_nm_per_mps", 0.0);
    hover_lateral_correction_limit_nm_ =
        declareOrGet("hover_lateral_correction_limit_nm", 0.0);
    hover_torque_limit_nm_ = declareOrGet("hover_torque_limit_nm", 0.12);

    auto_start_sequence_ = declareOrGet("auto_start_sequence", true);
    auto_engage_offboard_ = declareOrGet("auto_engage_offboard", true);
    auto_arm_ = declareOrGet("auto_arm", true);
    use_odometry_state_ = declareOrGet("use_odometry_state", true);

    mass_kg_ = declareOrGet("mass_kg", 1.35);
    gravity_mps2_ = declareOrGet("gravity_mps2", 9.81);
    hover_thrust_n_ = declareOrGet("hover_thrust_n", 0.0);
    min_thrust_n_ = declareOrGet("min_thrust_n", 0.0);
    max_thrust_n_ = declareOrGet("max_thrust_n", 20.0);
    thrust_norm_scale_ = declareOrGet("thrust_norm_scale", 0.0);
    arm_length_m_ = declareOrGet("arm_length_m", 0.25);
    inertia_xx_kgm2_ = declareOrGet("inertia_xx_kgm2", 0.029);
    inertia_yy_kgm2_ = declareOrGet("inertia_yy_kgm2", 0.029);
    inertia_zz_kgm2_ = declareOrGet("inertia_zz_kgm2", 0.055);
    yaw_moment_coefficient_ =
        declareOrGet("yaw_moment_coefficient", 0.1);
    min_motor_thrust_n_ = declareOrGet("min_motor_thrust_n", 0.0);
    max_motor_thrust_n_ = declareOrGet("max_motor_thrust_n", 0.0);
    max_roll_torque_nm_ = declareOrGet("max_roll_torque_nm", 0.0);
    max_pitch_torque_nm_ = declareOrGet("max_pitch_torque_nm", 0.0);
    max_yaw_torque_nm_ = declareOrGet("max_yaw_torque_nm", 0.0);
    torque_norm_scale_x_ = declareOrGet("torque_norm_scale_x", 0.0);
    torque_norm_scale_y_ = declareOrGet("torque_norm_scale_y", 0.0);
    torque_norm_scale_z_ = declareOrGet("torque_norm_scale_z", 0.0);
    target_yaw_rad_ = declareOrGet("target_yaw_rad", 0.0);
    solver_type_ = declareOrGet("solver_type", std::string("CLDDP"));
    reference_max_climb_rate_mps_ =
        declareOrGet("reference_max_climb_rate_mps", 1.0);
    reference_max_climb_accel_mps2_ =
        declareOrGet("reference_max_climb_accel_mps2", 1.5);
    reference_smoothing_tau_s_ =
        declareOrGet("reference_smoothing_tau_s", 0.6);
    reference_mode_ = declareOrGet("reference_mode", std::string("active_setpoint"));
    reference_topic_ = declareOrGet("reference_topic", std::string(""));
    reference_max_yaw_rate_rad_s_ =
        declareOrGet("reference_max_yaw_rate_rad_s", 0.6);
    reference_trajectory_period_s_ =
        declareOrGet("reference_trajectory_period_s", 8.0);
    reference_circle_radius_m_ =
        declareOrGet("reference_circle_radius_m", 0.5);
    reference_figure_eight_amplitude_m_ =
        declareOrGet("reference_figure_eight_amplitude_m", 0.5);
    reference_vertical_amplitude_m_ =
        declareOrGet("reference_vertical_amplitude_m", 0.0);
    reference_allow_jump_ = declareOrGet("reference_allow_jump", false);
    target_x_m_ = declareOrGet("target_x_m", 0.0);
    target_y_m_ = declareOrGet("target_y_m", 0.0);
    target_z_m_ = declareOrGet("target_z_m", -3.0);
    odom_frame_config_.quaternion_order =
        declareOrGet("odom_quaternion_order", std::string("auto"));
    odom_frame_config_.odom_body_frame =
        declareOrGet("odom_body_frame", std::string("auto"));
    odom_frame_config_.require_ned_pose_frame =
        declareOrGet("require_ned_pose_frame", true);
    odom_frame_config_.require_ned_velocity_frame =
        declareOrGet("require_ned_velocity_frame", true);
    odom_frame_config_.expected_pose_frame_ned =
        declareOrGet("expected_pose_frame_ned", 1);
    odom_frame_config_.expected_velocity_frame_ned =
        declareOrGet("expected_velocity_frame_ned", 1);

    pos_weight_ = declareOrGet("pos_weight", 2.0);
    vel_weight_ = declareOrGet("vel_weight", 5.0);
    yaw_weight_ = declareOrGet("yaw_weight", 0.8);
    tilt_weight_ = declareOrGet("tilt_weight", 5.0);
    thrust_weight_ = declareOrGet("thrust_weight", 0.08);
    rate_weight_ = declareOrGet("rate_weight", 0.08);
    terminal_pos_weight_ = declareOrGet("terminal_pos_weight", 6.0);
    terminal_vel_weight_ = declareOrGet("terminal_vel_weight", 4.0);
    terminal_yaw_weight_ = declareOrGet("terminal_yaw_weight", 2.5);
    terminal_tilt_weight_ = declareOrGet("terminal_tilt_weight", 6.0);

    max_iterations_ = declareOrGet("max_iterations", 8);
    constraint_tolerance_ = declareOrGet("constraint_tolerance", 0.5);
    solve_timeout_ms_ = declareOrGet("solve_timeout_ms", 450.0);
    max_pending_solve_requests_ = static_cast<std::size_t>(
        std::max(1, declareOrGet("max_pending_solve_requests", 2)));
    takeoff_min_thrust_margin_n_ =
        declareOrGet("takeoff_min_thrust_margin_n", 1.0);
    min_flight_thrust_ratio_ = declareOrGet("min_flight_thrust_ratio", 0.55);
    hover_min_thrust_ratio_ = declareOrGet("hover_min_thrust_ratio", 1.0);
    landing_min_thrust_ratio_ = declareOrGet("landing_min_thrust_ratio", 0.3);
    thrust_slew_rate_n_per_s_ =
        declareOrGet("thrust_slew_rate_n_per_s", 6.0);
    body_torque_slew_nm_per_s_ =
        declareOrGet("body_torque_slew_nm_per_s", 1.8);
    command_validity_thrust_jump_n_ =
        declareOrGet("command_validity_thrust_jump_n", 8.0);
    command_validity_torque_jump_nm_ =
        declareOrGet("command_validity_torque_jump_nm", 0.6);
    debug_logging_enabled_ = declareOrGet("debug_logging_enabled", true);
    enable_rotational_indi_ = declareOrGet("enable_rotational_indi", false);
    indi_blend_alpha_ = declareOrGet("indi_blend_alpha", 0.25);
    indi_rate_lpf_cutoff_hz_ = declareOrGet("indi_rate_lpf_cutoff_hz", 35.0);
    indi_accel_lpf_cutoff_hz_ = declareOrGet("indi_accel_lpf_cutoff_hz", 20.0);
    indi_torque_correction_limit_nm_ =
        declareOrGet("indi_torque_correction_limit_nm", 0.12);
    indi_max_measurement_age_s_ =
        declareOrGet("indi_max_measurement_age_s", 0.1);
    indi_max_body_rate_rad_s_ =
        declareOrGet("indi_max_body_rate_rad_s", 12.0);
    indi_use_rpm_feedback_ = declareOrGet("indi_use_rpm_feedback", false);
    indi_debug_logging_enabled_ =
        declareOrGet("indi_debug_logging_enabled", false);

    if (hover_thrust_n_ <= 0.0) {
      hover_thrust_n_ = mass_kg_ * gravity_mps2_;
    }
    if (std::abs(yaw_moment_coefficient_ -
                 cddp_mpc::kQuadrotorModelYawMomentCoefficient) > 1e-6) {
      RCLCPP_WARN(
          get_logger(),
          "yaw_moment_coefficient=%.3f does not match cddp::Quadrotor's fixed "
          "yaw moment coefficient %.3f. Using the model coefficient.",
          yaw_moment_coefficient_, cddp_mpc::kQuadrotorModelYawMomentCoefficient);
      yaw_moment_coefficient_ = cddp_mpc::kQuadrotorModelYawMomentCoefficient;
    }
    if (max_motor_thrust_n_ <= 0.0) {
      max_motor_thrust_n_ = std::max(max_thrust_n_ / 4.0, hover_thrust_n_ / 2.0);
    }
    if (max_roll_torque_nm_ <= 0.0) {
      max_roll_torque_nm_ = arm_length_m_ * max_motor_thrust_n_;
    }
    if (max_pitch_torque_nm_ <= 0.0) {
      max_pitch_torque_nm_ = arm_length_m_ * max_motor_thrust_n_;
    }
    if (max_yaw_torque_nm_ <= 0.0) {
      max_yaw_torque_nm_ = 2.0 * yaw_moment_coefficient_ * max_motor_thrust_n_;
    }
  }

  template <typename T>
  T declareOrGet(const std::string &name, const T &default_value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).template get_value<T>();
  }

  void localPositionCallback(const VehicleLocalPosition::SharedPtr msg) {
    if (use_odometry_state_ && odometry_received_ && odom_frame_ok_) {
      return;
    }
    current_position_ned_ =
        Eigen::Vector3d(static_cast<double>(msg->x), static_cast<double>(msg->y),
                        static_cast<double>(msg->z));
    current_velocity_ned_ =
        Eigen::Vector3d(static_cast<double>(msg->vx), static_cast<double>(msg->vy),
                        static_cast<double>(msg->vz));
    if (!home_z_ned_.has_value()) {
      home_z_ned_ = current_position_ned_->z();
    }

    if (!setpoint_enu_.has_value()) {
      const Eigen::Vector3d position_enu = cddp_mpc::nedToEnu(*current_position_ned_);
      setpoint_enu_ = Eigen::Vector3d(position_enu.x(), position_enu.y(),
                                      -takeoff_altitude_m_);
    }
  }

  void odometryCallback(const VehicleOdometry::SharedPtr msg) {
    if (!use_odometry_state_) {
      return;
    }

    const Eigen::Vector3d position(static_cast<double>(msg->position[0]),
                                   static_cast<double>(msg->position[1]),
                                   static_cast<double>(msg->position[2]));
    const Eigen::Vector3d velocity(static_cast<double>(msg->velocity[0]),
                                   static_cast<double>(msg->velocity[1]),
                                   static_cast<double>(msg->velocity[2]));
    const Eigen::Vector3d angular_velocity(
        static_cast<double>(msg->angular_velocity[0]),
        static_cast<double>(msg->angular_velocity[1]),
        static_cast<double>(msg->angular_velocity[2]));
    const Eigen::Vector4d raw_q(static_cast<double>(msg->q[0]),
                                static_cast<double>(msg->q[1]),
                                static_cast<double>(msg->q[2]),
                                static_cast<double>(msg->q[3]));

    const cddp_mpc::NormalizedOdometry odom = cddp_mpc::normalizeOdometry(
        position, velocity, angular_velocity, raw_q,
        std::optional<int>(static_cast<int>(msg->pose_frame)),
        std::optional<int>(static_cast<int>(msg->velocity_frame)), odom_frame_config_);

    odometry_received_ = true;
    odom_frame_ok_ = odom.frame_ok;
    current_body_rates_ = odom.angular_velocity;
    updateIndiMeasurement(msg->timestamp * 1e-6, current_body_rates_.value(),
                          "vehicle_odometry");

    if (!odom.frame_ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring VehicleOdometry with unexpected frame metadata.");
      return;
    }

    current_position_ned_ = odom.position;
    current_velocity_ned_ = odom.velocity;
    current_attitude_ned_ = odom.attitude_wxyz;

    if (!home_z_ned_.has_value()) {
      home_z_ned_ = current_position_ned_->z();
    }

    if (!setpoint_enu_.has_value()) {
      const Eigen::Vector3d position_enu = cddp_mpc::nedToEnu(*current_position_ned_);
      setpoint_enu_ = Eigen::Vector3d(position_enu.x(), position_enu.y(),
                                      -takeoff_altitude_m_);
    }
  }

  void attitudeCallback(const VehicleAttitude::SharedPtr msg) {
    if (use_odometry_state_ && odometry_received_ && odom_frame_ok_) {
      return;
    }
    const Eigen::Vector4d q_ned_wxyz(
        static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
        static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]));
    current_attitude_ned_ = cddp_mpc::normalizeQuaternionWxyz(q_ned_wxyz);
  }

#if CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY
  void angularVelocityCallback(const VehicleAngularVelocity::SharedPtr msg) {
    const Eigen::Vector3d body_rates(static_cast<double>(msg->xyz[0]),
                                     static_cast<double>(msg->xyz[1]),
                                     static_cast<double>(msg->xyz[2]));
    current_body_rates_ = body_rates;
    updateIndiMeasurement(msg->timestamp * 1e-6, body_rates,
                          "vehicle_angular_velocity");
  }
#endif

  void vehicleStatusCallback(const VehicleStatus::SharedPtr msg) {
    armed_ = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
    offboard_enabled_ =
        msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  void handleStartMission(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    mission_start_requested_ = true;
    mode_request_time_ = nowSeconds();
    response->success = true;
    response->message = "mission start requested";
    RCLCPP_INFO(get_logger(), "Mission start requested via /cddp_mpc/start_mission.");
  }

  void controlLoop() {
    if (!stateAvailable()) {
      if (armed_ || offboard_enabled_) {
        publishFallbackForUnavailableState();
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Waiting for PX4 state topics...");
      return;
    }

    updateMode();
    pollSolveResult();

    SafetyTrace trace{};
    Eigen::Vector4d command = selectCommandForPublish();
    trace.fallback_active = last_command_source_ != "solver";
    command = applySafetyLayer("shaping", command,
                               [this](const Eigen::Vector4d &value) {
                                 return shapeCommand(value);
                               }, &trace);
    command = applySafetyLayer("hover_vertical_correction", command,
                               [this](const Eigen::Vector4d &value) {
                                 return applyHoverVerticalCorrection(value);
                               }, &trace);
    command = applySafetyLayer("hover_lateral_correction", command,
                               [this](const Eigen::Vector4d &value) {
                                 return applyHoverLateralCorrection(value);
                               }, &trace);
    last_nominal_command_ = command;
    command = applySafetyLayer("rotational_indi", command,
                               [this](const Eigen::Vector4d &value) {
                                 return applyRotationalIndi(value);
                               }, &trace);
    last_corrected_command_ = command;
    command = enforceFinalSafetyGate(command, &trace);
    last_safety_trace_ = trace;
    last_published_command_ = command;
    publishVehicleThrustTorqueSetpoint(command);
    publishStatusDiagnostics();
    debugLog(command);
  }

  void solveLoop() {
    if (!stateAvailable()) {
      return;
    }
    if (mode_ == "READY" || mode_ == "LAND_DONE") {
      return;
    }

    pollSolveResult();
    bool has_fresh_request = false;
    for (auto &pending : pending_solves_) {
      const double age_ms = (nowSeconds() - pending.start_time_s) * 1000.0;
      if (age_ms <= solve_timeout_ms_) {
        has_fresh_request = true;
      } else if (!pending.timeout_reported) {
        pending.timeout_reported = true;
        RCLCPP_WARN(get_logger(),
                    "CDDP solve request %" PRIu64
                    " exceeded timeout budget (%.1f ms > %.1f ms). "
                    "Launching a fresher request if capacity allows.",
                    pending.request_id, age_ms, solve_timeout_ms_);
      }
    }
    if (has_fresh_request || pending_solves_.size() >= max_pending_solve_requests_) {
      return;
    }

    if (!(armed_ && offboard_enabled_)) {
      return;
    }

    const Eigen::Vector3d reference_target_enu = currentReferenceTargetEnu();
    const Eigen::VectorXd initial_state =
        buildCurrentStateErrorEnu(reference_target_enu);
    const ReferenceTrajectory reference_trajectory =
        buildReferenceTrajectory(initial_state);
    if (!reference_trajectory.controls.empty() &&
        reference_trajectory.controls.front().size() >= 1) {
      latest_reference_thrust_n_ = reference_trajectory.controls.front()(0);
    }
    const bool landing_mode = mode_ == "LAND";
    std::vector<Eigen::VectorXd> initial_state_guess;
    std::vector<Eigen::VectorXd> initial_control_guess;
    {
      std::lock_guard<std::mutex> lock(warm_start_mutex_);
      initial_state_guess =
          buildInitialStateGuessUnlocked(initial_state, reference_trajectory.states);
      initial_control_guess =
          buildInitialControlGuessUnlocked(initial_state, reference_trajectory.controls);
    }

    const std::uint64_t request_id = ++next_solve_request_id_;
    pending_solves_.push_back(PendingSolve{
        request_id,
        nowSeconds(),
        false,
        std::async(std::launch::async,
                   [this, request_id, initial_state, reference_trajectory,
                    initial_state_guess, initial_control_guess, landing_mode,
                    reference_target_enu]() {
                     return solveMPC(request_id, initial_state, reference_trajectory,
                                     initial_state_guess, initial_control_guess,
                                     reference_target_enu, landing_mode);
                   }),
    });
  }

  void updateMode() {
    const std::string previous_mode = mode_;
    const double now_s = nowSeconds();

    if (!mission_start_requested_) {
      mode_ = "READY";
      if (previous_mode != mode_) {
        RCLCPP_INFO(get_logger(), "Mode transition: %s -> %s", previous_mode.c_str(),
                    mode_.c_str());
      }
      return;
    }

    if (auto_engage_offboard_ && !offboard_enabled_ &&
        (now_s - mode_request_time_) > 1.0) {
      engageOffboardMode();
      mode_request_time_ = now_s;
    }

    if (auto_arm_ && !armed_ && (now_s - mode_request_time_) > 1.0) {
      arm();
      mode_request_time_ = now_s;
    }

    if (!(armed_ && offboard_enabled_)) {
      mode_ = "INIT";
      hover_start_time_s_.reset();
      hover_in_band_since_s_.reset();
      hover_out_of_band_since_s_.reset();
      landing_start_time_s_.reset();
      return;
    }

    if (mode_ == "READY") {
      mode_ = "INIT";
    }

    if (mode_ == "INIT") {
      setTakeoffSetpoint();
      mode_ = "TAKEOFF";
      hover_start_time_s_.reset();
      hover_in_band_since_s_.reset();
      hover_out_of_band_since_s_.reset();
      landing_start_time_s_.reset();
    }

    if (mode_ == "LAND") {
      updateLandingSetpoint(now_s);
    }

    const double target_z_ned = activeTargetZNed();
    const double current_z_ned = current_position_ned_->z();
    const double current_vz_ned = current_velocity_ned_->z();
    const double altitude_error = std::abs(current_z_ned - target_z_ned);
    const double current_body_rate_norm = currentBodyRateMagnitude();

    if (mode_ == "TAKEOFF") {
      if (altitude_error <= settle_tolerance_m_ &&
          std::abs(current_vz_ned) <= hover_entry_vz_threshold_mps_ &&
          current_body_rate_norm <= hover_entry_rate_threshold_rad_s_) {
        if (!hover_in_band_since_s_.has_value()) {
          hover_in_band_since_s_ = now_s;
        } else if ((now_s - *hover_in_band_since_s_) >= hover_entry_dwell_s_) {
          mode_ = "HOVER";
          hover_start_time_s_ = now_s;
          hover_out_of_band_since_s_.reset();
        }
      } else {
        hover_in_band_since_s_.reset();
      }
    }

    if (mode_ == "HOVER") {
      if (altitude_error > hover_reenter_error_m_) {
        if (!hover_out_of_band_since_s_.has_value()) {
          hover_out_of_band_since_s_ = now_s;
        } else if ((now_s - *hover_out_of_band_since_s_) >=
                   hover_reenter_dwell_s_) {
          setTakeoffSetpoint();
          mode_ = "TAKEOFF";
          hover_start_time_s_.reset();
          hover_in_band_since_s_.reset();
          hover_out_of_band_since_s_.reset();
          RCLCPP_WARN(get_logger(),
                      "Altitude drift exceeded hover threshold; re-entering TAKEOFF.");
          return;
        }
      } else {
        hover_out_of_band_since_s_.reset();
      }

      if (hover_start_time_s_.has_value() &&
          (now_s - *hover_start_time_s_) >= hover_duration_s_) {
        if (landing_enabled_ && home_z_ned_.has_value()) {
          mode_ = "LAND";
          landing_start_time_s_ = now_s;
          landing_start_setpoint_z_ned_ = activeTargetZNed();
        }
      }
    }

    if (mode_ == "LAND" && home_z_ned_.has_value()) {
      const double landing_error = std::abs(current_z_ned - *home_z_ned_);
      if (landing_error <= landing_touchdown_tolerance_m_ &&
          std::abs(current_vz_ned) <= landing_touchdown_vz_threshold_mps_) {
        disarm();
        mode_request_time_ = now_s;
        mode_ = "LAND_DONE";
      }
    }

    if (previous_mode != mode_) {
      RCLCPP_INFO(get_logger(),
                  "Mode transition: %s -> %s (z_ned=%.2f, target_z_ned=%.2f)",
                  previous_mode.c_str(), mode_.c_str(), current_z_ned, target_z_ned);
    }
  }

  SolveResult solveMPC(const std::uint64_t request_id,
                       const Eigen::VectorXd &initial_state,
                       const ReferenceTrajectory &reference_trajectory,
                       const std::vector<Eigen::VectorXd> &initial_state_guess,
                       const std::vector<Eigen::VectorXd> &initial_control_guess,
                       const Eigen::Vector3d &reference_target_enu,
                       bool landing_mode) {
    SolveResult result;
    result.request_id = request_id;
    result.command = hoverCommand(landing_mode ? landing_descent_rate_mps_ : 0.0);

    try {
      auto solver = createSolver(reference_trajectory, reference_target_enu);
      solver->setInitialState(initial_state);
      solver->setReferenceState(reference_trajectory.states.back());
      solver->setReferenceStates(reference_trajectory.states);
      solver->setInitialTrajectory(initial_state_guess, initial_control_guess);

      const cddp::CDDPSolution solution = solver->solve(solver_type_);
      result.status_message = solution.status_message;
      result.converged = solution.status_message == "OptimalSolutionFound" ||
                         solution.status_message == "AcceptableSolutionFound";
      result.solve_time_ms = solution.solve_time_ms;
      result.final_objective = solution.final_objective;
      result.max_constraint_violation = solution.final_primal_infeasibility;
      result.final_primal_infeasibility = solution.final_primal_infeasibility;
      result.final_dual_infeasibility = solution.final_dual_infeasibility;
      result.final_complementary_infeasibility =
          solution.final_complementary_infeasibility;
      if (!solution.control_trajectory.empty() &&
          solution.control_trajectory.front().size() >= 4) {
        const Eigen::Vector4d candidate =
            solution.control_trajectory.front().head<4>();
        const Eigen::Vector4d clipped = clipCommand(candidate);
        const bool finite_command = candidate.allFinite();
        const double raw_bound_violation = (candidate - clipped).cwiseAbs().maxCoeff();
        result.raw_bound_violation = raw_bound_violation;
        const bool bounds_ok = raw_bound_violation <= constraint_tolerance_;
        const bool finite_objective = std::isfinite(solution.final_objective);
        const bool finite_primal =
            std::isfinite(solution.final_primal_infeasibility);
        const bool feasible =
            finite_primal
                ? (solution.final_primal_infeasibility <= constraint_tolerance_)
                : false;

        result.command = clipped;
        result.state_trajectory = solution.state_trajectory;
        result.control_trajectory = solution.control_trajectory;
        result.usable_solution =
            finite_command && bounds_ok && finite_objective && feasible;
        result.success = result.usable_solution;

        if (!result.usable_solution) {
          if (!initial_control_guess.empty() &&
              initial_control_guess.front().size() >= 4) {
            result.command = clipCommand(initial_control_guess.front().head<4>());
          } else {
            result.command =
                hoverCommand(landing_mode ? landing_descent_rate_mps_ : 0.0);
          }
        }

        if (!result.success) {
          std::ostringstream status_detail;
          status_detail << solution.status_message
                        << " finite_cmd=" << (finite_command ? "true" : "false")
                        << " bounds_ok=" << (bounds_ok ? "true" : "false")
                        << " feasible=" << (feasible ? "true" : "false")
                        << " raw_bound_violation="
                        << formatDouble(raw_bound_violation)
                        << " inf_pr="
                        << formatDouble(solution.final_primal_infeasibility)
                        << " objective=" << formatDouble(solution.final_objective);
          result.status_message = status_detail.str();
        }
      }
    } catch (const std::exception &ex) {
      result.status_message = ex.what();
    }

    return result;
  }

  std::unique_ptr<cddp::CDDP> createSolver(
      const ReferenceTrajectory &reference_trajectory,
      const Eigen::Vector3d &setpoint_enu) const {
    auto system = std::make_unique<cddp_mpc::ErrorStateEnuQuadrotorThrust>(
        mpc_dt_, mass_kg_, inertiaMatrix(), arm_length_m_, setpoint_enu, "rk4");

    const int state_dim = 13;
    const int control_dim = 4;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Q.block<3, 3>(0, 0).diagonal().setConstant(pos_weight_);
    Q(3, 3) = 0.1 * yaw_weight_;
    Q(4, 4) = tilt_weight_;
    Q(5, 5) = tilt_weight_;
    Q(6, 6) = yaw_weight_;
    Q.block<3, 3>(7, 7).diagonal().setConstant(vel_weight_);
    Q.block<3, 3>(10, 10).diagonal().setConstant(rate_weight_);

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(control_dim, control_dim);
    R(0, 0) = thrust_weight_;
    R(1, 1) = rate_weight_;
    R(2, 2) = rate_weight_;
    R(3, 3) = rate_weight_;

    Eigen::MatrixXd Qf = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Qf.block<3, 3>(0, 0).diagonal().setConstant(terminal_pos_weight_);
    Qf(3, 3) = 0.1 * terminal_yaw_weight_;
    Qf(4, 4) = terminal_tilt_weight_;
    Qf(5, 5) = terminal_tilt_weight_;
    Qf(6, 6) = terminal_yaw_weight_;
    Qf.block<3, 3>(7, 7).diagonal().setConstant(terminal_vel_weight_);
    Qf.block<3, 3>(10, 10).diagonal().setConstant(rate_weight_);

    auto objective = std::make_unique<TrackingQuadraticObjective>(
        Q, R, Qf, reference_trajectory.states.back(), reference_trajectory.states,
        reference_trajectory.controls, mpc_dt_);

    cddp::CDDPOptions options;
    options.max_iterations = std::max(1, max_iterations_);
    options.tolerance = 1e-4;
    options.acceptable_tolerance = 1e-3;
    options.max_cpu_time = solve_timeout_ms_ / 1000.0;
    options.verbose = false;
    options.debug = false;
    options.print_solver_header = false;
    options.warm_start = true;
    options.regularization.initial_value = 1e-3;
    options.msipddp.segment_length = std::max(1, horizon_steps_ / 4);
    options.msipddp.rollout_type = "nonlinear";

    auto solver = std::make_unique<cddp::CDDP>(
        reference_trajectory.states.back(), reference_trajectory.states.back(),
        horizon_steps_, mpc_dt_, std::move(system), std::move(objective), options);

    const double min_collective_thrust_n =
        std::max(min_thrust_n_, 4.0 * min_motor_thrust_n_);
    const double max_collective_thrust_n =
        std::max(min_collective_thrust_n,
                 std::min(max_thrust_n_, 4.0 * max_motor_thrust_n_));
    Eigen::VectorXd lower_bound(control_dim);
    lower_bound << min_collective_thrust_n, -max_roll_torque_nm_, -max_pitch_torque_nm_,
        -max_yaw_torque_nm_;
    Eigen::VectorXd upper_bound(control_dim);
    upper_bound << max_collective_thrust_n, max_roll_torque_nm_, max_pitch_torque_nm_,
        max_yaw_torque_nm_;
    solver->addPathConstraint(
        "control_bounds",
        std::make_unique<cddp::ControlConstraint>(lower_bound, upper_bound));
    solver->addPathConstraint(
        "motor_allocation_bounds",
        std::make_unique<cddp_mpc::ThrustAllocationConstraint>(
            arm_length_m_, min_motor_thrust_n_, max_motor_thrust_n_));
    return solver;
  }

  Eigen::VectorXd
  buildCurrentStateErrorEnu(const Eigen::Vector3d &reference_target_enu) const {
    const Eigen::Vector3d position_enu = cddp_mpc::nedToEnu(*current_position_ned_);
    const Eigen::Vector3d velocity_enu = cddp_mpc::nedToEnu(*current_velocity_ned_);
    const Eigen::Quaterniond attitude_enu =
        cddp_mpc::quatNedToEnuWxyz(*current_attitude_ned_);
    const Eigen::Vector3d position_error_enu = position_enu - reference_target_enu;

    Eigen::VectorXd state(13);
    state.segment<3>(0) = position_error_enu;
    state(3) = attitude_enu.w();
    state(4) = attitude_enu.x();
    state(5) = attitude_enu.y();
    state(6) = attitude_enu.z();
    state.segment<3>(7) = velocity_enu;
    state.segment<3>(10) =
        current_body_rates_.value_or(Eigen::Vector3d::Zero());
    return state;
  }

  ReferenceTrajectory buildReferenceTrajectory(
      const Eigen::VectorXd &initial_state) const {
    return reference_provider_->build(initial_state);
  }

  std::vector<Eigen::VectorXd>
  buildInitialStateGuessUnlocked(const Eigen::VectorXd &initial_state,
                                 const std::vector<Eigen::VectorXd> &reference_states) const {
    if (previous_state_guess_.has_value() &&
        !shouldResetWarmStartUnlocked(initial_state) &&
        previous_state_guess_->size() == static_cast<std::size_t>(horizon_steps_ + 1)) {
      std::vector<Eigen::VectorXd> guess = *previous_state_guess_;
      guess.front() = initial_state;
      return guess;
    }

    std::vector<Eigen::VectorXd> guess(horizon_steps_ + 1, initial_state);
    for (int i = 1; i <= horizon_steps_; ++i) {
      guess[static_cast<std::size_t>(i)] = reference_states[static_cast<std::size_t>(i)];
    }
    return guess;
  }

  std::vector<Eigen::VectorXd>
  buildInitialControlGuessUnlocked(
      const Eigen::VectorXd &initial_state,
      const std::vector<Eigen::VectorXd> &reference_controls) const {
    if (previous_control_guess_.has_value() &&
        !shouldResetWarmStartUnlocked(initial_state) &&
        previous_control_guess_->size() == static_cast<std::size_t>(horizon_steps_)) {
      return *previous_control_guess_;
    }

    std::vector<Eigen::VectorXd> guess;
    guess.reserve(static_cast<std::size_t>(horizon_steps_));
    for (int i = 0; i < horizon_steps_; ++i) {
      if (i < static_cast<int>(reference_controls.size())) {
        guess.push_back(reference_controls[static_cast<std::size_t>(i)]);
      } else {
        Eigen::VectorXd u(4);
        u << hover_thrust_n_, 0.0, 0.0, 0.0;
        guess.push_back(u);
      }
    }
    return guess;
  }

  bool shouldResetWarmStartUnlocked(const Eigen::VectorXd &initial_state) const {
    if (!previous_state_guess_.has_value() ||
        previous_state_guess_->size() < static_cast<std::size_t>(horizon_steps_ + 1)) {
      return false;
    }

    const Eigen::VectorXd &expected_state = previous_state_guess_->front();
    return cddp_mpc::shouldResetWarmStartThrustModel(
        initial_state, expected_state, mpc_dt_, reference_max_climb_rate_mps_,
        reference_max_climb_accel_mps2_);
  }

  std::vector<Eigen::VectorXd>
  shiftStateTrajectory(const std::vector<Eigen::VectorXd> &trajectory) const {
    if (trajectory.size() != static_cast<std::size_t>(horizon_steps_ + 1)) {
      return {};
    }
    std::vector<Eigen::VectorXd> shifted = trajectory;
    for (int i = 0; i < horizon_steps_; ++i) {
      shifted[static_cast<std::size_t>(i)] =
          trajectory[static_cast<std::size_t>(std::min(i + 1, horizon_steps_))];
    }
    shifted.back() = trajectory.back();
    return shifted;
  }

  std::vector<Eigen::VectorXd>
  shiftControlTrajectory(const std::vector<Eigen::VectorXd> &trajectory) const {
    if (trajectory.size() != static_cast<std::size_t>(horizon_steps_)) {
      return {};
    }
    std::vector<Eigen::VectorXd> shifted = trajectory;
    for (int i = 0; i < horizon_steps_ - 1; ++i) {
      shifted[static_cast<std::size_t>(i)] =
          trajectory[static_cast<std::size_t>(i + 1)];
    }
    shifted.back() = trajectory.back();
    return shifted;
  }

  Eigen::Vector4d interpolateSolverCommand(double elapsed_s) const {
    if (last_planned_control_trajectory_.empty()) {
      return last_solver_command_;
    }

    const double dt = std::max(mpc_dt_, 1e-6);
    const int index = static_cast<int>(elapsed_s / dt);
    if (index >= static_cast<int>(last_planned_control_trajectory_.size()) - 1) {
      return clipCommand(last_planned_control_trajectory_.back().head<4>());
    }

    const double frac = std::clamp((elapsed_s / dt) - index, 0.0, 1.0);
    const Eigen::Vector4d u0 =
        last_planned_control_trajectory_[static_cast<std::size_t>(index)].head<4>();
    const Eigen::Vector4d u1 =
        last_planned_control_trajectory_[static_cast<std::size_t>(index + 1)].head<4>();
    return clipCommand(u0 + frac * (u1 - u0));
  }

  Eigen::Vector4d readyCommand() const {
    Eigen::Vector4d command = Eigen::Vector4d::Zero();
    command(0) = std::clamp(ready_thrust_n_, min_thrust_n_, max_thrust_n_);
    return command;
  }

  Eigen::Vector4d selectCommandForPublish() {
    const cddp_mpc::CommandSource source = cddp_mpc::selectCommandSource(
        mode_, last_usable_solution_, solve_fail_streak_, fallback_hold_cycles_);
    last_command_source_ = cddp_mpc::commandSourceLabel(source);
    last_plan_age_s_ = std::numeric_limits<double>::quiet_NaN();
    stale_plan_rejected_this_cycle_ = false;
    overrun_hover_active_ = false;

    if (source == cddp_mpc::CommandSource::ReadyWait) {
      return readyCommand();
    }
    if (source == cddp_mpc::CommandSource::LandDoneIdle) {
      return Eigen::Vector4d::Zero();
    }
    if (source == cddp_mpc::CommandSource::Solver) {
      const double elapsed = std::max(0.0, nowSeconds() - last_solve_wall_time_s_);
      last_plan_age_s_ = elapsed;
      if (consecutive_overrun_cycles_ >=
          std::max(1, solve_overrun_hover_cycles_threshold_)) {
        overrun_hover_active_ = true;
        last_command_source_ = "solver_overrun_hover";
        return hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
      }
      if (elapsed > stale_plan_timeout_s_) {
        stale_plan_rejected_this_cycle_ = true;
        ++stale_plan_reject_count_;
        last_command_source_ = "stale_plan_hover";
        return hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
      }
      return interpolateSolverCommand(elapsed);
    }
    if (source == cddp_mpc::CommandSource::RecoveryHover ||
        source == cddp_mpc::CommandSource::RecoveryDescent) {
      return hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
    }
    if (source == cddp_mpc::CommandSource::LastGoodHold) {
      return last_good_command_;
    }
    return hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
  }

  Eigen::Vector4d hoverCommand(double descent_rate_mps) const {
    const double descent_scale =
        std::clamp(descent_rate_mps / std::max(landing_descent_rate_mps_, 1e-3), 0.0, 1.0);
    const double thrust_ratio = 1.0 - 0.2 * descent_scale;
    Eigen::Vector4d command;
    command << hover_thrust_n_ * thrust_ratio, 0.0, 0.0, 0.0;
    return command;
  }

  double takeoffMinimumThrust() const {
    if (!current_position_ned_.has_value()) {
      return hover_thrust_n_ + takeoff_min_thrust_margin_n_;
    }

    const double reference_floor =
        latest_reference_thrust_n_.has_value() ? *latest_reference_thrust_n_ : min_thrust_n_;
    return cddp_mpc::computeTakeoffMinimumThrust(
        (*current_position_ned_)(2), activeTargetZNed(), hover_thrust_n_,
        takeoff_min_thrust_margin_n_, settle_tolerance_m_,
        min_flight_thrust_ratio_, min_thrust_n_, max_thrust_n_, reference_floor);
  }

  double hoverElapsedSeconds() {
    if (!hover_start_time_s_.has_value()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::max(0.0, nowSeconds() - *hover_start_time_s_);
  }

  Eigen::Vector4d shapeCommand(const Eigen::Vector4d &command) const {
    double pos_err = 1.0;
    if (current_position_ned_.has_value()) {
      pos_err =
          (currentReferenceTargetEnu() - cddp_mpc::nedToEnu(*current_position_ned_)).norm();
    }
    return cddp_mpc::shapeSolverThrustCommand(
        command, last_published_command_,
        cddp_mpc::selectCommandSource(mode_, last_usable_solution_,
                                      solve_fail_streak_, fallback_hold_cycles_),
        mode_, solver_command_blend_alpha_, hoverTorqueLimitVector(),
        maxBodyTorqueVector(), hover_thrust_n_, min_flight_thrust_ratio_,
        hover_min_thrust_ratio_, pos_err);
  }

  Eigen::Vector4d applyFlightThrustFloor(const Eigen::Vector4d &command) {
    return cddp_mpc::applyFlightThrustFloor(
        command, mode_, hover_thrust_n_, min_flight_thrust_ratio_,
        hover_min_thrust_ratio_, landing_min_thrust_ratio_,
        takeoffMinimumThrust(), hoverElapsedSeconds(), hover_handoff_duration_s_,
        min_thrust_n_, max_thrust_n_);
  }

  Eigen::Vector4d applyHoverVerticalCorrection(const Eigen::Vector4d &command) const {
    if (!current_position_ned_.has_value() || !current_velocity_ned_.has_value()) {
      return command;
    }

    const Eigen::Vector3d reference_target_enu = currentReferenceTargetEnu();
    const Eigen::Vector3d current_position_enu =
        cddp_mpc::nedToEnu(*current_position_ned_);
    const Eigen::Vector3d current_velocity_enu =
        cddp_mpc::nedToEnu(*current_velocity_ned_);
    const double position_error_enu_z = reference_target_enu(2) - current_position_enu.z();
    const double velocity_enu_z = current_velocity_enu.z();

    return cddp_mpc::applyHoverVerticalCorrection(
        command, mode_, position_error_enu_z, velocity_enu_z,
        hover_vertical_kp_n_per_m_, hover_vertical_kd_n_per_mps_,
        hover_vertical_correction_limit_n_, min_thrust_n_, max_thrust_n_);
  }

  Eigen::Vector4d applyHoverLateralCorrection(const Eigen::Vector4d &command) const {
    if (!current_position_ned_.has_value() || !current_velocity_ned_.has_value() ||
        !current_attitude_ned_.has_value()) {
      return command;
    }

    const Eigen::Vector3d reference_target_enu = currentReferenceTargetEnu();
    const Eigen::Vector3d current_position_enu =
        cddp_mpc::nedToEnu(*current_position_ned_);
    const Eigen::Vector3d current_velocity_enu =
        cddp_mpc::nedToEnu(*current_velocity_ned_);
    const Eigen::Vector3d position_error_enu = reference_target_enu - current_position_enu;
    const Eigen::Quaterniond attitude_enu =
        cddp_mpc::quatNedToEnuWxyz(*current_attitude_ned_);
    const Eigen::Matrix3d world_to_body = attitude_enu.toRotationMatrix().transpose();
    const Eigen::Vector3d body_error = world_to_body * position_error_enu;
    const Eigen::Vector3d body_velocity = world_to_body * current_velocity_enu;

    // body_error.y() is body-LEFT in FLU; negate for body-RIGHT expected
    // by applyHoverLateralTorqueCorrection so the roll correction sign is
    // correct (positive body_right_error → tilt right → move right).
    return cddp_mpc::applyHoverLateralTorqueCorrection(
        command, mode_, body_error.x(), -body_error.y(), body_velocity.x(),
        -body_velocity.y(), hover_lateral_kp_nm_per_m_,
        hover_lateral_kd_nm_per_mps_,
        Eigen::Vector3d(hover_lateral_correction_limit_nm_,
                        hover_lateral_correction_limit_nm_,
                        hover_lateral_correction_limit_nm_),
        maxBodyTorqueVector());
  }

  cddp_mpc::ReferenceConfig buildReferenceConfig() const {
    cddp_mpc::ReferenceConfig config;
    config.horizon_steps = horizon_steps_;
    config.mpc_dt = mpc_dt_;
    config.hover_thrust_n = hover_thrust_n_;
    config.min_thrust_n = min_thrust_n_;
    config.max_thrust_n = max_thrust_n_;
    config.max_axis_speed_mps = reference_max_climb_rate_mps_;
    config.max_axis_accel_mps2 = reference_max_climb_accel_mps2_;
    config.max_yaw_rate_rad_s = reference_max_yaw_rate_rad_s_;
    config.smoothing_tau_s = reference_smoothing_tau_s_;
    config.target_yaw_rad = target_yaw_rad_;
    config.trajectory_period_s = reference_trajectory_period_s_;
    config.circle_radius_m = reference_circle_radius_m_;
    config.figure_eight_amplitude_m = reference_figure_eight_amplitude_m_;
    config.vertical_amplitude_m = reference_vertical_amplitude_m_;
    config.allow_reference_jump = reference_allow_jump_;
    config.mode = reference_mode_;
    return config;
  }

  cddp_mpc::IndiRotationalCompensator::Config buildIndiConfig() const {
    cddp_mpc::IndiRotationalCompensator::Config config;
    config.enabled = enable_rotational_indi_;
    config.blend_alpha = indi_blend_alpha_;
    config.rate_lpf_cutoff_hz = indi_rate_lpf_cutoff_hz_;
    config.accel_lpf_cutoff_hz = indi_accel_lpf_cutoff_hz_;
    config.torque_correction_limit_nm = indi_torque_correction_limit_nm_;
    config.use_rpm_feedback = indi_use_rpm_feedback_;
    config.debug_logging_enabled = indi_debug_logging_enabled_;
    return config;
  }

  Eigen::Vector3d currentReferenceTargetEnu() const {
    if ((mode_ == "TAKEOFF" || mode_ == "HOVER" || mode_ == "LAND") &&
        setpoint_enu_.has_value()) {
      return *setpoint_enu_;
    }
    if (reference_mode_ == "configured_target") {
      return Eigen::Vector3d(target_x_m_, target_y_m_, target_z_m_);
    }
    if (setpoint_enu_.has_value()) {
      return *setpoint_enu_;
    }
    return Eigen::Vector3d(target_x_m_, target_y_m_, target_z_m_);
  }

  void updateIndiMeasurement(double timestamp_s, const Eigen::Vector3d &body_rates,
                             const char *source_label) {
    if (!rotational_indi_ || !body_rates.allFinite()) {
      return;
    }
    rotational_indi_->updateMeasurement(timestamp_s, body_rates);
    latest_rate_measurement_source_ = source_label;
  }

  Eigen::Vector4d applyRotationalIndi(const Eigen::Vector4d &command) {
    if (!rotational_indi_) {
      return command;
    }

    rotational_indi_->setConfig(buildIndiConfig());
    if (!enable_rotational_indi_) {
      last_indi_correction_.setZero();
      last_indi_disable_reason_ = "disabled_by_param";
      return command;
    }

    const auto &indi_status = rotational_indi_->status();
    if (!indi_status.has_measurement) {
      last_indi_correction_.setZero();
      last_indi_disable_reason_ = "no_measurement";
      return command;
    }
    if (!std::isfinite(indi_status.measurement_age_s) ||
        indi_status.measurement_age_s > indi_max_measurement_age_s_) {
      last_indi_correction_.setZero();
      last_indi_disable_reason_ = "stale_measurement";
      return command;
    }
    if (!indi_status.filtered_rates.allFinite() ||
        indi_status.filtered_rates.cwiseAbs().maxCoeff() > indi_max_body_rate_rad_s_) {
      last_indi_correction_.setZero();
      last_indi_disable_reason_ = "rate_guard";
      return command;
    }

    const Eigen::Vector3d body_rates =
        current_body_rates_.value_or(Eigen::Vector3d::Zero());
    const Eigen::Vector3d corrected_torque = rotational_indi_->computeTorqueCommand(
        nowSeconds(), command.tail<3>(), body_rates, last_published_command_.tail<3>());
    if (!corrected_torque.allFinite()) {
      last_indi_correction_.setZero();
      last_indi_disable_reason_ = "nonfinite_output";
      return command;
    }

    Eigen::Vector4d corrected_command = command;
    corrected_command.tail<3>() = corrected_torque;
    last_indi_correction_ = corrected_torque - command.tail<3>();
    last_indi_disable_reason_ = "active";
    return corrected_command;
  }

  Eigen::Vector4d safeFallbackCommand() const {
    if (mode_ == "READY" || !(armed_ || offboard_enabled_)) {
      return readyCommand();
    }
    return hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
  }

  bool isCommandFinite(const Eigen::Vector4d &command) const {
    return command.allFinite();
  }

  bool hasExcessiveCommandJump(const Eigen::Vector4d &command) const {
    const Eigen::Vector4d delta = (command - last_published_command_).cwiseAbs();
    return delta(0) > std::max(0.0, command_validity_thrust_jump_n_) ||
           delta.tail<3>().maxCoeff() >
               std::max(0.0, command_validity_torque_jump_nm_);
  }

  Eigen::Vector4d enforceFinalSafetyGate(const Eigen::Vector4d &command,
                                         SafetyTrace *trace) {
    Eigen::Vector4d gated = command;
    if (!isCommandFinite(gated)) {
      ++command_validity_failure_count_;
      last_command_validity_status_ = "nonfinite";
      gated = safeFallbackCommand();
      if (trace != nullptr) {
        trace->last_modified_layer = "command_validity_nonfinite";
      }
    } else if (hasExcessiveCommandJump(gated)) {
      ++command_validity_failure_count_;
      last_command_validity_status_ = "jump_rejected";
      gated = safeFallbackCommand();
      if (trace != nullptr) {
        trace->last_modified_layer = "command_validity_jump";
      }
    } else {
      last_command_validity_status_ = "ok";
    }

    gated = applySafetyLayer("clip_and_project", gated,
                             [this, trace](const Eigen::Vector4d &value) {
                               const Eigen::Vector4d clipped = clipCommand(value);
                               if (trace != nullptr &&
                                   (clipped - value).cwiseAbs().maxCoeff() > 1e-9) {
                                 const Eigen::Vector4d direct_clip =
                                     clipCommandWithoutProjection(value);
                                 trace->projection_changed_command =
                                     (clipped - direct_clip).cwiseAbs().maxCoeff() > 1e-9;
                               }
                               return clipped;
                             }, trace);
    gated = applySafetyLayer("slew_limit", gated,
                             [this](const Eigen::Vector4d &value) {
                               return applySlewLimits(value);
                             }, trace);
    gated = applySafetyLayer("thrust_floor", gated,
                             [this](const Eigen::Vector4d &value) {
                               return applyFlightThrustFloor(value);
                             }, trace);
    return gated;
  }

  void publishFallbackForUnavailableState() {
    SafetyTrace trace{};
    trace.fallback_active = true;
    Eigen::Vector4d command = safeFallbackCommand();
    command = enforceFinalSafetyGate(command, &trace);
    if (trace.last_modified_layer == "none") {
      trace.last_modified_layer = "state_unavailable_fallback";
    }
    last_nominal_command_ = command;
    last_corrected_command_ = command;
    last_safety_trace_ = trace;
    last_published_command_ = command;
    last_command_source_ = "state_unavailable_fallback";
    publishVehicleThrustTorqueSetpoint(command);
    publishStatusDiagnostics();
  }

  Eigen::Vector4d applySlewLimits(const Eigen::Vector4d &command) const {
    Eigen::Vector4d limited = command;
    const double dt = 1.0 / std::max(control_rate_hz_, 1e-3);
    const double max_thrust_step = thrust_slew_rate_n_per_s_ * dt;
    const double max_torque_step = body_torque_slew_nm_per_s_ * dt;

    limited(0) = std::clamp(limited(0), last_published_command_(0) - max_thrust_step,
                            last_published_command_(0) + max_thrust_step);
    for (int i = 1; i < 4; ++i) {
      limited(i) = std::clamp(limited(i), last_published_command_(i) - max_torque_step,
                              last_published_command_(i) + max_torque_step);
    }
    return limited;
  }

  template <typename LayerFn>
  Eigen::Vector4d applySafetyLayer(const std::string &layer_name,
                                   const Eigen::Vector4d &command,
                                   LayerFn &&layer_fn, SafetyTrace *trace) const {
    const Eigen::Vector4d updated = layer_fn(command);
    if (trace != nullptr &&
        (updated - command).cwiseAbs().maxCoeff() > 1e-9) {
      trace->last_modified_layer = layer_name;
    }
    return updated;
  }

  Eigen::Vector4d clipCommandWithoutProjection(const Eigen::Vector4d &command) const {
    Eigen::Vector4d clipped = command;
    const double min_collective_thrust_n =
        std::max(min_thrust_n_, 4.0 * min_motor_thrust_n_);
    const double max_collective_thrust_n =
        std::max(min_collective_thrust_n,
                 std::min(max_thrust_n_, 4.0 * max_motor_thrust_n_));
    clipped(0) =
        std::clamp(clipped(0), min_collective_thrust_n, max_collective_thrust_n);
    const Eigen::Vector3d max_torque = maxBodyTorqueVector();
    clipped(1) = std::clamp(clipped(1), -max_torque.x(), max_torque.x());
    clipped(2) = std::clamp(clipped(2), -max_torque.y(), max_torque.y());
    clipped(3) = std::clamp(clipped(3), -max_torque.z(), max_torque.z());
    return clipped;
  }

  Eigen::Vector4d clipCommand(const Eigen::Vector4d &command) const {
    const Eigen::Vector4d clipped = clipCommandWithoutProjection(command);
    return cddp_mpc::projectThrustTorqueToMotorLimits(
        clipped, min_motor_thrust_n_, max_motor_thrust_n_, arm_length_m_,
        yaw_moment_coefficient_);
  }

  void pollSolveResult() {
    bool has_overrun_pending = false;
    auto it = pending_solves_.begin();
    while (it != pending_solves_.end()) {
      if (it->future.wait_for(0ms) != std::future_status::ready) {
        const double age_ms = (nowSeconds() - it->start_time_s) * 1000.0;
        if (age_ms > solve_timeout_ms_ && !it->timeout_reported) {
          it->timeout_reported = true;
          ++solve_timeout_count_;
          has_overrun_pending = true;
        } else if (age_ms > solve_timeout_ms_) {
          ++solve_overrun_count_;
          has_overrun_pending = true;
        }
        ++it;
        continue;
      }

      SolveResult result = it->future.get();
      ++solve_count_;
      if (result.request_id > latest_applied_request_id_) {
        latest_applied_request_id_ = result.request_id;
        last_command_ = result.command;
        last_solver_command_ = result.command;
        last_solve_success_ = result.success;
        last_usable_solution_ = result.usable_solution;
        last_status_message_ = result.status_message;
        last_solve_time_ms_ = result.solve_time_ms;
        last_max_constraint_violation_ = result.max_constraint_violation;
        last_raw_bound_violation_ = result.raw_bound_violation;
        if (result.success) {
          last_good_command_ = result.command;
          last_solve_wall_time_s_ = it->start_time_s;
          solve_fail_streak_ = 0;
          consecutive_overrun_cycles_ = 0;
          last_planned_control_trajectory_ = result.control_trajectory;
          std::lock_guard<std::mutex> lock(warm_start_mutex_);
          previous_state_guess_ = shiftStateTrajectory(result.state_trajectory);
          previous_control_guess_ = shiftControlTrajectory(result.control_trajectory);
        } else {
          ++solve_fail_streak_;
          last_planned_control_trajectory_.clear();
          RCLCPP_WARN(get_logger(), "CDDP solve failed: %s",
                      result.status_message.c_str());
        }
      }
      it = pending_solves_.erase(it);
    }
    if (has_overrun_pending) {
      ++consecutive_overrun_cycles_;
    } else {
      consecutive_overrun_cycles_ = 0;
    }
  }

  void publishOffboardControlMode() {
    OffboardControlMode msg{};
    msg.timestamp = nowMicros();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = true;
    offboard_mode_pub_->publish(msg);
  }

  void publishVehicleThrustTorqueSetpoint(const Eigen::Vector4d &command) {
    VehicleThrustSetpoint thrust_msg{};
    thrust_msg.timestamp = nowMicros();
    const auto thrust_body = cddp_mpc::thrustBodyFromCommand(
        command(0), max_thrust_n_, thrust_norm_scale_);
    thrust_msg.xyz[0] = thrust_body[0];
    thrust_msg.xyz[1] = thrust_body[1];
    thrust_msg.xyz[2] = thrust_body[2];
    vehicle_thrust_pub_->publish(thrust_msg);

    VehicleTorqueSetpoint torque_msg{};
    torque_msg.timestamp = nowMicros();
    const Eigen::Vector3d torque_frd =
        cddp_mpc::bodyVectorFluToFrd(command.tail<3>());
    const auto torque_body =
        cddp_mpc::torqueBodyFromCommand(torque_frd, torqueNormalizationScale());
    torque_msg.xyz[0] = torque_body[0];
    torque_msg.xyz[1] = torque_body[1];
    torque_msg.xyz[2] = torque_body[2];
    vehicle_torque_pub_->publish(torque_msg);
  }

  void publishVehicleCommand(uint16_t command, float param1 = 0.0F,
                             float param2 = 0.0F) {
    VehicleCommand msg{};
    msg.timestamp = nowMicros();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void arm() {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    RCLCPP_INFO(get_logger(), "Sending arm command.");
  }

  void disarm() {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
    RCLCPP_INFO(get_logger(), "Sending disarm command.");
  }

  void engageOffboardMode() {
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
    RCLCPP_INFO(get_logger(), "Sending offboard mode command.");
  }

  bool stateAvailable() const {
    return current_position_ned_.has_value() && current_velocity_ned_.has_value() &&
           current_attitude_ned_.has_value() &&
           (setpoint_enu_.has_value() || reference_mode_ == "configured_target");
  }

  void setTakeoffSetpoint() {
    if (!current_position_ned_.has_value()) {
      return;
    }
    const Eigen::Vector3d current_enu = cddp_mpc::nedToEnu(*current_position_ned_);
    setpoint_enu_ =
        cddp_mpc::updateTakeoffSetpoint(current_enu, takeoff_altitude_m_, setpoint_enu_);
  }

  void updateLandingSetpoint(double now_s) {
    if (!setpoint_enu_.has_value() || !home_z_ned_.has_value() ||
        !landing_start_time_s_.has_value() ||
        !landing_start_setpoint_z_ned_.has_value()) {
      return;
    }

    const double elapsed = std::max(0.0, now_s - *landing_start_time_s_);
    const double target_ned =
        std::min(*home_z_ned_, *landing_start_setpoint_z_ned_ +
                                   landing_descent_rate_mps_ * elapsed);
    (*setpoint_enu_)(2) = -target_ned;
  }

  double activeTargetZNed() const {
    return -currentReferenceTargetEnu()(2);
  }

  Eigen::Matrix3d inertiaMatrix() const {
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    inertia(0, 0) = inertia_xx_kgm2_;
    inertia(1, 1) = inertia_yy_kgm2_;
    inertia(2, 2) = inertia_zz_kgm2_;
    return inertia;
  }

  Eigen::Vector3d maxBodyTorqueVector() const {
    return Eigen::Vector3d(std::max(0.0, max_roll_torque_nm_),
                           std::max(0.0, max_pitch_torque_nm_),
                           std::max(0.0, max_yaw_torque_nm_));
  }

  Eigen::Vector3d hoverTorqueLimitVector() const {
    const double hover_limit = std::max(0.0, hover_torque_limit_nm_);
    return Eigen::Vector3d(hover_limit, hover_limit, hover_limit);
  }

  Eigen::Vector3d torqueNormalizationScale() const {
    Eigen::Vector3d scale = Eigen::Vector3d::Zero();
    const Eigen::Vector3d max_torque = maxBodyTorqueVector();
    scale.x() = torque_norm_scale_x_ > 0.0
                    ? torque_norm_scale_x_
                    : 1.0 / std::max(max_torque.x(), 1e-6);
    scale.y() = torque_norm_scale_y_ > 0.0
                    ? torque_norm_scale_y_
                    : 1.0 / std::max(max_torque.y(), 1e-6);
    scale.z() = torque_norm_scale_z_ > 0.0
                    ? torque_norm_scale_z_
                    : 1.0 / std::max(max_torque.z(), 1e-6);
    return scale;
  }

  std::uint64_t nowMicros() {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  double nowSeconds() {
    return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
  }

  double currentBodyRateMagnitude() const {
    if (current_body_rates_.has_value()) {
      return current_body_rates_->norm();
    }
    return 0.0;
  }

  const char *stateSourceLabel() const {
    if (use_odometry_state_ && odometry_received_ && odom_frame_ok_) {
      return "odometry";
    }
    if (use_odometry_state_ && odometry_received_ && !odom_frame_ok_) {
      return "fallback";
    }
    return "legacy";
  }

  void debugLog(const Eigen::Vector4d &command) {
    if (!debug_logging_enabled_) {
      return;
    }

    const double current_z_ned =
        current_position_ned_.has_value()
            ? current_position_ned_->z()
            : std::numeric_limits<double>::quiet_NaN();
    const double rate_norm = currentBodyRateMagnitude();

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "mode=%s state=%s armed=%d offboard=%d z_ned=%.2f target_z_ned=%.2f "
        "rate_norm=%.2f source=%s indi=%d u=[%.2f, %.2f, %.2f, %.2f] solve_ms=%.1f status=%s",
        mode_.c_str(), stateSourceLabel(), armed_, offboard_enabled_, current_z_ned,
        activeTargetZNed(), rate_norm, last_command_source_.c_str(),
        enable_rotational_indi_ ? 1 : 0, command(0),
        command(1),
        command(2), command(3),
        last_solve_time_ms_, last_status_message_.c_str());
  }

  void publishStatusDiagnostics() {
    DiagnosticStatus status{};
    status.level = lastStatusIsOk() ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
    status.name = "cddp_mpc";
    status.message = mode_;
    status.values = buildDiagnosticValues();

    DiagnosticArray array{};
    array.status.push_back(std::move(status));
    diagnostic_pub_->publish(array);
  }

  bool lastStatusIsOk() const {
    return last_solve_success_;
  }

  std::vector<KeyValue> buildDiagnosticValues() const {
    std::vector<KeyValue> values;
    values.reserve(40);
    const double current_z_ned =
        current_position_ned_.has_value() ? current_position_ned_->z() : std::numeric_limits<double>::quiet_NaN();
    const double current_vz_ned =
        current_velocity_ned_.has_value() ? current_velocity_ned_->z() : std::numeric_limits<double>::quiet_NaN();
    const double altitude_error =
        std::isfinite(current_z_ned) ? std::abs(current_z_ned - activeTargetZNed()) : std::numeric_limits<double>::quiet_NaN();
    const double current_body_rate = currentBodyRateMagnitude();
    const double solver_cmd_thrust = last_solver_command_(0);
    const double published_cmd_thrust = last_published_command_(0);
    const Eigen::Vector3d solver_cmd_torque = last_solver_command_.tail<3>();
    const Eigen::Vector3d nominal_cmd_torque = last_nominal_command_.tail<3>();
    const Eigen::Vector3d corrected_cmd_torque = last_corrected_command_.tail<3>();
    const Eigen::Vector3d published_cmd_torque = last_published_command_.tail<3>();
    const cddp_mpc::IndiRotationalCompensator::Status *indi_status =
        rotational_indi_ ? &rotational_indi_->status() : &default_indi_status_;
    Eigen::Vector3d position_error_enu =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    if (current_position_ned_.has_value()) {
      position_error_enu =
          currentReferenceTargetEnu() - cddp_mpc::nedToEnu(*current_position_ned_);
    }

    auto push = [&values](const std::string &key, const std::string &value) {
      KeyValue kv{};
      kv.key = key;
      kv.value = value;
      values.push_back(std::move(kv));
    };

    push("solve_time_ms", formatDouble(last_solve_time_ms_));
    push("solve_success", lastStatusIsOk() ? "true" : "false");
    push("usable_solution", last_usable_solution_ ? "true" : "false");
    push("solve_status", last_status_message_);
    push("solve_fail_streak", std::to_string(solve_fail_streak_));
    push("solve_count", std::to_string(solve_count_));
    push("solve_timeout_count", std::to_string(solve_timeout_count_));
    push("solve_overrun_count", std::to_string(solve_overrun_count_));
    push("consecutive_overrun_cycles", std::to_string(consecutive_overrun_cycles_));
    push("plan_age_s", formatDouble(last_plan_age_s_));
    push("stale_plan_timeout_s", formatDouble(stale_plan_timeout_s_));
    push("stale_plan_reject_count", std::to_string(stale_plan_reject_count_));
    push("stale_plan_rejected_this_cycle",
         stale_plan_rejected_this_cycle_ ? "true" : "false");
    push("overrun_hover_active", overrun_hover_active_ ? "true" : "false");
    push("max_constraint_violation", formatDouble(last_max_constraint_violation_));
    push("raw_bound_violation", formatDouble(last_raw_bound_violation_));
    push("current_z_ned_m", formatDouble(current_z_ned));
    push("current_vz_ned_mps", formatDouble(current_vz_ned));
    push("current_body_rate_rad_s", formatDouble(current_body_rate));
    push("target_z_ned_m", formatDouble(activeTargetZNed()));
    push("home_z_ned_m", home_z_ned_.has_value() ? formatDouble(*home_z_ned_) : "nan");
    push("altitude_error_m", formatDouble(altitude_error));
    push("hover_thrust_n", formatDouble(hover_thrust_n_));
    push("mission_start_requested", mission_start_requested_ ? "true" : "false");
    push("auto_start_sequence", auto_start_sequence_ ? "true" : "false");
    push("auto_engage_offboard", auto_engage_offboard_ ? "true" : "false");
    push("auto_arm", auto_arm_ ? "true" : "false");
    push("landing_enabled", landing_enabled_ ? "true" : "false");
    push("landing_descent_rate_mps", formatDouble(landing_descent_rate_mps_));
    push("ready_thrust_n", formatDouble(ready_thrust_n_));
    push("fallback_hold_cycles", std::to_string(fallback_hold_cycles_));
    push("fallback_descent_rate_mps", formatDouble(fallback_descent_rate_mps_));
    push("solver_command_blend_alpha", formatDouble(solver_command_blend_alpha_));
    push("hover_vertical_kp_n_per_m", formatDouble(hover_vertical_kp_n_per_m_));
    push("hover_vertical_kd_n_per_mps", formatDouble(hover_vertical_kd_n_per_mps_));
    push("hover_vertical_correction_limit_n",
         formatDouble(hover_vertical_correction_limit_n_));
    push("hover_lateral_kp_nm_per_m",
         formatDouble(hover_lateral_kp_nm_per_m_));
    push("hover_lateral_kd_nm_per_mps",
         formatDouble(hover_lateral_kd_nm_per_mps_));
    push("hover_lateral_correction_limit_nm",
         formatDouble(hover_lateral_correction_limit_nm_));
    push("hover_torque_limit_nm", formatDouble(hover_torque_limit_nm_));
    push("command_validity_status", last_command_validity_status_);
    push("command_validity_failure_count",
         std::to_string(command_validity_failure_count_));
    push("command_validity_thrust_jump_n",
         formatDouble(command_validity_thrust_jump_n_));
    push("command_validity_torque_jump_nm",
         formatDouble(command_validity_torque_jump_nm_));
    push("arm_length_m", formatDouble(arm_length_m_));
    push("yaw_moment_coefficient", formatDouble(yaw_moment_coefficient_));
    push("min_motor_thrust_n", formatDouble(min_motor_thrust_n_));
    push("max_motor_thrust_n", formatDouble(max_motor_thrust_n_));
    push("max_roll_torque_nm", formatDouble(max_roll_torque_nm_));
    push("max_pitch_torque_nm", formatDouble(max_pitch_torque_nm_));
    push("max_yaw_torque_nm", formatDouble(max_yaw_torque_nm_));
    push("command_source", last_command_source_);
    push("fallback_active", last_safety_trace_.fallback_active ? "true" : "false");
    push("last_modified_safety_layer", last_safety_trace_.last_modified_layer);
    push("projection_changed_command",
         last_safety_trace_.projection_changed_command ? "true" : "false");
    push("safety_pipeline_order",
         "nominal->shaping->hover_vertical_correction->hover_lateral_correction->rotational_indi->clip_and_project->slew_limit->thrust_floor->publish");
    push("state_source", stateSourceLabel());
    push("odom_frame_ok", odom_frame_ok_ ? "true" : "false");
    push("position_error_enu_x_m", formatDouble(position_error_enu.x()));
    push("position_error_enu_y_m", formatDouble(position_error_enu.y()));
    push("position_error_enu_z_m", formatDouble(position_error_enu.z()));
    push("setpoint_enu_x_m", setpoint_enu_.has_value() ? formatDouble((*setpoint_enu_)(0)) : "nan");
    push("setpoint_enu_y_m", setpoint_enu_.has_value() ? formatDouble((*setpoint_enu_)(1)) : "nan");
    push("setpoint_enu_z_m", setpoint_enu_.has_value() ? formatDouble((*setpoint_enu_)(2)) : "nan");
    push("solver_cmd_thrust_n", formatDouble(solver_cmd_thrust));
    push("published_cmd_thrust_n", formatDouble(published_cmd_thrust));
    push("solver_cmd_tau_x_nm", formatDouble(solver_cmd_torque.x()));
    push("solver_cmd_tau_y_nm", formatDouble(solver_cmd_torque.y()));
    push("solver_cmd_tau_z_nm", formatDouble(solver_cmd_torque.z()));
    push("nominal_cmd_tau_x_nm", formatDouble(nominal_cmd_torque.x()));
    push("nominal_cmd_tau_y_nm", formatDouble(nominal_cmd_torque.y()));
    push("nominal_cmd_tau_z_nm", formatDouble(nominal_cmd_torque.z()));
    push("corrected_cmd_tau_x_nm", formatDouble(corrected_cmd_torque.x()));
    push("corrected_cmd_tau_y_nm", formatDouble(corrected_cmd_torque.y()));
    push("corrected_cmd_tau_z_nm", formatDouble(corrected_cmd_torque.z()));
    push("published_cmd_tau_x_nm", formatDouble(published_cmd_torque.x()));
    push("published_cmd_tau_y_nm", formatDouble(published_cmd_torque.y()));
    push("published_cmd_tau_z_nm", formatDouble(published_cmd_torque.z()));
    push("enable_rotational_indi", enable_rotational_indi_ ? "true" : "false");
    push("indi_active", indi_status->active ? "true" : "false");
    push("indi_disable_reason", last_indi_disable_reason_);
    push("indi_measurement_source", latest_rate_measurement_source_);
    push("indi_using_rpm_feedback",
         indi_status->using_rpm_feedback ? "true" : "false");
    push("indi_using_command_feedback_fallback",
         indi_status->using_command_feedback_fallback ? "true" : "false");
    push("indi_measurement_age_s", formatDouble(indi_status->measurement_age_s));
    push("indi_update_rate_hz", formatDouble(indi_status->update_rate_hz));
    push("indi_filtered_rate_x_rad_s", formatDouble(indi_status->filtered_rates.x()));
    push("indi_filtered_rate_y_rad_s", formatDouble(indi_status->filtered_rates.y()));
    push("indi_filtered_rate_z_rad_s", formatDouble(indi_status->filtered_rates.z()));
    push("indi_filtered_accel_x_rad_s2",
         formatDouble(indi_status->filtered_accel.x()));
    push("indi_filtered_accel_y_rad_s2",
         formatDouble(indi_status->filtered_accel.y()));
    push("indi_filtered_accel_z_rad_s2",
         formatDouble(indi_status->filtered_accel.z()));
    push("indi_torque_correction_x_nm",
         formatDouble(indi_status->torque_correction.x()));
    push("indi_torque_correction_y_nm",
         formatDouble(indi_status->torque_correction.y()));
    push("indi_torque_correction_z_nm",
         formatDouble(indi_status->torque_correction.z()));
    return values;
  }

  double control_rate_hz_{100.0};
  double solve_rate_hz_{20.0};
  double mpc_dt_{0.05};
  int horizon_steps_{20};
  double stale_plan_timeout_s_{0.25};
  int solve_overrun_hover_cycles_threshold_{3};

  double takeoff_altitude_m_{-3.0};
  double hover_duration_s_{20.0};
  double settle_tolerance_m_{0.3};
  double hover_entry_vz_threshold_mps_{0.35};
  double hover_entry_rate_threshold_rad_s_{0.3};
  double hover_entry_dwell_s_{0.6};
  double hover_handoff_duration_s_{1.5};
  double hover_reenter_error_m_{1.2};
  double hover_reenter_dwell_s_{2.0};
  bool landing_enabled_{true};
  double landing_descent_rate_mps_{0.3};
  double landing_touchdown_tolerance_m_{0.2};
  double landing_touchdown_vz_threshold_mps_{0.2};
  double ready_thrust_n_{0.0};
  int fallback_hold_cycles_{5};
  double fallback_descent_rate_mps_{0.2};
  double solver_command_blend_alpha_{0.3};
  double hover_vertical_kp_n_per_m_{0.0};
  double hover_vertical_kd_n_per_mps_{0.0};
  double hover_vertical_correction_limit_n_{0.0};
  double hover_lateral_kp_nm_per_m_{0.0};
  double hover_lateral_kd_nm_per_mps_{0.0};
  double hover_lateral_correction_limit_nm_{0.0};
  double hover_torque_limit_nm_{0.12};

  bool auto_start_sequence_{true};
  bool auto_engage_offboard_{true};
  bool auto_arm_{true};
  bool use_odometry_state_{true};

  double mass_kg_{1.35};
  double gravity_mps2_{9.81};
  double hover_thrust_n_{0.0};
  double min_thrust_n_{0.0};
  double max_thrust_n_{20.0};
  double thrust_norm_scale_{0.0};
  double arm_length_m_{0.25};
  double inertia_xx_kgm2_{0.029};
  double inertia_yy_kgm2_{0.029};
  double inertia_zz_kgm2_{0.055};
  double yaw_moment_coefficient_{0.1};
  double min_motor_thrust_n_{0.0};
  double max_motor_thrust_n_{0.0};
  double max_roll_torque_nm_{0.0};
  double max_pitch_torque_nm_{0.0};
  double max_yaw_torque_nm_{0.0};
  double torque_norm_scale_x_{0.0};
  double torque_norm_scale_y_{0.0};
  double torque_norm_scale_z_{0.0};
  double target_yaw_rad_{0.0};
  std::string solver_type_{"CLDDP"};

  double pos_weight_{1.5};
  double vel_weight_{4.0};
  double yaw_weight_{0.7};
  double tilt_weight_{4.0};
  double thrust_weight_{0.05};
  double rate_weight_{0.05};
  double terminal_pos_weight_{8.0};
  double terminal_vel_weight_{4.0};
  double terminal_yaw_weight_{3.0};
  double terminal_tilt_weight_{8.0};
  double reference_max_climb_rate_mps_{1.0};
  double reference_max_climb_accel_mps2_{1.5};
  double reference_max_yaw_rate_rad_s_{0.6};
  double reference_smoothing_tau_s_{0.6};
  double reference_trajectory_period_s_{8.0};
  double reference_circle_radius_m_{0.5};
  double reference_figure_eight_amplitude_m_{0.5};
  double reference_vertical_amplitude_m_{0.0};
  bool reference_allow_jump_{false};
  std::string reference_mode_{"active_setpoint"};
  std::string reference_topic_{""};
  double target_x_m_{0.0};
  double target_y_m_{0.0};
  double target_z_m_{-3.0};

  int max_iterations_{8};
  double constraint_tolerance_{0.5};
  double solve_timeout_ms_{450.0};
  double takeoff_min_thrust_margin_n_{1.0};
  double min_flight_thrust_ratio_{0.55};
  double hover_min_thrust_ratio_{1.0};
  double landing_min_thrust_ratio_{0.3};
  double thrust_slew_rate_n_per_s_{8.0};
  double body_torque_slew_nm_per_s_{2.5};
  double command_validity_thrust_jump_n_{8.0};
  double command_validity_torque_jump_nm_{0.6};
  bool debug_logging_enabled_{true};
  bool enable_rotational_indi_{false};
  double indi_blend_alpha_{0.25};
  double indi_rate_lpf_cutoff_hz_{35.0};
  double indi_accel_lpf_cutoff_hz_{20.0};
  double indi_torque_correction_limit_nm_{0.12};
  double indi_max_measurement_age_s_{0.1};
  double indi_max_body_rate_rad_s_{12.0};
  bool indi_use_rpm_feedback_{false};
  bool indi_debug_logging_enabled_{false};
  std::size_t max_pending_solve_requests_{2};
  cddp_mpc::FrameAdapterConfig odom_frame_config_{};

  std::optional<Eigen::Vector3d> current_position_ned_;
  std::optional<Eigen::Vector3d> current_velocity_ned_;
  std::optional<Eigen::Quaterniond> current_attitude_ned_;
  // Stored as normalized FRD body rates once the PX4 message is adapted.
  std::optional<Eigen::Vector3d> current_body_rates_;
  std::optional<Eigen::Vector3d> setpoint_enu_;
  std::optional<double> home_z_ned_;
  std::optional<double> hover_start_time_s_;
  std::optional<double> hover_in_band_since_s_;
  std::optional<double> hover_out_of_band_since_s_;
  std::optional<double> landing_start_time_s_;
  std::optional<double> landing_start_setpoint_z_ned_;
  std::optional<double> latest_reference_thrust_n_;

  bool mission_start_requested_{true};
  bool armed_{false};
  bool offboard_enabled_{false};
  bool odometry_received_{false};
  bool odom_frame_ok_{true};
  std::string mode_{"INIT"};
  double mode_request_time_{0.0};

  Eigen::Vector4d last_solver_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_good_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_published_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_nominal_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_corrected_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector3d last_indi_correction_{Eigen::Vector3d::Zero()};
  std::string last_indi_disable_reason_{"disabled_by_param"};
  std::string last_command_source_{"solver"};
  std::string last_command_validity_status_{"ok"};
  SafetyTrace last_safety_trace_{};
  std::string last_status_message_{"not_started"};
  bool last_solve_success_{false};
  bool last_usable_solution_{false};
  double last_solve_time_ms_{0.0};
  double last_solve_wall_time_s_{0.0};
  double last_max_constraint_violation_{std::numeric_limits<double>::quiet_NaN()};
  double last_raw_bound_violation_{std::numeric_limits<double>::quiet_NaN()};
  int solve_fail_streak_{0};
  int solve_count_{0};
  int solve_timeout_count_{0};
  int solve_overrun_count_{0};
  int stale_plan_reject_count_{0};
  int command_validity_failure_count_{0};
  int consecutive_overrun_cycles_{0};
  double last_plan_age_s_{std::numeric_limits<double>::quiet_NaN()};
  bool stale_plan_rejected_this_cycle_{false};
  bool overrun_hover_active_{false};
  std::vector<Eigen::VectorXd> last_planned_control_trajectory_;

  std::optional<std::vector<Eigen::VectorXd>> previous_state_guess_;
  std::optional<std::vector<Eigen::VectorXd>> previous_control_guess_;
  mutable std::mutex warm_start_mutex_;
  std::vector<PendingSolve> pending_solves_;
  std::uint64_t next_solve_request_id_{0};
  std::uint64_t latest_applied_request_id_{0};

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
#if CDDP_MPC_HAS_VEHICLE_ANGULAR_VELOCITY
  rclcpp::Subscription<VehicleAngularVelocity>::SharedPtr angular_velocity_sub_;
#endif

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_pub_;
  rclcpp::Publisher<VehicleTorqueSetpoint>::SharedPtr vehicle_torque_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostic_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr solve_timer_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;

  std::unique_ptr<cddp_mpc::ReferenceProvider> reference_provider_;
  std::unique_ptr<cddp_mpc::IndiRotationalCompensator> rotational_indi_;
  cddp_mpc::IndiRotationalCompensator::Status default_indi_status_{};
  std::string latest_rate_measurement_source_{"none"};
};

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PX4MPCNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
