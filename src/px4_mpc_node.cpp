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
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cddp.hpp"
#include "dynamics_model/quadrotor_rate.hpp"
#include "cddp_mpc/px4_utils.hpp"

using namespace std::chrono_literals;

namespace {

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitude;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleRatesSetpoint;
using px4_msgs::msg::VehicleStatus;

struct SolveResult {
  std::uint64_t request_id{0};
  bool success{false};
  std::string status_message{"not_started"};
  double solve_time_ms{0.0};
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

class PX4MPCNode : public rclcpp::Node {
public:
  explicit PX4MPCNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("px4_mpc_node", options),
        mission_start_requested_(auto_start_sequence_),
        last_solver_command_(hoverCommand(0.0)),
        last_published_command_(hoverCommand(0.0)) {
    loadParameters();
    mission_start_requested_ = auto_start_sequence_;
    hover_thrust_n_ = hover_thrust_n_ > 0.0 ? hover_thrust_n_ : mass_kg_ * gravity_mps2_;
    last_solver_command_ = hoverCommand(0.0);
    last_published_command_ = last_solver_command_;

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

    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos);
    vehicle_rates_pub_ = create_publisher<VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", qos);
    vehicle_command_pub_ = create_publisher<VehicleCommand>(
        "/fmu/in/vehicle_command", qos);

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
  }

private:
  void loadParameters() {
    control_rate_hz_ = declareOrGet("control_rate_hz", 50.0);
    solve_rate_hz_ = declareOrGet("solve_rate_hz", 10.0);
    mpc_dt_ = declareOrGet("mpc_dt", 0.1);
    horizon_steps_ = declareOrGet("horizon_steps", 20);

    takeoff_altitude_m_ = declareOrGet("takeoff_altitude_m", -3.0);
    hover_duration_s_ = declareOrGet("hover_duration_s", 20.0);
    settle_tolerance_m_ = declareOrGet("settle_tolerance_m", 0.3);
    hover_entry_vz_threshold_mps_ =
        declareOrGet("hover_entry_vz_threshold_mps", 0.35);
    hover_entry_rate_threshold_rad_s_ =
        declareOrGet("hover_entry_rate_threshold_rad_s", 0.3);
    hover_entry_dwell_s_ = declareOrGet("hover_entry_dwell_s", 0.6);
    hover_reenter_error_m_ = declareOrGet("hover_reenter_error_m", 1.2);
    hover_reenter_dwell_s_ = declareOrGet("hover_reenter_dwell_s", 2.0);
    landing_enabled_ = declareOrGet("landing_enabled", true);
    landing_descent_rate_mps_ = declareOrGet("landing_descent_rate_mps", 0.3);
    landing_touchdown_tolerance_m_ =
        declareOrGet("landing_touchdown_tolerance_m", 0.2);
    landing_touchdown_vz_threshold_mps_ =
        declareOrGet("landing_touchdown_vz_threshold_mps", 0.2);

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
    max_body_rate_rad_s_ = declareOrGet("max_body_rate_rad_s", 0.8);
    target_yaw_rad_ = declareOrGet("target_yaw_rad", 0.0);
    px4_roll_rate_sign_ = declareOrGet("px4_roll_rate_sign", 1.0);
    px4_pitch_rate_sign_ = declareOrGet("px4_pitch_rate_sign", 1.0);
    px4_yaw_rate_sign_ = declareOrGet("px4_yaw_rate_sign", 1.0);
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

    pos_weight_ = declareOrGet("pos_weight", 1.5);
    vel_weight_ = declareOrGet("vel_weight", 4.0);
    yaw_weight_ = declareOrGet("yaw_weight", 0.7);
    tilt_weight_ = declareOrGet("tilt_weight", 4.0);
    thrust_weight_ = declareOrGet("thrust_weight", 0.05);
    rate_weight_ = declareOrGet("rate_weight", 0.05);
    terminal_pos_weight_ = declareOrGet("terminal_pos_weight", 8.0);
    terminal_vel_weight_ = declareOrGet("terminal_vel_weight", 4.0);
    terminal_yaw_weight_ = declareOrGet("terminal_yaw_weight", 3.0);
    terminal_tilt_weight_ = declareOrGet("terminal_tilt_weight", 8.0);

    max_iterations_ = declareOrGet("max_iterations", 8);
    solve_timeout_ms_ = declareOrGet("solve_timeout_ms", 450.0);
    max_pending_solve_requests_ = static_cast<std::size_t>(
        std::max(1, declareOrGet("max_pending_solve_requests", 2)));
    takeoff_min_thrust_margin_n_ =
        declareOrGet("takeoff_min_thrust_margin_n", 1.0);
    min_flight_thrust_ratio_ = declareOrGet("min_flight_thrust_ratio", 0.55);
    hover_min_thrust_ratio_ = declareOrGet("hover_min_thrust_ratio", 1.0);
    landing_min_thrust_ratio_ = declareOrGet("landing_min_thrust_ratio", 0.3);
    thrust_slew_rate_n_per_s_ =
        declareOrGet("thrust_slew_rate_n_per_s", 8.0);
    body_rate_slew_rad_s2_ =
        declareOrGet("body_rate_slew_rad_s2", 4.0);
    debug_logging_enabled_ = declareOrGet("debug_logging_enabled", true);
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
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Waiting for PX4 state topics...");
      return;
    }

    pollSolveResult();
    updateMode();

    Eigen::Vector4d command = selectCommandForPublish();
    command = enforceModeFloors(command);
    command = applySlewLimits(command);
    command = clipCommand(command);
    last_published_command_ = command;
    publishVehicleRatesSetpoint(command);
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

    const Eigen::VectorXd initial_state = buildCurrentStateEnu();
    const std::vector<Eigen::VectorXd> reference_states =
        buildReferenceTrajectory(*setpoint_enu_);
    const bool landing_mode = mode_ == "LAND";
    std::vector<Eigen::VectorXd> initial_state_guess;
    std::vector<Eigen::VectorXd> initial_control_guess;
    {
      std::lock_guard<std::mutex> lock(warm_start_mutex_);
      initial_state_guess = buildInitialStateGuessUnlocked(initial_state, reference_states);
      initial_control_guess = buildInitialControlGuessUnlocked();
    }

    const std::uint64_t request_id = ++next_solve_request_id_;
    pending_solves_.push_back(PendingSolve{
        request_id,
        nowSeconds(),
        false,
        std::async(std::launch::async,
                   [this, request_id, initial_state, reference_states,
                    initial_state_guess, initial_control_guess, landing_mode]() {
                     return solveMPC(request_id, initial_state, reference_states,
                                     initial_state_guess, initial_control_guess,
                                     landing_mode);
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
                       const std::vector<Eigen::VectorXd> &reference_states,
                       const std::vector<Eigen::VectorXd> &initial_state_guess,
                       const std::vector<Eigen::VectorXd> &initial_control_guess,
                       bool landing_mode) {
    SolveResult result;
    result.request_id = request_id;
    result.command = hoverCommand(landing_mode ? landing_descent_rate_mps_ : 0.0);

    try {
      auto solver = createSolver(reference_states.back());
      solver->setInitialState(initial_state);
      solver->setReferenceState(reference_states.back());
      solver->setReferenceStates(reference_states);
      solver->setInitialTrajectory(initial_state_guess, initial_control_guess);

      const cddp::CDDPSolution solution = solver->solve("MSIPDDP");
      result.status_message = solution.status_message;
      result.solve_time_ms = solution.solve_time_ms;
      if (!solution.control_trajectory.empty()) {
        result.command = solution.control_trajectory.front().head<4>();
        result.success = true;
        result.state_trajectory = solution.state_trajectory;
        result.control_trajectory = solution.control_trajectory;
      }
    } catch (const std::exception &ex) {
      result.status_message = ex.what();
    }

    return result;
  }

  std::unique_ptr<cddp::CDDP> createSolver(const Eigen::VectorXd &reference_state) const {
    auto system = std::make_unique<cddp::QuadrotorRate>(
        mpc_dt_, mass_kg_, max_thrust_n_, max_body_rate_rad_s_, "rk4");

    const int state_dim = 10;
    const int control_dim = 4;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Q.block<3, 3>(0, 0).diagonal().setConstant(pos_weight_);
    Q.block<3, 3>(3, 3).diagonal().setConstant(vel_weight_);
    Q(6, 6) = 0.1 * yaw_weight_;
    Q(7, 7) = tilt_weight_;
    Q(8, 8) = tilt_weight_;
    Q(9, 9) = yaw_weight_;

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(control_dim, control_dim);
    R(0, 0) = thrust_weight_;
    R(1, 1) = rate_weight_;
    R(2, 2) = rate_weight_;
    R(3, 3) = rate_weight_;

    Eigen::MatrixXd Qf = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Qf.block<3, 3>(0, 0).diagonal().setConstant(terminal_pos_weight_);
    Qf.block<3, 3>(3, 3).diagonal().setConstant(terminal_vel_weight_);
    Qf(6, 6) = 0.1 * terminal_yaw_weight_;
    Qf(7, 7) = terminal_tilt_weight_;
    Qf(8, 8) = terminal_tilt_weight_;
    Qf(9, 9) = terminal_yaw_weight_;

    auto objective = std::make_unique<cddp::QuadraticObjective>(
        Q, R, Qf, reference_state, std::vector<Eigen::VectorXd>(), mpc_dt_);

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
        reference_state, reference_state, horizon_steps_, mpc_dt_, std::move(system),
        std::move(objective), options);

    Eigen::VectorXd lower_bound(control_dim);
    lower_bound << min_thrust_n_, -max_body_rate_rad_s_, -max_body_rate_rad_s_,
        -max_body_rate_rad_s_;
    Eigen::VectorXd upper_bound(control_dim);
    upper_bound << max_thrust_n_, max_body_rate_rad_s_, max_body_rate_rad_s_,
        max_body_rate_rad_s_;
    solver->addPathConstraint(
        "control_bounds",
        std::make_unique<cddp::ControlConstraint>(lower_bound, upper_bound));
    return solver;
  }

  Eigen::VectorXd buildCurrentStateEnu() const {
    const Eigen::Vector3d position_enu = cddp_mpc::nedToEnu(*current_position_ned_);
    const Eigen::Vector3d velocity_enu = cddp_mpc::nedToEnu(*current_velocity_ned_);
    const Eigen::Quaterniond attitude_enu =
        cddp_mpc::quatNedToEnuWxyz(*current_attitude_ned_);

    Eigen::VectorXd state(10);
    state.segment<3>(0) = position_enu;
    state.segment<3>(3) = velocity_enu;
    state(6) = attitude_enu.w();
    state(7) = attitude_enu.x();
    state(8) = attitude_enu.y();
    state(9) = attitude_enu.z();
    return state;
  }

  std::vector<Eigen::VectorXd>
  buildReferenceTrajectory(const Eigen::Vector3d &setpoint_enu) const {
    std::vector<Eigen::VectorXd> reference_states;
    reference_states.reserve(static_cast<std::size_t>(horizon_steps_ + 1));
    const Eigen::Quaterniond q_ref = cddp_mpc::quaternionFromYaw(target_yaw_rad_);

    for (int i = 0; i <= horizon_steps_; ++i) {
      Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
      state.segment<3>(0) = setpoint_enu;
      state(6) = q_ref.w();
      state(7) = q_ref.x();
      state(8) = q_ref.y();
      state(9) = q_ref.z();
      reference_states.push_back(state);
    }

    return reference_states;
  }

  std::vector<Eigen::VectorXd>
  buildInitialStateGuessUnlocked(const Eigen::VectorXd &initial_state,
                                 const std::vector<Eigen::VectorXd> &reference_states) const {
    if (previous_state_guess_.has_value() &&
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

  std::vector<Eigen::VectorXd> buildInitialControlGuessUnlocked() const {
    if (previous_control_guess_.has_value() &&
        previous_control_guess_->size() == static_cast<std::size_t>(horizon_steps_)) {
      return *previous_control_guess_;
    }

    std::vector<Eigen::VectorXd> guess;
    guess.reserve(static_cast<std::size_t>(horizon_steps_));
    for (int i = 0; i < horizon_steps_; ++i) {
      Eigen::VectorXd u(4);
      u << hover_thrust_n_, 0.0, 0.0, 0.0;
      guess.push_back(u);
    }
    return guess;
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

  Eigen::Vector4d selectCommandForPublish() const {
    if (mode_ == "READY") {
      return Eigen::Vector4d::Zero();
    }
    if (mode_ == "LAND_DONE") {
      return Eigen::Vector4d::Zero();
    }
    return last_solver_command_;
  }

  Eigen::Vector4d hoverCommand(double descent_rate_mps) const {
    const double descent_scale =
        std::clamp(descent_rate_mps / std::max(landing_descent_rate_mps_, 1e-3), 0.0, 1.0);
    const double thrust_ratio = 1.0 - 0.2 * descent_scale;
    Eigen::Vector4d command;
    command << hover_thrust_n_ * thrust_ratio, 0.0, 0.0, 0.0;
    return command;
  }

  Eigen::Vector4d enforceModeFloors(const Eigen::Vector4d &command) const {
    Eigen::Vector4d shaped = command;
    if (mode_ == "TAKEOFF") {
      shaped(0) = std::max(shaped(0), hover_thrust_n_ + takeoff_min_thrust_margin_n_);
    } else if (mode_ == "HOVER") {
      shaped(0) = std::max(
          shaped(0),
          hover_thrust_n_ *
              std::max({1.0, min_flight_thrust_ratio_, hover_min_thrust_ratio_}));
    } else if (mode_ == "LAND") {
      shaped(0) = std::max(shaped(0),
                           hover_thrust_n_ * std::max(0.0, landing_min_thrust_ratio_));
    }
    return shaped;
  }

  Eigen::Vector4d applySlewLimits(const Eigen::Vector4d &command) const {
    Eigen::Vector4d limited = command;
    const double dt = 1.0 / std::max(control_rate_hz_, 1e-3);
    const double max_thrust_step = thrust_slew_rate_n_per_s_ * dt;
    const double max_rate_step = body_rate_slew_rad_s2_ * dt;

    limited(0) = std::clamp(limited(0), last_published_command_(0) - max_thrust_step,
                            last_published_command_(0) + max_thrust_step);
    for (int i = 1; i < 4; ++i) {
      limited(i) = std::clamp(limited(i), last_published_command_(i) - max_rate_step,
                              last_published_command_(i) + max_rate_step);
    }
    return limited;
  }

  Eigen::Vector4d clipCommand(const Eigen::Vector4d &command) const {
    Eigen::Vector4d clipped = command;
    clipped(0) = std::clamp(clipped(0), min_thrust_n_, max_thrust_n_);
    for (int i = 1; i < 4; ++i) {
      clipped(i) =
          std::clamp(clipped(i), -max_body_rate_rad_s_, max_body_rate_rad_s_);
    }
    return clipped;
  }

  void pollSolveResult() {
    auto it = pending_solves_.begin();
    while (it != pending_solves_.end()) {
      if (it->future.wait_for(0ms) != std::future_status::ready) {
        ++it;
        continue;
      }

      SolveResult result = it->future.get();
      if (result.request_id > latest_applied_request_id_) {
        latest_applied_request_id_ = result.request_id;
        if (result.success) {
          last_solver_command_ = result.command;
          last_status_message_ = result.status_message;
          last_solve_time_ms_ = result.solve_time_ms;
          std::lock_guard<std::mutex> lock(warm_start_mutex_);
          previous_state_guess_ = shiftStateTrajectory(result.state_trajectory);
          previous_control_guess_ = shiftControlTrajectory(result.control_trajectory);
        } else {
          last_solver_command_ =
              hoverCommand(mode_ == "LAND" ? landing_descent_rate_mps_ : 0.0);
          last_status_message_ = result.status_message;
          last_solve_time_ms_ = result.solve_time_ms;
          RCLCPP_WARN(get_logger(), "CDDP solve failed: %s",
                      result.status_message.c_str());
        }
      }
      it = pending_solves_.erase(it);
    }
  }

  void publishOffboardControlMode() {
    OffboardControlMode msg{};
    msg.timestamp = nowMicros();
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;
    offboard_mode_pub_->publish(msg);
  }

  void publishVehicleRatesSetpoint(const Eigen::Vector4d &command) {
    VehicleRatesSetpoint msg{};
    msg.timestamp = nowMicros();
    msg.roll = static_cast<float>(px4_roll_rate_sign_ * command(1));
    msg.pitch = static_cast<float>(px4_pitch_rate_sign_ * command(2));
    msg.yaw = static_cast<float>(px4_yaw_rate_sign_ * command(3));
    const auto thrust_body = cddp_mpc::thrustBodyFromCommand(
        command(0), max_thrust_n_, thrust_norm_scale_);
    msg.thrust_body[0] = thrust_body[0];
    msg.thrust_body[1] = thrust_body[1];
    msg.thrust_body[2] = thrust_body[2];
    vehicle_rates_pub_->publish(msg);
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
           current_attitude_ned_.has_value() && setpoint_enu_.has_value();
  }

  void setTakeoffSetpoint() {
    if (!current_position_ned_.has_value()) {
      return;
    }
    const Eigen::Vector3d current_enu = cddp_mpc::nedToEnu(*current_position_ned_);
    setpoint_enu_ =
        Eigen::Vector3d(current_enu.x(), current_enu.y(), -takeoff_altitude_m_);
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
    if (!setpoint_enu_.has_value()) {
      return takeoff_altitude_m_;
    }
    return -(*setpoint_enu_)(2);
  }

  std::uint64_t nowMicros() const {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  double nowSeconds() const {
    return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
  }

  double currentBodyRateMagnitude() const {
    if (current_body_rates_.has_value()) {
      return current_body_rates_->norm();
    }
    return last_published_command_.tail<3>().norm();
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

  void debugLog(const Eigen::Vector4d &command) const {
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
        "rate_norm=%.2f u=[%.2f, %.2f, %.2f, %.2f] solve_ms=%.1f status=%s",
        mode_.c_str(), stateSourceLabel(), armed_, offboard_enabled_, current_z_ned,
        activeTargetZNed(), rate_norm, command(0), command(1),
        command(2), command(3),
        last_solve_time_ms_, last_status_message_.c_str());
  }

  double control_rate_hz_{50.0};
  double solve_rate_hz_{10.0};
  double mpc_dt_{0.1};
  int horizon_steps_{20};

  double takeoff_altitude_m_{-3.0};
  double hover_duration_s_{20.0};
  double settle_tolerance_m_{0.3};
  double hover_entry_vz_threshold_mps_{0.35};
  double hover_entry_rate_threshold_rad_s_{0.3};
  double hover_entry_dwell_s_{0.6};
  double hover_reenter_error_m_{1.2};
  double hover_reenter_dwell_s_{2.0};
  bool landing_enabled_{true};
  double landing_descent_rate_mps_{0.3};
  double landing_touchdown_tolerance_m_{0.2};
  double landing_touchdown_vz_threshold_mps_{0.2};

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
  double max_body_rate_rad_s_{0.8};
  double target_yaw_rad_{0.0};
  double px4_roll_rate_sign_{1.0};
  double px4_pitch_rate_sign_{1.0};
  double px4_yaw_rate_sign_{1.0};

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

  int max_iterations_{8};
  double solve_timeout_ms_{450.0};
  double takeoff_min_thrust_margin_n_{1.0};
  double min_flight_thrust_ratio_{0.55};
  double hover_min_thrust_ratio_{1.0};
  double landing_min_thrust_ratio_{0.3};
  double thrust_slew_rate_n_per_s_{8.0};
  double body_rate_slew_rad_s2_{4.0};
  bool debug_logging_enabled_{true};
  std::size_t max_pending_solve_requests_{2};
  cddp_mpc::FrameAdapterConfig odom_frame_config_{};

  std::optional<Eigen::Vector3d> current_position_ned_;
  std::optional<Eigen::Vector3d> current_velocity_ned_;
  std::optional<Eigen::Quaterniond> current_attitude_ned_;
  std::optional<Eigen::Vector3d> current_body_rates_;
  std::optional<Eigen::Vector3d> setpoint_enu_;
  std::optional<double> home_z_ned_;
  std::optional<double> hover_start_time_s_;
  std::optional<double> hover_in_band_since_s_;
  std::optional<double> hover_out_of_band_since_s_;
  std::optional<double> landing_start_time_s_;
  std::optional<double> landing_start_setpoint_z_ned_;

  bool mission_start_requested_{true};
  bool armed_{false};
  bool offboard_enabled_{false};
  bool odometry_received_{false};
  bool odom_frame_ok_{true};
  std::string mode_{"INIT"};
  double mode_request_time_{0.0};

  Eigen::Vector4d last_solver_command_{Eigen::Vector4d::Zero()};
  Eigen::Vector4d last_published_command_{Eigen::Vector4d::Zero()};
  std::string last_status_message_{"not_started"};
  double last_solve_time_ms_{0.0};

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

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr solve_timer_;
  rclcpp::TimerBase::SharedPtr offboard_timer_;
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
