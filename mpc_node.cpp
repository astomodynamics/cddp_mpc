/**
 * @file mpc_node.cpp
 * @brief Implementation of Model Predictive Control node for PX4 quadrotor
 */

#include "quadrotor_mpc/mpc_node.hpp"
#include "cddp.hpp"
#include "dynamics_model/quadrotor_rate.hpp"
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>

MPCNodePX4::MPCNodePX4() : Node("mpc_node_px4") {
    // Declare parameters
    this->declare_parameter<std::string>("robot_id", "px4_robot");
    this->declare_parameter<double>("timestep_", 0.05);
    this->declare_parameter<int>("horizon_", 20);
    this->declare_parameter<double>("processing_frequency_", 50.0);
    this->declare_parameter<int>("goal_index_", 7);
    this->declare_parameter<double>("max_compute_time_", 0.1);
    
    // Quadrotor physical parameters (Rate-based model)
    // Match Python model defaults for proper thrust scaling
    this->declare_parameter<double>("mass_", 1.0);           // kg - match Python model
    this->declare_parameter<double>("max_thrust_", 13.44);   // N - match Python (9.81/0.73)
    this->declare_parameter<double>("max_rate_", 0.5);       // rad/s - match Python model

    // Cost parameters matching Python implementation weights
    this->declare_parameter<double>("Q_px_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("Q_py_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("Q_pz_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("Q_qw_", 0.0);   // 0.0 like Python
    this->declare_parameter<double>("Q_qx_", 0.1);   // 2*0.1 like Python
    this->declare_parameter<double>("Q_qy_", 0.1);   // 2*0.1 like Python
    this->declare_parameter<double>("Q_qz_", 0.1);   // 2*0.1 like Python
    this->declare_parameter<double>("Q_vx_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("Q_vy_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("Q_vz_", 10.0);  // 2*1e1 like Python

    this->declare_parameter<double>("R_thrust_", 10.0);  // 2*1e1 like Python
    this->declare_parameter<double>("R_wx_", 500.0);     // 2*5e2 like Python
    this->declare_parameter<double>("R_wy_", 500.0);     // 2*5e2 like Python
    this->declare_parameter<double>("R_wz_", 500.0);     // 2*5e2 like Python

    this->declare_parameter<double>("Qf_px_", 300.0);  // 2*3e2 like Python
    this->declare_parameter<double>("Qf_py_", 300.0);  // 2*3e2 like Python
    this->declare_parameter<double>("Qf_pz_", 300.0);  // 2*3e2 like Python
    this->declare_parameter<double>("Qf_qw_", 0.0);    // 0.0 like Python
    this->declare_parameter<double>("Qf_qx_", 0.0);    // 0.0 like Python
    this->declare_parameter<double>("Qf_qy_", 0.0);    // 0.0 like Python
    this->declare_parameter<double>("Qf_qz_", 0.0);    // 0.0 like Python
    this->declare_parameter<double>("Qf_vx_", 100.0);  // 2*1e2 like Python
    this->declare_parameter<double>("Qf_vy_", 100.0);  // 2*1e2 like Python
    this->declare_parameter<double>("Qf_vz_", 100.0);  // 2*1e2 like Python
    this->declare_parameter<bool>("enable_warm_start_", true);

    // Get parameters
    robot_id_ = this->get_parameter("robot_id").as_string();
    this->get_parameter("timestep_", timestep_);
    this->get_parameter("horizon_", horizon_);
    this->get_parameter("processing_frequency_", processing_frequency_);
    this->get_parameter("goal_index_", goal_index_);
    this->get_parameter("max_compute_time_", max_compute_time_);
    
    this->get_parameter("mass_", mass_);
    this->get_parameter("max_thrust_", max_thrust_);
    this->get_parameter("max_rate_", max_rate_);

    // Get cost parameters
    this->get_parameter("Q_px_", Q_px_);
    this->get_parameter("Q_py_", Q_py_);
    this->get_parameter("Q_pz_", Q_pz_);
    this->get_parameter("Q_qw_", Q_qw_);
    this->get_parameter("Q_qx_", Q_qx_);
    this->get_parameter("Q_qy_", Q_qy_);
    this->get_parameter("Q_qz_", Q_qz_);
    this->get_parameter("Q_vx_", Q_vx_);
    this->get_parameter("Q_vy_", Q_vy_);
    this->get_parameter("Q_vz_", Q_vz_);
    
    this->get_parameter("R_thrust_", R_thrust_);
    this->get_parameter("R_wx_", R_wx_);
    this->get_parameter("R_wy_", R_wy_);
    this->get_parameter("R_wz_", R_wz_);

    this->get_parameter("Qf_px_", Qf_px_);
    this->get_parameter("Qf_py_", Qf_py_);
    this->get_parameter("Qf_pz_", Qf_pz_);
    this->get_parameter("Qf_qw_", Qf_qw_);
    this->get_parameter("Qf_qx_", Qf_qx_);
    this->get_parameter("Qf_qy_", Qf_qy_);
    this->get_parameter("Qf_qz_", Qf_qz_);
    this->get_parameter("Qf_vx_", Qf_vx_);
    this->get_parameter("Qf_vy_", Qf_vy_);
    this->get_parameter("Qf_vz_", Qf_vz_);

    this->get_parameter("enable_warm_start_", enable_warm_start_);

    // Validate parameters
    if (!validateParameters()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter validation failed. Node will not start.");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "PX4 MPC Node initialized for Quadrotor");

    // PX4 QoS profile (best effort, transient local)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::TransientLocal);

    // PX4 Subscribers
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos_profile,
        std::bind(&MPCNodePX4::localPositionCallback, this, std::placeholders::_1));

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos_profile,
        std::bind(&MPCNodePX4::attitudeCallback, this, std::placeholders::_1));

    // Use vehicle_odometry for angular velocity since PX4 doesn't publish vehicle_angular_velocity separately
    odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&MPCNodePX4::odometryCallback, this, std::placeholders::_1));

    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", qos_profile,
        std::bind(&MPCNodePX4::vehicleStatusCallback, this, std::placeholders::_1));

    // Goal/Path Subscribers
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&MPCNodePX4::goalCallback, this, std::placeholders::_1));

    path_subscription_ = create_subscription<nav_msgs::msg::Path>(
        "/global_path", 10, std::bind(&MPCNodePX4::pathCallback, this, std::placeholders::_1));

    // PX4 Publishers
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos_profile);

    // Rate-based control publishers
    rates_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
        "/fmu/in/vehicle_rates_setpoint", qos_profile);
    thrust_setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
        "/fmu/in/vehicle_thrust_setpoint", qos_profile);

    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos_profile);

    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);

    // Initialize state vectors (Rate-based model: 10 states)
    current_state_ = Eigen::VectorXd::Zero(10);
    goal_state_ = Eigen::VectorXd::Zero(10);
    setpoint_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);  // Default setpoint at origin
    goal_reception_time_ = this->now();
    
    // Initialize offboard mode control
    offboard_mode_requested_ = false;
    arm_requested_ = false;

    // Control loop timer
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds((int)(1000.0/processing_frequency_)), 
        std::bind(&MPCNodePX4::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "PX4 MPC Node setup complete. Using rate-based control.");
}

void MPCNodePX4::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    // PX4 uses NED frame, convert to ENU exactly like Python implementation
    // Python: self.vehicle_local_position[0] = msg.y
    //         self.vehicle_local_position[1] = msg.x
    //         self.vehicle_local_position[2] = -msg.z
    
    // Store absolute position in ENU frame (matching Python)
    Eigen::Vector3d vehicle_position_enu;
    vehicle_position_enu(0) = msg->y;    // Y -> X (ENU)
    vehicle_position_enu(1) = msg->x;    // X -> Y (ENU)
    vehicle_position_enu(2) = -msg->z;   // -Z -> Z (ENU)
    
    Eigen::Vector3d vehicle_velocity_enu;
    vehicle_velocity_enu(0) = msg->vy;   // VY -> VX (ENU)
    vehicle_velocity_enu(1) = msg->vx;   // VX -> VY (ENU)
    vehicle_velocity_enu(2) = -msg->vz;  // -VZ -> VZ (ENU)
    
    // Use ERROR-based state like Python implementation
    Eigen::Vector3d position_error = vehicle_position_enu - setpoint_position_;
    
    current_state_(0) = position_error(0);  // Position error X
    current_state_(1) = position_error(1);  // Position error Y
    current_state_(2) = position_error(2);  // Position error Z
    current_state_(3) = vehicle_velocity_enu(0);  // Velocity X
    current_state_(4) = vehicle_velocity_enu(1);  // Velocity Y
    current_state_(5) = vehicle_velocity_enu(2);  // Velocity Z
    
    // Log received position data
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Position NED: [%.3f, %.3f, %.3f] → ENU: [%.3f, %.3f, %.3f] → Error: [%.3f, %.3f, %.3f]",
                        msg->x, msg->y, msg->z, 
                        vehicle_position_enu(0), vehicle_position_enu(1), vehicle_position_enu(2),
                        current_state_(0), current_state_(1), current_state_(2));
    
    position_received_ = true;
}

void MPCNodePX4::attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    // NED->ENU transformation exactly like Python implementation
    // Python: q_enu = 1/np.sqrt(2) * np.array([msg.q[0] + msg.q[3], msg.q[1] + msg.q[2], msg.q[1] - msg.q[2], msg.q[0] - msg.q[3]])
    // Then normalized
    
    double sqrt2_inv = 1.0 / std::sqrt(2.0);
    Eigen::Vector4d q_enu;
    q_enu(0) = sqrt2_inv * (msg->q[0] + msg->q[3]);  // qw
    q_enu(1) = sqrt2_inv * (msg->q[1] + msg->q[2]);  // qx
    q_enu(2) = sqrt2_inv * (msg->q[1] - msg->q[2]);  // qy
    q_enu(3) = sqrt2_inv * (msg->q[0] - msg->q[3]);  // qz
    
    // Normalize quaternion
    q_enu.normalize();
    
    current_state_(6) = q_enu(0);  // qw
    current_state_(7) = q_enu(1);  // qx
    current_state_(8) = q_enu(2);  // qy
    current_state_(9) = q_enu(3);  // qz
    
    attitude_received_ = true;
}

void MPCNodePX4::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // For rate-based model, we don't need to store angular velocity in state
    // The angular velocity is part of the control inputs
    angular_velocity_received_ = true;
}

void MPCNodePX4::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    vehicle_status_ = *msg;
    
    // Log important status changes
    static uint8_t last_nav_state = 255;
    if (msg->nav_state != last_nav_state) {
        RCLCPP_INFO(this->get_logger(), "Navigation state changed to: %u", msg->nav_state);
        last_nav_state = msg->nav_state;
    }
}

void MPCNodePX4::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Goals come in ENU frame from takeoff script, store directly
    setpoint_position_(0) = msg->pose.position.x;    // X same
    setpoint_position_(1) = msg->pose.position.y;    // Y same (ENU input)
    setpoint_position_(2) = msg->pose.position.z;    // Z same (ENU input)
    
    // Error-based MPC: goal state is ZERO error (desired state)
    goal_state_ << 0.0, 0.0, 0.0,  // Zero position error (at goal)
                   0.0, 0.0, 0.0,  // Zero velocities
                   1.0, 0.0, 0.0, 0.0;  // Identity quaternion
    
    goal_received_ = true;
    goal_reception_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Setpoint received ENU: [%.2f,%.2f,%.2f] stored as: [%.2f,%.2f,%.2f]", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                setpoint_position_(0), setpoint_position_(1), setpoint_position_(2));
}

void MPCNodePX4::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) return;

    // Convert path waypoints from NED to ENU
    X_ref_.resize(msg->poses.size());
    for (size_t i = 0; i < msg->poses.size(); i++) {
        X_ref_[i] = Eigen::VectorXd::Zero(10);
        X_ref_[i] << msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, -msg->poses[i].pose.position.z,  // NED to ENU
                     0.0, 0.0, 0.0,  // Zero velocities
                     msg->poses[i].pose.orientation.w, msg->poses[i].pose.orientation.x, 
                     msg->poses[i].pose.orientation.y, msg->poses[i].pose.orientation.z;
    }

    // Set goal from path
    int path_size = msg->poses.size();
    int current_goal_idx = std::min(goal_index_, path_size - 1);
    
    goal_state_ = X_ref_[current_goal_idx];
    goal_received_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Path received with %zu waypoints, goal index: %d", 
                msg->poses.size(), current_goal_idx);
}

void MPCNodePX4::initializeCDDP() {
    int state_dim = 10;
    int control_dim = 4;

    std::string integration_type = "rk4";
    auto system = std::make_unique<cddp::QuadrotorRate>(
        timestep_, mass_, max_thrust_, max_rate_, integration_type
    );

    // Initialize cost matrices for rate-based model
    Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Q(0,0) = Q_px_; Q(1,1) = Q_py_; Q(2,2) = Q_pz_;
    Q(3,3) = Q_vx_; Q(4,4) = Q_vy_; Q(5,5) = Q_vz_;
    Q(6,6) = Q_qw_; Q(7,7) = Q_qx_; Q(8,8) = Q_qy_; Q(9,9) = Q_qz_;
    
    RCLCPP_INFO(this->get_logger(), "Q matrix: pos=[%.1f,%.1f,%.1f] vel=[%.1f,%.1f,%.1f]", 
                Q(0,0), Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5));

    R = Eigen::MatrixXd::Zero(control_dim, control_dim);
    R(0,0) = R_thrust_; R(1,1) = R_wx_; R(2,2) = R_wy_; R(3,3) = R_wz_;

    Qf = Eigen::MatrixXd::Zero(state_dim, state_dim);
    Qf(0,0) = Qf_px_; Qf(1,1) = Qf_py_; Qf(2,2) = Qf_pz_;
    Qf(3,3) = Qf_vx_; Qf(4,4) = Qf_vy_; Qf(5,5) = Qf_vz_;
    Qf(6,6) = Qf_qw_; Qf(7,7) = Qf_qx_; Qf(8,8) = Qf_qy_; Qf(9,9) = Qf_qz_;

    // Create initial reference trajectory for initialization
    // Start from current state (zeros) and end at goal state
    std::vector<Eigen::VectorXd> initial_reference_states;
    initial_reference_states.reserve(horizon_ + 1);
    
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(state_dim);
    initial_state(6) = 1.0;  // Set quaternion w to 1 for identity
    
    for (int i = 0; i <= horizon_; ++i) {
        double alpha = static_cast<double>(i) / static_cast<double>(horizon_);
        Eigen::VectorXd ref_state = (1.0 - alpha) * initial_state + alpha * goal_state_;
        // Ensure quaternion is normalized (simple interpolation for now)
        if (i < horizon_) {
            ref_state(6) = 1.0;  // Keep identity quaternion except for last state
            ref_state(7) = 0.0;
            ref_state(8) = 0.0;
            ref_state(9) = 0.0;
        }
        initial_reference_states.push_back(ref_state);
    }
    
    // Ensure the last reference state is exactly the goal state
    initial_reference_states.back() = goal_state_;
    
    auto objective = std::make_unique<cddp::QuadraticObjective>(
        Q, R, Qf,
        goal_state_,
        initial_reference_states,
        timestep_
    );

    cddp_solver_ = std::make_unique<cddp::CDDP>(
        Eigen::VectorXd::Zero(state_dim),
        Eigen::VectorXd::Zero(state_dim),
        horizon_,
        timestep_
    );

    cddp_solver_->setDynamicalSystem(std::move(system));
    cddp_solver_->setObjective(std::move(objective));

    // Control constraints for rate-based model
    Eigen::VectorXd lower_bound(control_dim);
    Eigen::VectorXd upper_bound(control_dim);
    // Thrust constraints
    lower_bound(0) = 0.0;
    upper_bound(0) = max_thrust_;
    // Angular rate constraints (conservative limits)
    for (int i = 1; i < control_dim; ++i) {
        lower_bound(i) = -max_rate_;  // ±2.0 rad/s max
        upper_bound(i) = max_rate_;
    }

    cddp_solver_->addPathConstraint(
        "ControlConstraint",
        std::make_unique<cddp::ControlConstraint>(upper_bound, lower_bound)
    );

    // Solver options (more conservative for stability)
    cddp::CDDPOptions options;
    options.max_iterations = 15;  // Reduced iterations for faster response
    options.tolerance = 1e-2;     // Relaxed tolerance
    options.acceptable_tolerance = 1e-2;
    options.enable_parallel = true;
    options.num_threads = 4;      // Reduced threads
    options.verbose = false;
    options.debug = false;
    options.regularization.initial_value = 1e-2;  // Higher regularization
    options.warm_start = enable_warm_start_;
    cddp_solver_->setOptions(options);
    
    RCLCPP_INFO(this->get_logger(), "Control bounds: thrust=[%.1f, %.1f]N, rates=[%.1f, %.1f]rad/s", 
                lower_bound(0), upper_bound(0), lower_bound(1), upper_bound(1));

    // Initial trajectory guess
    std::vector<Eigen::VectorXd> X(horizon_ + 1, Eigen::VectorXd::Zero(state_dim));
    std::vector<Eigen::VectorXd> U(horizon_, Eigen::VectorXd::Zero(control_dim));
    cddp_solver_->setInitialTrajectory(X, U);

    RCLCPP_INFO(this->get_logger(), "CDDP solver initialized");
}

void MPCNodePX4::controlLoop() {
    // Always publish offboard control mode to maintain offboard mode
    publishOffboardControlMode();
    
    // State machine for better control flow
    switch (current_control_state_) {
        case ControlState::INITIALIZING:
            if (cddp_solver_ == nullptr) {
                current_control_state_ = ControlState::WAITING_FOR_DATA;
                state_change_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "State: WAITING_FOR_DATA");
            }
            return;
            
        case ControlState::WAITING_FOR_DATA:
            if (!position_received_ || !attitude_received_ || !goal_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Waiting for data: pos=%d, att=%d, goal=%d",
                                    position_received_, attitude_received_, goal_received_);
                return;
            }
            current_control_state_ = ControlState::ARMING;
            state_change_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "State: ARMING");
            break;
            
        case ControlState::ARMING:
            if (!arm_requested_) {
                RCLCPP_INFO(this->get_logger(), "Requesting ARM");
                sendVehicleCommand(400, 1.0); // VEHICLE_CMD_COMPONENT_ARM_DISARM
                arm_requested_ = true;
            } else if ((this->now() - state_change_time_).seconds() > 3.0) {
                current_control_state_ = ControlState::OFFBOARD_REQUEST;
                state_change_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "State: OFFBOARD_REQUEST");
            }
            return;
            
        case ControlState::OFFBOARD_REQUEST:
            if (!offboard_mode_requested_) {
                RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
                sendVehicleCommand(176, 1.0, 6.0); // VEHICLE_CMD_DO_SET_MODE, offboard=6
                offboard_mode_requested_ = true;
            } else if ((this->now() - state_change_time_).seconds() > 2.0) {
                current_control_state_ = ControlState::MPC_CONTROL;
                state_change_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "State: MPC_CONTROL - Starting active control");
            }
            return;
            
        case ControlState::EMERGENCY_HOVER:
            // Send emergency hover commands
            publishEmergencyHover();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Emergency hover mode active");
            return;
            
        case ControlState::ERROR_STATE:
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Control in error state - manual intervention required");
            return;
            
        case ControlState::MPC_CONTROL:
            // Continue with MPC control below
            break;
    }

    // Run actual MPC control
    if (current_control_state_ == ControlState::MPC_CONTROL) {
        // Initialize CDDP solver if not already done
        if (!cddp_solver_) {
            initializeCDDP();
        }
        
        // Solve MPC and get control commands
        auto [u_opt, X_opt] = solveCDDPMPC();
        
        // Publish the optimal control commands
        publishRateCommands(u_opt);
        
        // Publish predicted trajectory for visualization
        publishLocalPath(X_opt);
    }
}

void MPCNodePX4::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    // Rate-based control mode
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    
    offboard_control_mode_pub_->publish(msg);
}

void MPCNodePX4::publishRateCommands(const Eigen::VectorXd& u) {
    // Rate-based control: u = [thrust, wx, wy, wz]
    // Match Python implementation thrust scaling
    
    // Python implementation:
    // thrust_rates = u_pred[0, :]
    // thrust_command = -(thrust_rates[0] * 0.07 + 0.0)
    // Hover thrust = 0.73 (from Python model)
    
    double hover_thrust_normalized = 0.73;  // From Python model
    
    // Apply thrust scaling like Python
    // Python uses: thrust_command = -(thrust_rates[0] * 0.07 + 0.0)
    double thrust_command = -(u(0) * 0.07 + 0.0);
    
    // Publish rates setpoint (combine thrust and rates in one message)
    px4_msgs::msg::VehicleRatesSetpoint rates_msg;
    rates_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    // Body rates in rad/s with proper sign conventions exactly like Python
    // Python: setpoint_msg.roll = float(thrust_rates[1])
    //         setpoint_msg.pitch = float(-thrust_rates[2])
    //         setpoint_msg.yaw = float(-thrust_rates[3])
    rates_msg.roll = float(u(1));       // Roll rate (wx) - same sign as Python
    rates_msg.pitch = float(-u(2));     // Pitch rate (wy) - negated like Python
    rates_msg.yaw = float(-u(3));       // Yaw rate (wz) - negated like Python
    
    // Set thrust in rates message exactly like Python
    rates_msg.thrust_body[0] = 0.0;
    rates_msg.thrust_body[1] = 0.0;
    rates_msg.thrust_body[2] = float(thrust_command);
    
    // Only publish rates setpoint (Python doesn't use separate thrust message)
    rates_setpoint_pub_->publish(rates_msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "MPC Commands: thrust_raw=%.4f, thrust_cmd=%.4f | Rates=[%.3f,%.3f,%.3f]rad/s",
                        u(0), thrust_command,
                        rates_msg.roll, rates_msg.pitch, rates_msg.yaw);
}



void MPCNodePX4::publishLocalPath(const std::vector<Eigen::VectorXd>& X_traj) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = "odom";
    
    for (const auto& x : X_traj) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->now();
        pose_stamped.header.frame_id = "odom";
        
        // Convert from ENU (MPC) to NED (visualization)
        // Rate-based model: [px, py, pz, vx, vy, vz, qw, qx, qy, qz]
        pose_stamped.pose.position.x = x(0);
        pose_stamped.pose.position.y = x(1);
        pose_stamped.pose.position.z = -x(2);  // ENU to NED
        
        pose_stamped.pose.orientation.w = x(6);
        pose_stamped.pose.orientation.x = x(7);
        pose_stamped.pose.orientation.y = x(8);
        pose_stamped.pose.orientation.z = x(9);
        
        path_msg.poses.push_back(pose_stamped);
    }
    
    local_path_publisher_->publish(path_msg);
}

void MPCNodePX4::sendVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    
    vehicle_command_pub_->publish(msg);
}

std::tuple<Eigen::VectorXd, std::vector<Eigen::VectorXd>> MPCNodePX4::solveCDDPMPC() {
    int state_dim = 10;
    int control_dim = 4;
    
    // Generate smooth reference trajectory using S-curve
    std::vector<Eigen::VectorXd> reference_trajectory;
    generateSmoothTrajectory(reference_trajectory);
    
    // Debug: Log first and last reference states
    if (!reference_trajectory.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Reference trajectory: start=[%.2f,%.2f,%.2f] end=[%.2f,%.2f,%.2f]",
                    reference_trajectory.front()(0), reference_trajectory.front()(1), reference_trajectory.front()(2),
                    reference_trajectory.back()(0), reference_trajectory.back()(1), reference_trajectory.back()(2));
        
        // Also check middle point
        size_t mid = reference_trajectory.size() / 2;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Reference mid-point: [%.2f,%.2f,%.2f]",
                    reference_trajectory[mid](0), reference_trajectory[mid](1), reference_trajectory[mid](2));
    }
    
    // Initialize solver once if not already created
    if (!cddp_solver_) {
        auto system = std::make_unique<cddp::QuadrotorRate>(
            timestep_, mass_, max_thrust_, max_rate_, "rk4"
        );
        
        auto objective = std::make_unique<cddp::QuadraticObjective>(
            Q, R, Qf,
            goal_state_,
            reference_trajectory,
            timestep_
        );
        
        cddp::CDDPOptions options;
        options.max_iterations = 5;   // Very few iterations for stability
        options.tolerance = 5e-2;     // Very relaxed tolerance
        options.acceptable_tolerance = 5e-2;
        options.enable_parallel = false; // Disable parallel for consistency
        options.num_threads = 1;      // Single thread
        options.verbose = false;
        options.debug = false;
        options.regularization.initial_value = 1e-1;  // Very high regularization
        options.warm_start = enable_warm_start_;
        
        cddp_solver_ = std::make_unique<cddp::CDDP>(current_state_, goal_state_, horizon_, timestep_,
                          std::move(system), std::move(objective), options);
        
        // Add control constraints for rate-based model
        Eigen::VectorXd lower_bound(4), upper_bound(4);
        lower_bound(0) = 0.0; upper_bound(0) = max_thrust_;  // Thrust
        for (int i = 1; i < 4; ++i) {
            lower_bound(i) = -max_rate_; upper_bound(i) = max_rate_;  // Angular rates
        }
        cddp_solver_->addPathConstraint("ControlConstraint",
            std::make_unique<cddp::ControlConstraint>(upper_bound, lower_bound));
        
        RCLCPP_INFO(this->get_logger(), "CDDP solver initialized once for reuse");
    } else {
        // Update states and objective for reused solver
        cddp_solver_->setInitialState(current_state_);
        cddp_solver_->setReferenceState(goal_state_);
        cddp_solver_->setReferenceStates(reference_trajectory);
    }
    
    // Set initial trajectory guess for the new solver
    std::vector<Eigen::VectorXd> X_guess(horizon_ + 1, Eigen::VectorXd::Zero(state_dim));
    std::vector<Eigen::VectorXd> U_guess(horizon_, Eigen::VectorXd::Zero(control_dim));
    
    if (enable_warm_start_ && has_previous_solution_) {
        // Validate previous solution quality before using as warm start
        double state_prediction_error = (previous_state_trajectory_[0] - current_state_).norm();
        
        if (state_prediction_error < 0.5) {  // Good prediction
            // Use previous solution as warm start (already shifted)
            X_guess = previous_state_trajectory_;
            U_guess = previous_control_trajectory_;
            
            // Update first state to current state
            X_guess[0] = current_state_;
            
            // Propagate correction through first few states
            for (int i = 1; i < std::min(3, horizon_); ++i) {
                X_guess[i] = X_guess[i] + (current_state_ - previous_state_trajectory_[0]) * (1.0 - i/3.0);
                // Ensure quaternion normalization
                Eigen::Vector4d quat = X_guess[i].segment(6, 4);
                quat.normalize();
                X_guess[i].segment(6, 4) = quat;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Using warm start with prediction error: %.3f", state_prediction_error);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Large prediction error %.3f, using cold start", state_prediction_error);
            has_previous_solution_ = false;  // Force cold start
        }
    }
    
    if (!enable_warm_start_ || !has_previous_solution_) {
        // Cold start - simple initial guess
        X_guess[0] = current_state_;
        
        // Set initial control guess for rate-based model
        double hover_thrust = mass_ * 9.81;  // Total hover thrust
        for (int i = 0; i < horizon_; i++) {
            U_guess[i] = Eigen::VectorXd::Zero(control_dim);
            U_guess[i](0) = hover_thrust;  // Thrust
            U_guess[i](1) = 0.0;  // Roll rate
            U_guess[i](2) = 0.0;  // Pitch rate
            U_guess[i](3) = 0.0;  // Yaw rate
            // Propagate state forward with simple dynamics
            X_guess[i+1] = X_guess[i];
            X_guess[i+1].segment(0,3) += timestep_ * X_guess[i].segment(3,3); // Update position
            // Ensure quaternion normalization
            X_guess[i+1](6) = 1.0;  // qw
            X_guess[i+1](7) = 0.0;  // qx
            X_guess[i+1](8) = 0.0;  // qy
            X_guess[i+1](9) = 0.0;  // qz
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Using cold start with hover thrust: %.2fN total", hover_thrust);
    }
    
    cddp_solver_->setInitialTrajectory(X_guess, U_guess);

    // Solve using the reused solver instance with error handling
    RCLCPP_DEBUG(this->get_logger(), "Starting CDDP solve with reused solver");
    auto start_time = this->now();
    
    cddp::CDDPSolution solution;
    try {
        solution = cddp_solver_->solve("MSIPDDP");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CDDP solver exception: %s", e.what());
        failed_solves_++;
        // Return emergency hover command
        Eigen::VectorXd emergency_control(4);
        emergency_control << mass_ * 9.81, 0.0, 0.0, 0.0;  // Hover thrust, zero rates
        std::vector<Eigen::VectorXd> emergency_trajectory(horizon_ + 1, current_state_);
        return std::make_tuple(emergency_control, emergency_trajectory);
    }
    
    auto end_time = this->now();
    double solve_time_ms = (end_time - start_time).seconds() * 1000.0;
    
    // Update performance metrics
    solve_count_++;
    max_solve_time_ms_ = std::max(max_solve_time_ms_, solve_time_ms);
    average_solve_time_ms_ = ((solve_count_ - 1) * average_solve_time_ms_ + solve_time_ms) / solve_count_;
    
    // Check for excessive solve time
    if (solve_time_ms > max_compute_time_ * 1000.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "CDDP solve took %.1f ms (limit: %.1f ms)", 
                           solve_time_ms, max_compute_time_ * 1000.0);
    }
    
    // Log performance statistics periodically
    if (solve_count_ % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Performance stats - Avg: %.1f ms, Max: %.1f ms, Failed: %d/%d (%.1f%%)",
                   average_solve_time_ms_, max_solve_time_ms_, failed_solves_, solve_count_,
                   (failed_solves_ * 100.0) / solve_count_);
    }
    
    // Log solver performance
    if (solution.find("iterations_completed") != solution.end()) {
        int iterations = std::any_cast<int>(solution.at("iterations_completed"));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "CDDP solve: %d iterations, %.1f ms", iterations, solve_time_ms);
    }  

    // Extract solution with error checking
    std::vector<Eigen::VectorXd> X_sol, U_sol;
    try {
        if (solution.find("state_trajectory") == solution.end() || 
            solution.find("control_trajectory") == solution.end()) {
            throw std::runtime_error("Missing trajectory data in solution");
        }
        
        X_sol = std::any_cast<std::vector<Eigen::VectorXd>>(solution.at("state_trajectory"));
        U_sol = std::any_cast<std::vector<Eigen::VectorXd>>(solution.at("control_trajectory"));
        
        if (X_sol.empty() || U_sol.empty()) {
            throw std::runtime_error("Empty trajectory solutions");
        }
        
        if (X_sol.size() != horizon_ + 1 || U_sol.size() != horizon_) {
            throw std::runtime_error("Incorrect trajectory dimensions");
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Solution extraction error: %s", e.what());
        failed_solves_++;
        // Return emergency hover command
        Eigen::VectorXd emergency_control(4);
        emergency_control << mass_ * 9.81, 0.0, 0.0, 0.0;
        std::vector<Eigen::VectorXd> emergency_trajectory(horizon_ + 1, current_state_);
        return std::make_tuple(emergency_control, emergency_trajectory);
    }
    
    // Debug: Print initial and final states of the solution trajectory
    if (!X_sol.empty()) {
        const auto& x0 = X_sol.front();
        const auto& xf = X_sol.back();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "MPC Trajectory: Initial=[%.2f,%.2f,%.2f] Final=[%.2f,%.2f,%.2f] Goal=[%.2f,%.2f,%.2f]",
                           x0(0), x0(1), x0(2),  // Initial position
                           xf(0), xf(1), xf(2),  // Final position in trajectory
                           goal_state_(0), goal_state_(1), goal_state_(2));  // Goal position
        
        // Also log velocities
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Velocities: Initial=[%.2f,%.2f,%.2f] Final=[%.2f,%.2f,%.2f]",
                           x0(3), x0(4), x0(5),  // Initial velocity
                           xf(3), xf(4), xf(5)); // Final velocity
    }
    
    // Debug: Check if solution converged and log solver info
    if (solution.find("converged") != solution.end()) {
        bool converged = std::any_cast<bool>(solution.at("converged"));
        if (!converged) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                               "CDDP did not converge!");
        }
    }
    
    // Force balance is now within acceptable range due to proper reference trajectory
    
    // Store solution for next warm start (shift by one time step)
    if (enable_warm_start_) {
        previous_state_trajectory_.resize(horizon_ + 1);
        previous_control_trajectory_.resize(horizon_);
        
        // Shift trajectories by one time step for next warm start
        for (int i = 0; i < horizon_ - 1; i++) {
            previous_state_trajectory_[i] = X_sol[i + 1];
            previous_control_trajectory_[i] = U_sol[i + 1];
        }
        
        // Extend final state/control using simple prediction
        Eigen::VectorXd final_state = X_sol[horizon_];
        Eigen::VectorXd final_control = U_sol[horizon_ - 1];
        
        // Simple prediction: maintain final velocity and control
        final_state.head(3) += timestep_ * final_state.segment(3, 3);  // Position prediction
        
        previous_state_trajectory_[horizon_ - 1] = final_state;
        previous_state_trajectory_[horizon_] = final_state;
        previous_control_trajectory_[horizon_ - 1] = final_control;
        
        has_previous_solution_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Stored shifted solution for next warm start");
    }
    
    // Return first control input and trajectory
    return std::make_tuple(U_sol[0], X_sol);
}

bool MPCNodePX4::validateParameters() {
    bool valid = true;
    
    // Physical parameter validation
    if (mass_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid mass: %.2f kg (must be > 0)", mass_);
        valid = false;
    }
    
    if (max_thrust_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid max_thrust: %.2f N (must be > 0)", max_thrust_);
        valid = false;
    }
    
    if (max_rate_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid max_rate: %.2f rad/s (must be > 0)", max_rate_);
        valid = false;
    }
    
    // Check if thrust is sufficient for hover
    double hover_thrust = mass_ * 9.81;
    if (max_thrust_ < hover_thrust) {
        RCLCPP_ERROR(this->get_logger(), "Max thrust %.2fN insufficient for hover (need %.2fN)", 
                     max_thrust_, hover_thrust);
        valid = false;
    }
    
    // MPC parameter validation
    if (timestep_ <= 0.0 || timestep_ > 0.2) {
        RCLCPP_ERROR(this->get_logger(), "Invalid timestep: %.3f s (must be in (0, 0.2])", timestep_);
        valid = false;
    }
    
    if (horizon_ <= 0 || horizon_ > 100) {
        RCLCPP_ERROR(this->get_logger(), "Invalid horizon: %d (must be in [1, 100])", horizon_);
        valid = false;
    }
    
    if (processing_frequency_ <= 0.0 || processing_frequency_ > 200.0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid frequency: %.1f Hz (must be in (0, 200])", processing_frequency_);
        valid = false;
    }
    
    // Cost parameter validation (should be non-negative)
    std::vector<std::pair<double, std::string>> cost_params = {
        {Q_px_, "Q_px"}, {Q_py_, "Q_py"}, {Q_pz_, "Q_pz"},
        {Q_vx_, "Q_vx"}, {Q_vy_, "Q_vy"}, {Q_vz_, "Q_vz"},
        {Q_qx_, "Q_qx"}, {Q_qy_, "Q_qy"}, {Q_qz_, "Q_qz"},
        {R_thrust_, "R_thrust"}, {R_wx_, "R_wx"}, {R_wy_, "R_wy"}, {R_wz_, "R_wz"},
        {Qf_px_, "Qf_px"}, {Qf_py_, "Qf_py"}, {Qf_pz_, "Qf_pz"},
        {Qf_vx_, "Qf_vx"}, {Qf_vy_, "Qf_vy"}, {Qf_vz_, "Qf_vz"}
    };
    
    for (const auto& [value, name] : cost_params) {
        if (value < 0.0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid cost parameter %s: %.3f (must be >= 0)", 
                         name.c_str(), value);
            valid = false;
        }
    }
    
    if (valid) {
        RCLCPP_INFO(this->get_logger(), "All parameters validated successfully");
        RCLCPP_INFO(this->get_logger(), "Hover thrust ratio: %.1f%% of max thrust", 
                   (hover_thrust / max_thrust_) * 100.0);
    }
    
    return valid;
}

void MPCNodePX4::generateSmoothTrajectory(std::vector<Eigen::VectorXd>& ref_trajectory) {
    ref_trajectory.clear();
    ref_trajectory.reserve(horizon_ + 1);
    
    // Reference trajectory always starts from current position and goes to goal
    
    // Generate reference trajectory from current position to goal
    Eigen::Vector3d current_position = current_state_.head(3);
    Eigen::Vector3d goal_position = goal_state_.head(3);
    
    for (int i = 0; i <= horizon_; ++i) {
        double alpha = static_cast<double>(i) / static_cast<double>(horizon_);
        
        // S-curve (smoothstep function) for smooth trajectory
        double smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha);
        
        Eigen::VectorXd ref_state = Eigen::VectorXd::Zero(10);
        // Interpolate position from current to goal over the MPC horizon
        ref_state.head(3) = current_position + smooth_alpha * (goal_position - current_position);
        
        // Compute velocity for smooth trajectory
        if (i < horizon_) {
            double dt = timestep_;
            double smooth_alpha_next = ((i+1.0)/horizon_) * ((i+1.0)/horizon_) * (3.0 - 2.0 * (i+1.0)/horizon_);
            ref_state.segment(3, 3) = (smooth_alpha_next - smooth_alpha) * (goal_position - current_position) / dt;
        }
        
        ref_state(6) = 1.0;  // Identity quaternion
        
        ref_trajectory.push_back(ref_state);
    }
    
    // Ensure the last reference state is exactly the goal state
    if (!ref_trajectory.empty()) {
        ref_trajectory.back() = goal_state_;
    }
}

void MPCNodePX4::publishEmergencyHover() {
    // Publish emergency hover thrust command
    px4_msgs::msg::VehicleThrustSetpoint emergency_thrust;
    emergency_thrust.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    emergency_thrust.xyz[0] = 0.0;   // No thrust in X
    emergency_thrust.xyz[1] = 0.0;   // No thrust in Y  
    emergency_thrust.xyz[2] = -0.5;  // 50% hover thrust in Z (NED down)
    
    // Publish emergency hover rates command (zero rates for stable hover)
    px4_msgs::msg::VehicleRatesSetpoint emergency_rates;
    emergency_rates.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    emergency_rates.roll = 0.0;      // Zero roll rate
    emergency_rates.pitch = 0.0;     // Zero pitch rate
    emergency_rates.yaw = 0.0;       // Zero yaw rate
    emergency_rates.thrust_body[0] = 0.0;  // No thrust in body X
    emergency_rates.thrust_body[1] = 0.0;  // No thrust in body Y
    emergency_rates.thrust_body[2] = -0.5; // 50% hover thrust in body Z
    
    // Publish offboard control mode to maintain offboard mode
    publishOffboardControlMode();
    
    // Publish the emergency commands
    thrust_setpoint_pub_->publish(emergency_thrust);
    rates_setpoint_pub_->publish(emergency_rates);
}

void MPCNodePX4::publishAttitudeStabilizationTest() {
    // Simple attitude stabilization test - just try to hover level
    
    // Calculate hover thrust needed to counter gravity
    double hover_thrust_newtons = mass_ * 9.81;  // Thrust needed to hover
    double hover_thrust_normalized = hover_thrust_newtons / max_thrust_;  // Normalize to [0,1]
    
    // Get current attitude from quaternion (current_state_[6:9] = [qw, qx, qy, qz])
    Eigen::Quaterniond current_quat(current_state_(6), current_state_(7), current_state_(8), current_state_(9));
    
    // Convert to Euler angles to get roll/pitch
    Eigen::Vector3d euler = current_quat.toRotationMatrix().eulerAngles(0, 1, 2);  // Roll, Pitch, Yaw
    double roll = euler(0);
    double pitch = euler(1);
    
    // Simple proportional control for attitude stabilization
    double kp_attitude = 1.0;  // Proportional gain for attitude
    double max_rate = 0.5;     // Maximum rate command (rad/s)
    
    // Desired attitude is level (roll=0, pitch=0)
    double roll_error = 0.0 - roll;
    double pitch_error = 0.0 - pitch;
    
    // Compute desired rates
    double desired_roll_rate = kp_attitude * roll_error;
    double desired_pitch_rate = kp_attitude * pitch_error;
    double desired_yaw_rate = 0.0;  // No yaw control for now
    
    // Clamp rates to safe limits
    desired_roll_rate = std::clamp(desired_roll_rate, -max_rate, max_rate);
    desired_pitch_rate = std::clamp(desired_pitch_rate, -max_rate, max_rate);
    
    // Publish thrust setpoint
    px4_msgs::msg::VehicleThrustSetpoint thrust_msg;
    thrust_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    thrust_msg.xyz[0] = 0.0;
    thrust_msg.xyz[1] = 0.0;
    thrust_msg.xyz[2] = -hover_thrust_normalized;  // Negative for NED down
    
    // Publish rates setpoint
    px4_msgs::msg::VehicleRatesSetpoint rates_msg;
    rates_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    rates_msg.roll = desired_roll_rate;
    rates_msg.pitch = desired_pitch_rate;
    rates_msg.yaw = desired_yaw_rate;
    rates_msg.thrust_body[0] = 0.0;
    rates_msg.thrust_body[1] = 0.0;
    rates_msg.thrust_body[2] = -hover_thrust_normalized;
    
    // Publish offboard control mode
    publishOffboardControlMode();
    
    // Publish commands
    thrust_setpoint_pub_->publish(thrust_msg);
    rates_setpoint_pub_->publish(rates_msg);
    
    // Log attitude stabilization info
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Attitude Test: Roll=%.2f°→%.2f°, Pitch=%.2f°→%.2f°, Thrust=%.1f%%, Rates=[%.2f,%.2f,%.2f]",
                        roll*180/M_PI, 0.0, pitch*180/M_PI, 0.0, 
                        hover_thrust_normalized*100, desired_roll_rate, desired_pitch_rate, desired_yaw_rate);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCNodePX4>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}