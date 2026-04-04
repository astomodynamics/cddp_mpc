/**
 * @file mpc_node.hpp
 * @brief Model Predictive Control node for PX4 quadrotor
 */

#ifndef MPC_NODE_HPP
#define MPC_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <Eigen/Dense>
#include <memory>
#include <vector>

// Forward declarations
namespace cddp {
    class CDDP;
}

enum class ControlState {
    INITIALIZING,
    WAITING_FOR_DATA,
    ARMING,
    OFFBOARD_REQUEST,
    MPC_CONTROL,
    EMERGENCY_HOVER,
    ERROR_STATE
};

class MPCNodePX4 : public rclcpp::Node {
public:
    MPCNodePX4();
    ~MPCNodePX4() = default;

private:
    // Callbacks
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    // Control functions
    void controlLoop();
    void initializeCDDP();
    std::tuple<Eigen::VectorXd, std::vector<Eigen::VectorXd>> solveCDDPMPC();
    void generateSmoothTrajectory(std::vector<Eigen::VectorXd>& ref_trajectory);
    
    // Utility functions
    bool validateParameters();

    // Publishing functions
    void publishOffboardControlMode();
    void publishRateCommands(const Eigen::VectorXd& u);
    void publishLocalPath(const std::vector<Eigen::VectorXd>& X_traj);
    void publishEmergencyHover();
    void publishAttitudeStabilizationTest();
    void sendVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // CDDP solver
    std::unique_ptr<cddp::CDDP> cddp_solver_;

    // State and control dimensions (Rate-based model)
    static constexpr int STATE_DIM = 10;  // [px, py, pz, vx, vy, vz, qw, qx, qy, qz]
    static constexpr int CONTROL_DIM = 4;  // [thrust, wx, wy, wz]

    // Current state and goal
    Eigen::VectorXd current_state_;
    Eigen::VectorXd goal_state_;
    std::vector<Eigen::VectorXd> X_ref_;
    Eigen::Vector3d goal_position_enu_;  // Goal position in ENU frame for error-based control
    Eigen::Vector3d setpoint_position_;  // Current setpoint position for error calculation

    // Vehicle status
    px4_msgs::msg::VehicleStatus vehicle_status_;

    // Status flags
    bool position_received_{false};
    bool attitude_received_{false};
    bool angular_velocity_received_{false};
    bool goal_received_{false};
    bool offboard_mode_requested_{false};
    bool arm_requested_{false};
    
    // State machine
    ControlState current_control_state_{ControlState::INITIALIZING};
    rclcpp::Time state_change_time_;

    // Parameters
    std::string robot_id_;
    double timestep_;
    int horizon_;
    double processing_frequency_;
    int goal_index_;
    double max_compute_time_;
    
    // Quadrotor physical parameters (Rate-based model)
    double mass_;
    double max_thrust_;    // Maximum total thrust
    double max_rate_;      // Maximum angular rate
    
    // Cost parameters (Rate-based model)
    double Q_px_, Q_py_, Q_pz_;
    double Q_vx_, Q_vy_, Q_vz_;
    double Q_qw_, Q_qx_, Q_qy_, Q_qz_;
    double R_thrust_, R_wx_, R_wy_, R_wz_;
    double Qf_px_, Qf_py_, Qf_pz_;
    double Qf_vx_, Qf_vy_, Qf_vz_;
    double Qf_qw_, Qf_qx_, Qf_qy_, Qf_qz_;

    // Cost matrices
    Eigen::MatrixXd Q, R, Qf;

    // Rate-based control (no mode switching needed)
    bool enable_warm_start_;
    rclcpp::Time goal_reception_time_;

    // Warm start storage
    bool has_previous_solution_{false};
    std::vector<Eigen::VectorXd> previous_state_trajectory_;
    std::vector<Eigen::VectorXd> previous_control_trajectory_;
    
    // Performance monitoring
    double average_solve_time_ms_{0.0};
    int solve_count_{0};
    int failed_solves_{0};
    double max_solve_time_ms_{0.0};
};

#endif // MPC_NODE_HPP