#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
// #include "tf2/LinearMath/Quaternion.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

// #include "cddp_core/CDDPProblem.hpp"
// #include "cddp_core/Constraint.hpp"
#include "CDDP.hpp"

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node") {
        goal_subscriber_ =  create_subscription<geometry_msgs::msg::Pose>(
            "/goal_pose", 10, std::bind(&MPCNode::goalCallback, this, std::placeholders::_1));

        // Subscribe current pose
        pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
            "/robot_pose", 10, std::bind(&MPCNode::poseCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MPCNode::controlLoop, this));  
    }

private:
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr goal){
        geometry_msgs::msg::Pose goal_pose = *goal;

    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr pose){
        geometry_msgs::msg::Pose current_pose = *pose;
    }

    void setupCDDPMPC(){
        int state_dim = 3; 
        int control_dim = 2; 
        double dt = 0.05;
        int horizon = 100;
        int integration_type = 0; // 0 for Euler, 1 for Heun, 2 for RK3, 3 for RK4

        // Problem Setup
        Eigen::VectorXd initial_state(state_dim);
        initial_state << 0.0, 0.0, 0.0; // Initial state

        Eigen::VectorXd goal_state(state_dim);
        goal_state << 2.0, 2.0, M_PI/2.0;

        RCLCPP_INFO(this->get_logger(), "Setting up CDDP MPC");

        cddp::DubinsCar system(state_dim, control_dim, dt, integration_type); // Your DoubleIntegrator instance
        cddp::CDDPProblem cddp_solver(initial_state, goal_state, horizon, dt);
        cddp_solver.setDynamicalSystem(std::make_unique<cddp::DubinsCar>(system));

RCLCPP_INFO(this->get_logger(), "goal state values %f %f %f", goal_state(0), goal_state(1), goal_state(2));

        // Simple Cost Matrices 
        Eigen::MatrixXd Q(state_dim, state_dim);
        Q << 0e-2, 0, 0, 
            0, 0e-2, 0, 
            0, 0, 0e-3;
        Eigen::MatrixXd R(control_dim, control_dim);
        R << 1e+0, 0, 
            0, 1e+0; 
        Eigen::MatrixXd Qf(state_dim, state_dim);
        Qf << 50, 0, 0, 
            0, 50, 0, 
            0, 0, 10; 

        cddp::QuadraticCost objective(Q, R, Qf, goal_state, dt);
        cddp_solver.setObjective(std::make_unique<cddp::QuadraticCost>(objective));

        // Add constraints 
        Eigen::VectorXd lower_bound(control_dim);
        lower_bound << -0.1, -M_PI;

        Eigen::VectorXd upper_bound(control_dim);
        upper_bound << 0.1, M_PI;

        cddp::ControlBoxConstraint control_constraint(lower_bound, upper_bound);
        cddp_solver.addConstraint(std::make_unique<cddp::ControlBoxConstraint>(control_constraint));

        cddp::CDDPOptions opts;
        opts.max_iterations = 20;
        opts.cost_tolerance = 1e-6;
        opts.grad_tolerance = 1e-8;
        opts.print_iterations = false;

        cddp_solver.setOptions(opts);

        // // Set initial trajectory if needed
        // std::vector<Eigen::VectorXd> X = std::vector<Eigen::VectorXd>(horizon + 1, Eigen::VectorXd::Zero(state_dim));
        // std::vector<Eigen::VectorXd> U = std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(control_dim));
        // // X.front() = initialState;
        // cddp_solver.setInitialTrajectory(X, U);

        // // Solve!
        // cddp_solver.solve();

        // std::vector<Eigen::VectorXd> U_sol = cddp_solver.getControlTrajectory();
        // std::vector<Eigen::VectorXd> X_sol = cddp_solver.getTrajectory();

        // // Print solution
        // for (int i = 0; i < horizon; i++){
        //     RCLCPP_INFO(this->get_logger(), "Control %d: %f %f", i, U_sol[i](0), U_sol[i](1));
        // }


    }
    void controlLoop(){

        // Publish control command
        auto twist_msg = geometry_msgs::msg::Twist();
        // twist_msg.linear.x = u(0);
        // twist_msg.angular.z = u(1);
        setupCDDPMPC();

        cmd_vel_publisher_->publish(twist_msg);
    }


    Eigen::VectorXd goal_state_;
    Eigen::VectorXd initial_state_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}