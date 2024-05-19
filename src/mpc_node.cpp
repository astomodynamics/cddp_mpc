#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <eigen3/Eigen/Dense>
// #include "tf2/LinearMath/Quaternion.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

// #include "cddp_core/CDDPProblem.hpp"
// #include "cddp_core/Constraint.hpp"
#include "CDDP.hpp"

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node") {

        // Subscribe goal pose
        goal_subscription_ =  create_subscription<geometry_msgs::msg::Pose>(
            "/goal_pose", 10, std::bind(&MPCNode::goalCallback, this, std::placeholders::_1));

        // Subscribe current pose
        pose_subscription_ = create_subscription<geometry_msgs::msg::Pose>(
            "/robot_pose", 10, std::bind(&MPCNode::poseCallback, this, std::placeholders::_1));

        // Subscribe path
        path_subscription_ = create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&MPCNode::pathCallback, this, std::placeholders::_1));

        // Publish control command
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Control loop
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MPCNode::controlLoop, this));  
    }

private:
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        goal_pose_ = *msg;
        geometry_msgs::msg::Quaternion q = goal_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        goal_state_.resize(3);
        goal_state_ << goal_pose_.position.x, goal_pose_.position.y, euler(2);

        goal_state_ << 2.0, 2.0, M_PI/4.0;
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        initial_pose_ = *msg;
        geometry_msgs::msg::Quaternion q = initial_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        initial_state_.resize(3);
        initial_state_ << initial_pose_.position.x, initial_pose_.position.y, euler(2);

        initial_state_ << 0.0, 0.0, M_PI/4; // Initial state
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        if (msg->poses.size() == 0){
            return;
        }
        // Set initial state to the first pose in the path
        initial_state_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 0.0;
        // Set goal state to the last pose in the path
        goal_state_ << msg->poses[msg->poses.size()-1].pose.position.x, msg->poses[msg->poses.size()-1].pose.position.y, 0.0;
    }

    void controlLoop(){
        // If initial and goal states are not set, return
        if (initial_state_.size() == 0 || goal_state_.size() == 0){
            return;
        }
        Eigen::VectorXd u = solveCDDPMPC();

        // Publish control command
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = u(0);
        twist_msg.angular.z = u(1);
        cmd_vel_publisher_->publish(twist_msg);
    }

    Eigen::VectorXd solveCDDPMPC(){
        int state_dim = 3; 
        int control_dim = 2; 
        double dt = 0.03;
        int horizon = 100;
        int integration_type = 0; // 0 for Euler, 1 for Heun, 2 for RK3, 3 for RK4

        // Problem Setup
        RCLCPP_INFO(this->get_logger(), "Setting up CDDP MPC");

        cddp::DubinsCar system(state_dim, control_dim, dt, integration_type); // Your DoubleIntegrator instance
        cddp::CDDPProblem cddp_solver(initial_state_, goal_state_, horizon, dt);
        cddp_solver.setDynamicalSystem(std::make_unique<cddp::DubinsCar>(system));

RCLCPP_INFO(this->get_logger(), "initial state values %f %f %f", initial_state_(0), initial_state_(1), initial_state_(2));
RCLCPP_INFO(this->get_logger(), "goal state values %f %f %f", goal_state_(0), goal_state_(1), goal_state_(2));

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

        cddp::QuadraticCost objective(Q, R, Qf, goal_state_, dt);
        cddp_solver.setObjective(std::make_unique<cddp::QuadraticCost>(objective));

        // Add constraints 
        Eigen::VectorXd lower_bound(control_dim);
        lower_bound << -5.0, -M_PI;

        Eigen::VectorXd upper_bound(control_dim);
        upper_bound << 5.0, M_PI;

        cddp::ControlBoxConstraint control_constraint(lower_bound, upper_bound);
        cddp_solver.addConstraint(std::make_unique<cddp::ControlBoxConstraint>(control_constraint));

        cddp::CDDPOptions opts;
        opts.max_iterations = 20;
        // opts.cost_tolerance = 1e-6;
        // opts.grad_tolerance = 1e-8;
        // opts.print_iterations = false;

        cddp_solver.setOptions(opts);

        // // Set initial trajectory if needed
        std::vector<Eigen::VectorXd> X = std::vector<Eigen::VectorXd>(horizon + 1, initial_state_);
        std::vector<Eigen::VectorXd> U = std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(control_dim));
        cddp_solver.setInitialTrajectory(X, U);

        // Solve!
        std::vector<Eigen::VectorXd> U_sol = cddp_solver.solve();

        std::vector<Eigen::VectorXd> X_sol = cddp_solver.getTrajectory();

        // Extract control
        Eigen::VectorXd u = U_sol[0];

RCLCPP_INFO(this->get_logger(), "Final state %f %f", X_sol[horizon](0), X_sol[horizon](1));

        return u;
    }
    

    Eigen::VectorXd getEulerFromQuaternion(const Eigen::Quaterniond& quat){
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        // Wrap yaw between -pi and pi (ONLY INTERESTED IN YAW ANGLE)
        if (euler(2) > M_PI){
            euler(2) -= 2*M_PI;
        } else if (euler(2) < -M_PI){
            euler(2) += 2*M_PI;
        }
        return euler;
    }

    geometry_msgs::msg::Pose goal_pose_;
    geometry_msgs::msg::Pose initial_pose_;
    Eigen::VectorXd goal_state_;
    Eigen::VectorXd initial_state_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
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

// TEST COMMANDS
/*
ros2 run cddp_mpc mpc_node

ros2 topic pub /goal_pose geometry_msgs/msg/Pose "{position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" 

ros2 topic pub /robot_pose geometry_msgs/msg/Pose "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" 
*/