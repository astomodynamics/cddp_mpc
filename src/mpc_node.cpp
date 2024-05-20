#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <eigen3/Eigen/Dense>
#include "CDDP.hpp"

class MPCNode : public rclcpp::Node {
public:
    MPCNode() : Node("mpc_node") {
        // Declare parameters
        this->declare_parameter("timestep_", 0.05); // NOTE: This is a hyper-parameter and needs to be tuned
        this->declare_parameter("horizon_", 50); // NOTE: This is a hyper-parameter and needs to be tuned
        this->declare_parameter("processing_frequency_", 10.0); 
        this->declare_parameter("goal_index_", 7); // Taking 7th element in the path as goal pose; NOTE: This is a hyper-parameter and needs to be tuned

        // Get parameters
        this->get_parameter("timestep_", timestep_);
        this->get_parameter("horizon_", horizon_);
        this->get_parameter("processing_frequency_", processing_frequency_);
        this->get_parameter("goal_index_", goal_index_);

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

        // Publish local path
        local_path_publisher_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);

        // Control loop
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0/processing_frequency_)), std::bind(&MPCNode::controlLoop, this));
    }

private:
    void goalCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        goal_pose_ = *msg;
        geometry_msgs::msg::Quaternion q = goal_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        goal_state_.resize(3);
        goal_state_ << goal_pose_.position.x, goal_pose_.position.y, euler(2);
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        initial_pose_ = *msg;
        geometry_msgs::msg::Quaternion q = initial_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        initial_state_.resize(3);
        initial_state_ << initial_pose_.position.x, initial_pose_.position.y, euler(2);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        if (msg->poses.size() == 0){
            return;
        }
        // Set initial state to the first pose in the path
        geometry_msgs::msg::Quaternion q = msg->poses[0].pose.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        initial_state_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, euler(2);

        // Set goal state to the last pose in the path
        int path_size = msg->poses.size();
        if (path_size < goal_index_){
            goal_index_ = path_size - 1;
        }
        q = msg->poses[goal_index_].pose.orientation;
        quat = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        euler = getEulerFromQuaternion(quat);
        goal_state_ << msg->poses[goal_index_].pose.position.x, msg->poses[goal_index_].pose.position.y, euler(2);

        // Set reference path
        X_ref_.resize(msg->poses.size());
        for (int i = 0; i < msg->poses.size(); i++){
            geometry_msgs::msg::Quaternion q = msg->poses[i].pose.orientation;
            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
            Eigen::Vector3d euler = getEulerFromQuaternion(quat);
            X_ref_[i].resize(3);
            X_ref_[i] << msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, euler(2);
        }
    }

    void controlLoop(){
        // If initial and goal states are not set, return
        if (initial_state_.size() == 0 || goal_state_.size() == 0){
            return;
        }
        auto [u, X] = solveCDDPMPC();

        // Publish control command
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = u(0);
        twist_msg.angular.z = u(1);
        cmd_vel_publisher_->publish(twist_msg);

        // Publish local path
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = "map";
        for (int i = 0; i < X.size(); i++){
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = X[i](0);
            pose_stamped.pose.position.y = X[i](1);
            pose_stamped.pose.position.z = 0.0;
            
            Eigen::VectorXd euler(3);
            euler << 0.0, 0.0, X[i](2);
            Eigen::Quaterniond quat = getQuaternionFromEuler(euler);
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            pose_stamped.pose.orientation.w = quat.w();
            path_msg.poses.push_back(pose_stamped);
        }

    }

    // CDDP MPC Solver which returns control input and path
    std::tuple<Eigen::VectorXd, std::vector<Eigen::VectorXd>> solveCDDPMPC(){
        // CDDP MPC Solver
        int state_dim = 3; 
        int control_dim = 2; 
        int integration_type = 0; // 0 for Euler, 1 for Heun, 2 for RK3, 3 for RK4

        // Problem Setup
        RCLCPP_INFO(this->get_logger(), "Setting up CDDP MPC");

        cddp::DubinsCar system(state_dim, control_dim, timestep_, integration_type); // Your DoubleIntegrator instance
        cddp::CDDPProblem cddp_solver(initial_state_, goal_state_, horizon_, timestep_);
        cddp_solver.setDynamicalSystem(std::make_unique<cddp::DubinsCar>(system));

RCLCPP_INFO(this->get_logger(), "initial state values %f %f %f", initial_state_(0), initial_state_(1), initial_state_(2));
RCLCPP_INFO(this->get_logger(), "goal state values %f %f %f", goal_state_(0), goal_state_(1), goal_state_(2));

        // Simple Cost Matrices; NOTE: These are hyper-parameters and need to be tuned
        Eigen::MatrixXd Q(state_dim, state_dim);
        Q << 1e-1, 0, 0, 
            0, 1e-1, 0, 
            0, 0, 0e-3;
        Eigen::MatrixXd R(control_dim, control_dim);
        R << 1e-1, 0, 
            0, 1e-1; 
        Eigen::MatrixXd Qf(state_dim, state_dim);
        Qf << 100, 0, 0, 
            0, 100, 0, 
            0, 0, 0; 

        cddp::QuadraticCost objective(Q, R, Qf, goal_state_, timestep_);
        cddp_solver.setObjective(std::make_unique<cddp::QuadraticCost>(objective));

        // Add constraints 
        Eigen::VectorXd lower_bound(control_dim);
        lower_bound << -0.4, -2*M_PI;

        Eigen::VectorXd upper_bound(control_dim);
        upper_bound << 0.4, 2*M_PI;

        cddp::ControlBoxConstraint control_constraint(lower_bound, upper_bound);
        cddp_solver.addConstraint(std::make_unique<cddp::ControlBoxConstraint>(control_constraint));

        // Set options
        cddp::CDDPOptions opts;
        opts.max_iterations = 10;
        // opts.cost_tolerance = 1e-6;
        // opts.grad_tolerance = 1e-8;
        // opts.print_iterations = false;

        cddp_solver.setOptions(opts);

        // Set initial trajectory 
        std::vector<Eigen::VectorXd> X = std::vector<Eigen::VectorXd>(horizon_ + 1, initial_state_);
        std::vector<Eigen::VectorXd> U = std::vector<Eigen::VectorXd>(horizon_, Eigen::VectorXd::Zero(control_dim));
        cddp_solver.setInitialTrajectory(X, U);

        // Solve!
        std::vector<Eigen::VectorXd> U_sol = cddp_solver.solve();

        std::vector<Eigen::VectorXd> X_sol = cddp_solver.getTrajectory();

        // Extract control
        Eigen::VectorXd u = U_sol[0];

RCLCPP_INFO(this->get_logger(), "Final state %f %f", X_sol[horizon_](0), X_sol[horizon_](1));

        return std::make_tuple(u, X_sol);
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

    Eigen::Quaterniond getQuaternionFromEuler(const Eigen::Vector3d& euler){
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;
        return quat;
    }
    
    double timestep_;
    int horizon_;
    double processing_frequency_;
    int goal_index_;

    geometry_msgs::msg::Pose goal_pose_;
    geometry_msgs::msg::Pose initial_pose_;
    Eigen::VectorXd goal_state_;
    Eigen::VectorXd initial_state_;
    std::vector<Eigen::VectorXd> X_ref_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
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