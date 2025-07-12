#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <eigen3/Eigen/Dense>
#include "cddp.hpp"
#include "dynamics_model/car.hpp"

class CarMPCNode : public rclcpp::Node {
public:
    CarMPCNode() : Node("car_mpc_node") {
        // Declare parameters
        this->declare_parameter<std::string>("robot_id", "robot_1");
        this->declare_parameter<double>("timestep_", 0.1);
        this->declare_parameter<int>("horizon_", 40);
        this->declare_parameter<double>("processing_frequency_", 10.0);
        this->declare_parameter<int>("goal_index_", 7);
        this->declare_parameter<double>("max_compute_time_", 0.1);
        this->declare_parameter<double>("wheelbase_", 1.6);
        this->declare_parameter<double>("max_steering_angle_", 0.5);
        
        // Control constraints
        this->declare_parameter<double>("delta_max_", 0.5);
        this->declare_parameter<double>("delta_min_", -0.5);
        this->declare_parameter<double>("a_max_", 1.5);
        this->declare_parameter<double>("a_min_", -1.5);
        
        // Cost parameters
        this->declare_parameter<double>("Q_x_", 1e-1);
        this->declare_parameter<double>("Q_y_", 1e-1);
        this->declare_parameter<double>("Q_theta_", 0e-3);
        this->declare_parameter<double>("Q_v_", 1e-2);
        this->declare_parameter<double>("R_delta_", 1e-2);
        this->declare_parameter<double>("R_a_", 1e-2);
        this->declare_parameter<double>("Qf_x_", 0e-1);
        this->declare_parameter<double>("Qf_y_", 0e-1);
        this->declare_parameter<double>("Qf_theta_", 0.0);
        this->declare_parameter<double>("Qf_v_", 0.0);

        // Get parameters
        robot_id_ = this->get_parameter("robot_id").as_string();
        this->get_parameter("timestep_", timestep_);
        this->get_parameter("horizon_", horizon_);
        this->get_parameter("processing_frequency_", processing_frequency_);
        this->get_parameter("goal_index_", goal_index_);
        this->get_parameter("max_compute_time_", max_compute_time_);
        this->get_parameter("wheelbase_", wheelbase_);
        this->get_parameter("max_steering_angle_", max_steering_angle_);
        this->get_parameter("delta_max_", delta_max_);
        this->get_parameter("delta_min_", delta_min_);
        this->get_parameter("a_max_", a_max_);
        this->get_parameter("a_min_", a_min_);
        this->get_parameter("Q_x_", Q_x_);
        this->get_parameter("Q_y_", Q_y_);
        this->get_parameter("Q_theta_", Q_theta_);
        this->get_parameter("Q_v_", Q_v_);
        this->get_parameter("R_delta_", R_delta_);
        this->get_parameter("R_a_", R_a_);
        this->get_parameter("Qf_x_", Qf_x_);
        this->get_parameter("Qf_y_", Qf_y_);
        this->get_parameter("Qf_theta_", Qf_theta_);
        this->get_parameter("Qf_v_", Qf_v_);

        RCLCPP_INFO(this->get_logger(), "Car MPC is initialized");

        // Subscribe goal pose
        goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&CarMPCNode::goalCallback, this, std::placeholders::_1));

        // Subscribe current pose
        pose_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            robot_id_ + "/pose", 10, std::bind(&CarMPCNode::poseCallback, this, std::placeholders::_1));

        // Subscribe path
        path_subscription_ = create_subscription<nav_msgs::msg::Path>(
            robot_id_ + "/global_path", 10, std::bind(&CarMPCNode::pathCallback, this, std::placeholders::_1));

        // Publish control command
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(robot_id_ + "/cmd_vel", 10);
        
        // Publish local path
        local_path_publisher_ = create_publisher<nav_msgs::msg::Path>(robot_id_ + "/local_path", 10);

        // Control loop
        control_timer_ = create_wall_timer(
            std::chrono::milliseconds((int)(1000.0/processing_frequency_)), std::bind(&CarMPCNode::controlLoop, this));
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        goal_pose_ = msg->pose;
        geometry_msgs::msg::Quaternion q = goal_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        goal_state_.resize(4);
        goal_state_ << goal_pose_.position.x, goal_pose_.position.y, euler(2), 0.0;
        RCLCPP_INFO(this->get_logger(), "goal state = [%f, %f, %f, %f]", 
                    goal_state_(0), goal_state_(1), goal_state_(2), goal_state_(3));
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        initial_pose_ = msg->pose;
        geometry_msgs::msg::Quaternion q = initial_pose_.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        initial_state_.resize(4);
        initial_state_ << initial_pose_.position.x, initial_pose_.position.y, euler(2), 0.0;
        RCLCPP_INFO(this->get_logger(), "initial state = [%f, %f, %f, %f]", 
                    initial_state_(0), initial_state_(1), initial_state_(2), initial_state_(3));
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg){
        if (msg->poses.size() == 0){
            return;
        }
        // Set initial state to the first pose in the path
        geometry_msgs::msg::Quaternion q = msg->poses[0].pose.orientation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = getEulerFromQuaternion(quat);
        initial_state_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, euler(2), 0.0;

        // Set goal state to the last pose in the path
        int path_size = msg->poses.size();
        if (path_size < goal_index_){
            goal_index_ = path_size - 1;
        }
        q = msg->poses[goal_index_].pose.orientation;
        quat = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        euler = getEulerFromQuaternion(quat);
        goal_state_ << msg->poses[goal_index_].pose.position.x, msg->poses[goal_index_].pose.position.y, euler(2), 0.0;

        // Set reference path
        X_ref_.resize(msg->poses.size());
        for (int i = 0; i < msg->poses.size(); i++){
            geometry_msgs::msg::Quaternion q = msg->poses[i].pose.orientation;
            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
            Eigen::Vector3d euler = getEulerFromQuaternion(quat);
            X_ref_[i].resize(4);
            X_ref_[i] << msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, euler(2), 0.0;
        }
    }

    void initializeCDDP() {
        // Construct the CDDP solver
        int state_dim   = 4;  // (x, y, theta, v)
        int control_dim = 2;  // (steering_angle, acceleration)

        std::string integration_type = "euler";
        auto system = std::make_unique<cddp::Car>(timestep_, wheelbase_, integration_type);

        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(state_dim, state_dim);
        Q(0,0) = Q_x_;
        Q(1,1) = Q_y_;
        Q(2,2) = Q_theta_;
        Q(3,3) = Q_v_;

        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(control_dim, control_dim);
        R(0,0) = R_delta_;
        R(1,1) = R_a_;

        Eigen::MatrixXd Qf = Eigen::MatrixXd::Zero(state_dim, state_dim);
        Qf(0,0) = Qf_x_;
        Qf(1,1) = Qf_y_;
        Qf(2,2) = Qf_theta_;
        Qf(3,3) = Qf_v_;

        std::vector<Eigen::VectorXd> empty_reference_states;
        auto objective = std::make_unique<cddp::QuadraticObjective>(
            Q, R, Qf,
            Eigen::VectorXd::Zero(4),
            empty_reference_states,
            timestep_
        );

        cddp_solver_ = std::make_unique<cddp::CDDP>(
            Eigen::VectorXd::Zero(4),  
            Eigen::VectorXd::Zero(4),  
            horizon_,
            timestep_
        );

        // Assign system and objective
        cddp_solver_->setDynamicalSystem(std::move(system));
        cddp_solver_->setObjective(std::move(objective));

        // Add control constraints 
        Eigen::VectorXd lower_bound(control_dim);
        lower_bound << delta_min_, a_min_;
        Eigen::VectorXd upper_bound(control_dim);
        upper_bound << delta_max_, a_max_;

        cddp_solver_->addPathConstraint(
            "ControlConstraint",
            std::make_unique<cddp::ControlConstraint>(upper_bound, lower_bound)
        );

        // Add state constraints on velocity
        Eigen::VectorXd state_upper_bound(1);
        state_upper_bound << 2.0;
        cddp_solver_->addPathConstraint("StateConstraint",
                                       std::make_unique<cddp::StateConstraint>(state_upper_bound));

        // Set some solver options
        cddp::CDDPOptions options;
        options.max_iterations = 50;
        options.tolerance = 1e-4;
        options.acceptable_tolerance = 1e-3;
        options.enable_parallel = true;
        options.num_threads = 10;
        options.verbose = true;
        options.debug = true;
        options.regularization.initial_value = 1e-2;
        cddp_solver_->setOptions(options);

        // Provide an initial trajectory guess (X, U)
        std::vector<Eigen::VectorXd> X(horizon_ + 1, Eigen::VectorXd::Zero(state_dim));
        std::vector<Eigen::VectorXd> U(horizon_,     Eigen::VectorXd::Zero(control_dim));
        cddp_solver_->setInitialTrajectory(X, U);

        RCLCPP_INFO(this->get_logger(), "CDDP solver has been constructed");
    }

    void controlLoop(){
        // If initial and goal states are not set, return
        if (initial_state_.size() == 0 || goal_state_.size() == 0){
            return;
        }

        // Initialize CDDP solver
        if (cddp_solver_ == nullptr){
            initializeCDDP();
        }

        // Solve CDDP MPC
        auto [u, X] = solveCDDPMPC();

        // Publish control command
        // Apply control limits for safety
        u(0) = std::clamp(u(0), delta_min_, delta_max_);
        u(1) = std::clamp(u(1), a_min_, a_max_);
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = u(1);  // acceleration (Car control order: [steering, acceleration])
        twist_msg.angular.z = u(0); // steering angle
        cmd_vel_publisher_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Control command = [%f, %f]", u(0), u(1));

        // Publish local path
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
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
        local_path_publisher_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(), "Local path has been published");
    }

    // CDDP MPC Solver which returns control input and path
    std::tuple<Eigen::VectorXd, std::vector<Eigen::VectorXd>> solveCDDPMPC(){
        int state_dim   = 4;  // (x, y, theta, v)
        int control_dim = 2;  // (steering_angle, acceleration)
        
        // Set initial state
        cddp_solver_->setInitialState(initial_state_);

        // Set goal state
        cddp_solver_->setReferenceState(goal_state_);

        // Provide an initial trajectory guess (X, U)
        std::vector<Eigen::VectorXd> X(horizon_ + 1, Eigen::VectorXd::Zero(state_dim));
        std::vector<Eigen::VectorXd> U(horizon_,     Eigen::VectorXd::Zero(control_dim));
        X[0] = initial_state_;
        for (int i = 0; i < horizon_; i++){
            U[i] = Eigen::VectorXd::Zero(control_dim);
            X[i+1] = initial_state_;
        }
        cddp_solver_->setInitialTrajectory(X, U);

        RCLCPP_INFO(this->get_logger(), "CDDP solver has been constructed");

        // Solve the problem
        cddp::CDDPSolution solution = cddp_solver_->solve("MSIPDDP");

        // Extract solution
        auto X_sol = std::any_cast<std::vector<Eigen::VectorXd>>(solution.at("state_trajectory")); // size: horizon + 1
        auto U_sol = std::any_cast<std::vector<Eigen::VectorXd>>(solution.at("control_trajectory")); // size: horizon
        auto t_sol = std::any_cast<std::vector<double>>(solution.at("time_points")); // size: horizon + 1   

        // Extract control
        Eigen::VectorXd u = U_sol[0];
    
        RCLCPP_INFO(this->get_logger(), "Final state = [%f, %f]", X_sol[horizon_](0), X_sol[horizon_](1));

        return std::make_tuple(u, X_sol);   
    }
    

    Eigen::VectorXd getEulerFromQuaternion(const Eigen::Quaterniond& quat){
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
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
    
    // Member variables
    std::string robot_id_;
    double timestep_;
    int horizon_;
    double processing_frequency_;
    int goal_index_;
    double max_compute_time_;
    double wheelbase_;
    double max_steering_angle_;
    double delta_max_;
    double delta_min_;
    double a_max_;
    double a_min_;
    double Q_x_;
    double Q_y_;
    double Q_theta_;
    double Q_v_;
    double R_delta_;
    double R_a_;
    double Qf_x_;
    double Qf_y_;
    double Qf_theta_;
    double Qf_v_;

    geometry_msgs::msg::Pose goal_pose_;
    geometry_msgs::msg::Pose initial_pose_;
    Eigen::VectorXd goal_state_;
    Eigen::VectorXd initial_state_;
    std::vector<Eigen::VectorXd> X_ref_;
    std::unique_ptr<cddp::CDDP> cddp_solver_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_; 
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarMPCNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}