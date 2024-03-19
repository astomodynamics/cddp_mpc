#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 



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

    }
    void controlLoop(){

        // Publish control command
        auto twist_msg = geometry_msgs::msg::Twist();
        // twist_msg.linear.x = u(0);
        // twist_msg.angular.z = u(1);

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