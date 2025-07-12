#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cddp_mpc/forklift.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

class ForkliftRobotNode : public rclcpp::Node {
public:
  ForkliftRobotNode()
  : Node("forklift_robot_node")
  {
    // Declare and retrieve the robot_id parameter (default "forklift_1")
    this->declare_parameter<std::string>("robot_id", "forklift_1");
    robot_id_ = this->get_parameter("robot_id").as_string();

    // Declare and retrieve initial pose parameters (default to 0.0)
    this->declare_parameter<double>("init_x", 0.0);
    this->declare_parameter<double>("init_y", 0.0);
    this->declare_parameter<double>("init_yaw", 0.0);
    this->declare_parameter<double>("wheelbase", 1.6);
    this->declare_parameter<bool>("rear_steer", true);
    this->declare_parameter<double>("max_steering_angle", 0.9);
    
    double init_x = this->get_parameter("init_x").as_double();
    double init_y = this->get_parameter("init_y").as_double();
    double init_yaw = this->get_parameter("init_yaw").as_double();
    double wheelbase = this->get_parameter("wheelbase").as_double();
    bool rear_steer = this->get_parameter("rear_steer").as_bool();
    double max_steering_angle = this->get_parameter("max_steering_angle").as_double();

    // Create the forklift instance with the initial pose and parameters.
    forklift_ = std::make_shared<Forklift>(init_x, init_y, init_yaw, wheelbase, rear_steer, max_steering_angle);

    // Construct topic names using robot_id.
    std::string cmd_vel_topic = robot_id_ + "/cmd_vel";
    std::string pose_topic = robot_id_ + "/pose";

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&ForkliftRobotNode::cmdVelCallback, this, std::placeholders::_1)
    );
    
    // Publisher for robot state as PoseStamped on the constructed topic.
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    // Timer to update simulation at fixed intervals.
    timer_ = this->create_wall_timer(50ms, std::bind(&ForkliftRobotNode::updateSimulation, this));
    
    last_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Forklift robot node initialized with ID: %s", robot_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Parameters - Wheelbase: %.2f, Rear steer: %s, Max steering: %.2f rad", 
                wheelbase, rear_steer ? "true" : "false", max_steering_angle);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    forklift_->setCommand(msg->linear.x, msg->angular.z);
  }

  void updateSimulation() {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    forklift_->update(dt);
    auto state = forklift_->getState(); // [x, y, theta, v, delta]

    // Prepare and publish the PoseStamped message.
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = state[0];  // x
    pose_msg.pose.position.y = state[1];  // y
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state[2]);  // theta
    pose_msg.pose.orientation = tf2::toMsg(q);

    pose_pub_->publish(pose_msg);
    
    static auto last_log_time = current_time;
    if ((current_time - last_log_time).seconds() > 2.0) {
      RCLCPP_INFO(this->get_logger(), 
                  "State: x=%.2f, y=%.2f, theta=%.2f°, v=%.2f m/s, delta=%.2f°", 
                  state[0], state[1], state[2]*180.0/M_PI, state[3], state[4]*180.0/M_PI);
      last_log_time = current_time;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  std::string robot_id_;
  std::shared_ptr<Forklift> forklift_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForkliftRobotNode>());
  rclcpp::shutdown();
  return 0;
}