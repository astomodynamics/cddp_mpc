#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cddp_mpc/unicycle.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

class UnicycleRobotNode : public rclcpp::Node {
public:
  UnicycleRobotNode()
  : Node("unicycle_robot_node")
  {
    // Declare and retrieve the robot_id parameter (default "robot_1")
    this->declare_parameter<std::string>("robot_id", "robot_1");
    robot_id_ = this->get_parameter("robot_id").as_string();

    // Declare and retrieve initial pose parameters (default to 0.0)
    this->declare_parameter<double>("init_x", 0.0);
    this->declare_parameter<double>("init_y", 0.0);
    this->declare_parameter<double>("init_yaw", 0.0);
    double init_x = this->get_parameter("init_x").as_double();
    double init_y = this->get_parameter("init_y").as_double();
    double init_yaw = this->get_parameter("init_yaw").as_double();

    // Create the unicycle instance with the initial pose.
    unicycle_ = std::make_shared<Unicycle>(init_x, init_y, init_yaw);

    // Construct topic names using robot_id.
    std::string cmd_vel_topic = robot_id_ + "/cmd_vel";
    std::string pose_topic = robot_id_ + "/pose";

    // Subscribe to velocity commands on the constructed topic.
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&UnicycleRobotNode::cmdVelCallback, this, std::placeholders::_1)
    );
    
    // Publisher for robot state as PoseStamped on the constructed topic.
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    // Timer to update simulation at fixed intervals.
    timer_ = this->create_wall_timer(50ms, std::bind(&UnicycleRobotNode::updateSimulation, this));
    
    last_time_ = this->now();
  }

private:
  // Callback to update the unicycle commands from incoming Twist messages.
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    unicycle_->setCommand(msg->linear.x, msg->angular.z);
  }

  // Update simulation state and publish the robot state as a PoseStamped message.
  void updateSimulation() {
    // Compute elapsed time.
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Propagate the unicycle state.
    unicycle_->update(dt);
    auto state = unicycle_->getState(); // [x, y, theta]

    // Prepare and publish the PoseStamped message.
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = current_time;
    // Using robot_id as the header frame_id for reference.
    pose_msg.header.frame_id = "map";

    pose_msg.pose.position.x = state[0];
    pose_msg.pose.position.y = state[1];
    pose_msg.pose.position.z = 0.0;

    // Convert yaw (theta) to a quaternion.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state[2]);
    pose_msg.pose.orientation = tf2::toMsg(q);

    pose_pub_->publish(pose_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;

  std::string robot_id_;
  std::shared_ptr<Unicycle> unicycle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnicycleRobotNode>());
  rclcpp::shutdown();
  return 0;
}
