#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class GamepadTeleop : public rclcpp::Node {
public:
  GamepadTeleop() : Node("gamepad_teleop") {
    loadParameters();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::QoS(10),
        std::bind(&GamepadTeleop::joyCallback, this, std::placeholders::_1));
    teleop_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(teleop_topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GamepadTeleop::publishCommand, this));

    RCLCPP_INFO(
        get_logger(),
        "Gamepad teleop ready: joy_topic=%s teleop_topic=%s publish_rate_hz=%.1f",
        joy_topic_.c_str(), teleop_topic_.c_str(), publish_rate_hz_);
  }

private:
  template <typename T> T declareOrGet(const std::string &name, const T &default_value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).template get_value<T>();
  }

  void loadParameters() {
    joy_topic_ = declareOrGet("joy_topic", std::string("/joy"));
    teleop_topic_ = declareOrGet("teleop_topic", std::string("/teleop/cmd_vel"));
    frame_id_ = declareOrGet("frame_id", std::string("base_link"));
    publish_rate_hz_ = declareOrGet("publish_rate_hz", 30.0);
    command_timeout_s_ = declareOrGet("command_timeout_s", 0.25);
    deadzone_ = declareOrGet("deadzone", 0.10);

    axis_forward_ = declareOrGet("axis_forward", 1);
    axis_lateral_ = declareOrGet("axis_lateral", 0);
    axis_yaw_ = declareOrGet("axis_yaw", 3);
    axis_descend_ = declareOrGet("axis_descend", 2);
    axis_ascend_ = declareOrGet("axis_ascend", 5);

    scale_forward_ = declareOrGet("scale_forward", 1.0);
    scale_lateral_ = declareOrGet("scale_lateral", 1.0);
    scale_yaw_ = declareOrGet("scale_yaw", 1.0);
    scale_vertical_ = declareOrGet("scale_vertical", 1.0);

    trigger_released_value_ = declareOrGet("trigger_released_value", 1.0);
    trigger_pressed_value_ = declareOrGet("trigger_pressed_value", -1.0);

    max_forward_mps_ = declareOrGet("max_forward_mps", 1.0);
    max_lateral_mps_ = declareOrGet("max_lateral_mps", 1.0);
    max_vertical_mps_ = declareOrGet("max_vertical_mps", 0.6);
    max_yaw_rate_rad_s_ = declareOrGet("max_yaw_rate_rad_s", 0.8);
  }

  static double clampUnit(double value) { return std::clamp(value, -1.0, 1.0); }

  double axisValue(const sensor_msgs::msg::Joy &msg, int index, double scale) const {
    if (index < 0 || index >= static_cast<int>(msg.axes.size())) {
      return 0.0;
    }

    double value = static_cast<double>(msg.axes[static_cast<std::size_t>(index)]) * scale;
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }

    const double sign = value >= 0.0 ? 1.0 : -1.0;
    const double magnitude = (std::abs(value) - deadzone_) / std::max(1.0 - deadzone_, 1e-6);
    return sign * clampUnit(magnitude);
  }

  double triggerValue(const sensor_msgs::msg::Joy &msg, int index) const {
    if (index < 0 || index >= static_cast<int>(msg.axes.size())) {
      return 0.0;
    }

    const double raw = static_cast<double>(msg.axes[static_cast<std::size_t>(index)]);
    const double denominator = trigger_released_value_ - trigger_pressed_value_;
    if (std::abs(denominator) < 1e-6) {
      return 0.0;
    }

    const double pressed = (trigger_released_value_ - raw) / denominator;
    const double clamped = std::clamp(pressed, 0.0, 1.0);
    if (clamped < deadzone_) {
      return 0.0;
    }
    return (clamped - deadzone_) / std::max(1.0 - deadzone_, 1e-6);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    latest_joy_ = msg;
    last_joy_time_ = now();
  }

  void publishCommand() {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = frame_id_;

    if (latest_joy_) {
      const double age_s = (now() - last_joy_time_).seconds();
      if (std::isfinite(age_s) && age_s <= command_timeout_s_) {
        const double forward = axisValue(*latest_joy_, axis_forward_, scale_forward_);
        const double lateral = axisValue(*latest_joy_, axis_lateral_, scale_lateral_);
        const double yaw = axisValue(*latest_joy_, axis_yaw_, scale_yaw_);
        const double ascend = triggerValue(*latest_joy_, axis_ascend_);
        const double descend = triggerValue(*latest_joy_, axis_descend_);
        const double vertical = clampUnit((ascend - descend) * scale_vertical_);

        command.twist.linear.x = forward * max_forward_mps_;
        command.twist.linear.y = lateral * max_lateral_mps_;
        command.twist.linear.z = vertical * max_vertical_mps_;
        command.twist.angular.z = yaw * max_yaw_rate_rad_s_;
      }
    }

    teleop_pub_->publish(command);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr teleop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};

  std::string joy_topic_;
  std::string teleop_topic_;
  std::string frame_id_;

  double publish_rate_hz_{30.0};
  double command_timeout_s_{0.25};
  double deadzone_{0.10};

  int axis_forward_{1};
  int axis_lateral_{0};
  int axis_yaw_{3};
  int axis_descend_{2};
  int axis_ascend_{5};

  double scale_forward_{1.0};
  double scale_lateral_{1.0};
  double scale_yaw_{1.0};
  double scale_vertical_{1.0};

  double trigger_released_value_{1.0};
  double trigger_pressed_value_{-1.0};

  double max_forward_mps_{1.0};
  double max_lateral_mps_{1.0};
  double max_vertical_mps_{0.6};
  double max_yaw_rate_rad_s_{0.8};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
