#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TeleopKeyboard : public rclcpp::Node {
public:
  TeleopKeyboard()
  : Node("teleop_keyboard")
  {
    // Declare and retrieve the robot_id parameter (default "robot_1")
    this->declare_parameter<std::string>("robot_id", "robot_1");
    robot_id_ = this->get_parameter("robot_id").as_string();

    // Construct the cmd_vel topic based on robot_id.
    std::string cmd_vel_topic = robot_id_ + "/cmd_vel";
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Start the keyboard reading thread.
    keyboard_thread_ = std::thread(std::bind(&TeleopKeyboard::keyboardLoop, this));
  }
  
  ~TeleopKeyboard() {
    exit_requested_ = true;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }
  
private:
  void keyboardLoop() {
    char c;
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    const double linear_step = 0.1;
    const double angular_step = 0.1;
    
    // Set terminal to raw mode for immediate key press reading.
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    std::cout << "\nTeleop Keyboard Control for robot: " << robot_id_ << "\n";
    std::cout << "-----------------------------------------\n";
    std::cout << "w: increase forward speed\n";
    std::cout << "s: decrease forward speed (or move backwards)\n";
    std::cout << "a: turn left\n";
    std::cout << "d: turn right\n";
    std::cout << "x: stop\n";
    std::cout << "q: quit\n";
    
    while (rclcpp::ok() && !exit_requested_) {
      if (read(STDIN_FILENO, &c, 1) > 0) {
        geometry_msgs::msg::Twist twist_msg;
        switch (c) {
          case 'w':
            linear_vel += linear_step;
            break;
          case 's':
            linear_vel -= linear_step;
            break;
          case 'a':
            angular_vel += angular_step;
            break;
          case 'd':
            angular_vel -= angular_step;
            break;
          case 'x':
            linear_vel = 0.0;
            angular_vel = 0.0;
            break;
          case 'q':
            exit_requested_ = true;
            break;
          default:
            break;
        }
        twist_msg.linear.x = linear_vel;
        twist_msg.angular.z = angular_vel;
        publisher_->publish(twist_msg);
        std::cout << "Published to " << robot_id_ << "/cmd_vel: linear = " 
                  << twist_msg.linear.x << ", angular = " << twist_msg.angular.z << "\n";
      }
    }
    // Restore terminal settings before exit.
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::thread keyboard_thread_;
  bool exit_requested_{false};
  std::string robot_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopKeyboard>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
