#include <cmath>
#include <exception>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

#include "cddp_mpc/px4_utils.hpp"

class PX4Visualizer : public rclcpp::Node {
public:
  PX4Visualizer() : Node("px4_visualizer") {
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    auto diag_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", px4_qos,
        std::bind(&PX4Visualizer::attitudeCallback, this, std::placeholders::_1));

    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", px4_qos,
        std::bind(&PX4Visualizer::positionCallback, this, std::placeholders::_1));

    diagnostic_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/cddp_mpc/status", diag_qos,
        std::bind(&PX4Visualizer::diagnosticCallback, this, std::placeholders::_1));

    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/px4_visualizer/vehicle_pose", 10);
    vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/px4_visualizer/vehicle_path", 10);
    setpoint_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/px4_visualizer/setpoint_path", 10);
    velocity_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/px4_visualizer/vehicle_velocity", 10);
    setpoint_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/px4_visualizer/setpoint_marker", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PX4Visualizer::publishVisualization, this));

    RCLCPP_INFO(get_logger(),
                "PX4 visualizer initialized for Foxglove-friendly ROS topics.");
  }

private:
  static geometry_msgs::msg::Quaternion
  toGeometryQuaternion(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }

  static std::optional<double> parseDouble(const std::string &text) {
    try {
      const double value = std::stod(text);
      if (std::isfinite(value)) {
        return value;
      }
    } catch (const std::exception &) {
    }
    return std::nullopt;
  }

  void appendPose(nav_msgs::msg::Path &path,
                  const geometry_msgs::msg::PoseStamped &pose) {
    path.header = pose.header;
    path.poses.push_back(pose);
    constexpr std::size_t kMaxPathPoints = 1000;
    if (path.poses.size() > kMaxPathPoints) {
      path.poses.erase(path.poses.begin());
    }
  }

  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
    const Eigen::Quaterniond q_ned(
        static_cast<double>(msg->q[0]), static_cast<double>(msg->q[1]),
        static_cast<double>(msg->q[2]), static_cast<double>(msg->q[3]));
    current_attitude_ =
        toGeometryQuaternion(cddp_mpc::quatNedToEnuWxyz(q_ned));
    attitude_received_ = true;
  }

  void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_position_.x = static_cast<double>(msg->y);
    current_position_.y = static_cast<double>(msg->x);
    current_position_.z = static_cast<double>(-msg->z);

    current_velocity_.x = static_cast<double>(msg->vy);
    current_velocity_.y = static_cast<double>(msg->vx);
    current_velocity_.z = static_cast<double>(-msg->vz);

    position_received_ = true;
  }

  void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    std::optional<double> setpoint_x;
    std::optional<double> setpoint_y;
    std::optional<double> setpoint_z;

    for (const auto &status : msg->status) {
      if (status.name != "cddp_mpc") {
        continue;
      }
      for (const auto &kv : status.values) {
        if (kv.key == "setpoint_enu_x_m") {
          setpoint_x = parseDouble(kv.value);
        } else if (kv.key == "setpoint_enu_y_m") {
          setpoint_y = parseDouble(kv.value);
        } else if (kv.key == "setpoint_enu_z_m") {
          setpoint_z = parseDouble(kv.value);
        }
      }
    }

    if (!setpoint_x.has_value() || !setpoint_y.has_value() ||
        !setpoint_z.has_value()) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "map";
    pose.pose.position.x = *setpoint_x;
    pose.pose.position.y = *setpoint_y;
    pose.pose.position.z = *setpoint_z;
    pose.pose.orientation.w = 1.0;

    current_setpoint_pose_ = pose;
    appendPose(setpoint_path_, pose);
  }

  void publishVisualization() {
    if (!position_received_ || !attitude_received_) {
      return;
    }

    const auto stamp = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position = current_position_;
    pose_msg.pose.orientation = current_attitude_;
    vehicle_pose_pub_->publish(pose_msg);

    appendPose(vehicle_path_, pose_msg);
    vehicle_path_pub_->publish(vehicle_path_);

    setpoint_path_.header.stamp = stamp;
    setpoint_path_.header.frame_id = "map";
    setpoint_path_pub_->publish(setpoint_path_);

    visualization_msgs::msg::Marker velocity_marker;
    velocity_marker.header.stamp = stamp;
    velocity_marker.header.frame_id = "map";
    velocity_marker.ns = "velocity";
    velocity_marker.id = 0;
    velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
    velocity_marker.action = visualization_msgs::msg::Marker::ADD;
    velocity_marker.points.resize(2);
    velocity_marker.points[0].x = current_position_.x;
    velocity_marker.points[0].y = current_position_.y;
    velocity_marker.points[0].z = current_position_.z;

    constexpr double kVelocityScale = 0.5;
    velocity_marker.points[1].x = current_position_.x + current_velocity_.x * kVelocityScale;
    velocity_marker.points[1].y = current_position_.y + current_velocity_.y * kVelocityScale;
    velocity_marker.points[1].z = current_position_.z + current_velocity_.z * kVelocityScale;
    velocity_marker.scale.x = 0.1;
    velocity_marker.scale.y = 0.2;
    velocity_marker.scale.z = 0.2;
    velocity_marker.color.a = 1.0;
    velocity_marker.color.r = 1.0;
    velocity_marker_pub_->publish(velocity_marker);

    visualization_msgs::msg::Marker setpoint_marker;
    setpoint_marker.header.stamp = stamp;
    setpoint_marker.header.frame_id = "map";
    setpoint_marker.ns = "setpoint";
    setpoint_marker.id = 1;
    setpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
    setpoint_marker.action = visualization_msgs::msg::Marker::ADD;
    setpoint_marker.scale.x = 0.2;
    setpoint_marker.scale.y = 0.2;
    setpoint_marker.scale.z = 0.2;
    setpoint_marker.color.a = 1.0;
    setpoint_marker.color.g = 1.0;
    if (current_setpoint_pose_.has_value()) {
      setpoint_marker.pose = current_setpoint_pose_->pose;
    } else {
      setpoint_marker.color.a = 0.0;
    }
    setpoint_marker_pub_->publish(setpoint_marker);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Point current_position_{};
  geometry_msgs::msg::Point current_velocity_{};
  geometry_msgs::msg::Quaternion current_attitude_{};
  bool position_received_{false};
  bool attitude_received_{false};
  nav_msgs::msg::Path vehicle_path_{};
  nav_msgs::msg::Path setpoint_path_{};
  std::optional<geometry_msgs::msg::PoseStamped> current_setpoint_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Visualizer>());
  rclcpp::shutdown();
  return 0;
}
