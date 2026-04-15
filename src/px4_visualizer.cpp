#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

#include "cddp_mpc/px4_utils.hpp"

namespace {

std::string trimTrailingSlashes(std::string value) {
  while (value.size() > 1 && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string topicFromPrefix(const std::string &prefix, const std::string &suffix) {
  const std::string normalized_prefix = trimTrailingSlashes(prefix);
  if (normalized_prefix.empty()) {
    return suffix;
  }
  if (normalized_prefix == "/") {
    return "/" + suffix;
  }
  return normalized_prefix + "/" + suffix;
}

} // namespace

class PX4Visualizer : public rclcpp::Node {
public:
  explicit PX4Visualizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("px4_visualizer", options) {
    loadParameters();

    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                       .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        attitude_topic_, px4_qos,
        std::bind(&PX4Visualizer::attitudeCallback, this, std::placeholders::_1));

    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic_, px4_qos,
        std::bind(&PX4Visualizer::positionCallback, this, std::placeholders::_1));

    active_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        active_goal_topic_, viz_qos,
        std::bind(&PX4Visualizer::activeGoalCallback, this, std::placeholders::_1));

    vehicle_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        vehicle_pose_topic_, 10);
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, 10);
    vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        vehicle_path_topic_, 10);
    setpoint_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        setpoint_path_topic_, 10);
    velocity_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        vehicle_velocity_topic_, 10);
    setpoint_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        setpoint_marker_topic_, 10);

    interactive_marker_server_ =
        std::make_unique<interactive_markers::InteractiveMarkerServer>(
            interactive_goal_server_name_, get_node_base_interface(),
            get_node_clock_interface(), get_node_logging_interface(),
            get_node_topics_interface(), get_node_services_interface());
    upsertInteractiveGoalMarker(makeDefaultGoalPose());

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&PX4Visualizer::publishVisualization, this));

    RCLCPP_INFO(get_logger(),
                "PX4 visualizer initialized for fmu_prefix=%s controller_prefix=%s.",
                fmu_prefix_.c_str(), controller_prefix_.c_str());
  }

private:
  static constexpr const char *kInteractiveGoalMarkerName = "goal_reference";
  static constexpr double kGoalSyncPositionToleranceM = 0.05;
  static constexpr double kGoalSyncOrientationTolerance = 1e-3;
  static constexpr double kGoalSyncTimeoutS = 0.5;

  template <typename T>
  T declareOrGet(const std::string &name, const T &default_value) {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, default_value);
    }
    return get_parameter(name).template get_value<T>();
  }

  void loadParameters() {
    fmu_prefix_ = trimTrailingSlashes(declareOrGet("fmu_prefix", std::string("/fmu")));
    controller_prefix_ = trimTrailingSlashes(
        declareOrGet("controller_prefix", std::string("/cddp_mpc")));
    visualizer_prefix_ = trimTrailingSlashes(
        declareOrGet("visualizer_prefix", std::string("/px4_visualizer")));
    map_frame_ = declareOrGet("map_frame", std::string("map"));

    attitude_topic_ = declareOrGet(
        "attitude_topic", topicFromPrefix(fmu_prefix_, "out/vehicle_attitude"));
    local_position_topic_ = declareOrGet(
        "local_position_topic", topicFromPrefix(fmu_prefix_, "out/vehicle_local_position"));
    active_goal_topic_ = declareOrGet(
        "active_goal_topic", topicFromPrefix(controller_prefix_, "active_goal"));
    goal_pose_topic_ = declareOrGet(
        "goal_pose_topic", topicFromPrefix(controller_prefix_, "goal_pose"));
    interactive_goal_server_name_ = declareOrGet(
        "interactive_goal_server", topicFromPrefix(controller_prefix_, "interactive_goal"));
    vehicle_pose_topic_ = declareOrGet(
        "vehicle_pose_topic", topicFromPrefix(visualizer_prefix_, "vehicle_pose"));
    vehicle_path_topic_ = declareOrGet(
        "vehicle_path_topic", topicFromPrefix(visualizer_prefix_, "vehicle_path"));
    setpoint_path_topic_ = declareOrGet(
        "setpoint_path_topic", topicFromPrefix(visualizer_prefix_, "setpoint_path"));
    vehicle_velocity_topic_ = declareOrGet(
        "vehicle_velocity_topic", topicFromPrefix(visualizer_prefix_, "vehicle_velocity"));
    setpoint_marker_topic_ = declareOrGet(
        "setpoint_marker_topic", topicFromPrefix(visualizer_prefix_, "setpoint_marker"));
  }

  static geometry_msgs::msg::Quaternion
  toGeometryQuaternion(const Eigen::Quaterniond &q) {
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
  }

  static bool posesMatch(const geometry_msgs::msg::Pose &lhs,
                         const geometry_msgs::msg::Pose &rhs) {
    const double dx = lhs.position.x - rhs.position.x;
    const double dy = lhs.position.y - rhs.position.y;
    const double dz = lhs.position.z - rhs.position.z;
    const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (position_error > kGoalSyncPositionToleranceM) {
      return false;
    }

    const Eigen::Quaterniond q_lhs(lhs.orientation.w, lhs.orientation.x,
                                   lhs.orientation.y, lhs.orientation.z);
    const Eigen::Quaterniond q_rhs(rhs.orientation.w, rhs.orientation.x,
                                   rhs.orientation.y, rhs.orientation.z);
    return 1.0 - std::abs(q_lhs.dot(q_rhs)) <= kGoalSyncOrientationTolerance;
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

  geometry_msgs::msg::PoseStamped makeDefaultGoalPose() const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = map_frame_;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  visualization_msgs::msg::Marker makeGoalMarkerBody() const {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.7;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.a = 0.85;
    marker.color.r = 1.0;
    marker.color.g = 0.55;
    marker.color.b = 0.0;
    return marker;
  }

  static visualization_msgs::msg::InteractiveMarkerControl
  makeAxisControl(const std::string &name, double x, double y, double z,
                  uint8_t interaction_mode) {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = name;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.interaction_mode = interaction_mode;
    return control;
  }

  visualization_msgs::msg::InteractiveMarker
  makeInteractiveGoalMarker(const geometry_msgs::msg::PoseStamped &pose) const {
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header = pose.header;
    marker.name = kInteractiveGoalMarkerName;
    marker.description = "Drag goal";
    marker.scale = 1.0;
    marker.pose = pose.pose;

    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.name = "goal_visual";
    visual_control.always_visible = true;
    visual_control.interaction_mode =
        visualization_msgs::msg::InteractiveMarkerControl::NONE;
    visual_control.markers.push_back(makeGoalMarkerBody());
    marker.controls.push_back(visual_control);

    marker.controls.push_back(
        makeAxisControl("move_xy", 0.0, 0.0, 1.0,
                        visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE));
    marker.controls.push_back(
        makeAxisControl("move_z", 0.0, 0.0, 1.0,
                        visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS));
    marker.controls.push_back(
        makeAxisControl("rotate_z", 0.0, 0.0, 1.0,
                        visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS));

    return marker;
  }

  void upsertInteractiveGoalMarker(const geometry_msgs::msg::PoseStamped &pose) {
    if (!interactive_marker_server_) {
      return;
    }

    if (!interactive_marker_initialized_) {
      interactive_marker_server_->insert(
          makeInteractiveGoalMarker(pose),
          std::bind(&PX4Visualizer::interactiveGoalFeedbackCallback, this,
                    std::placeholders::_1));
      interactive_marker_initialized_ = true;
    } else {
      interactive_marker_server_->setPose(kInteractiveGoalMarkerName, pose.pose,
                                          pose.header);
    }
    interactive_marker_server_->applyChanges();
  }

  void publishInteractiveGoalPose(const geometry_msgs::msg::PoseStamped &pose) {
    goal_pose_pub_->publish(pose);
  }

  void interactiveGoalFeedbackCallback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          feedback) {
    if (feedback->marker_name != kInteractiveGoalMarkerName) {
      return;
    }
    if (feedback->event_type ==
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
      interactive_goal_dragging_ = true;
      return;
    }
    if (feedback->event_type !=
            visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
        feedback->event_type !=
            visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = feedback->header;
    pose.header.stamp = now();
    pose.header.frame_id =
        pose.header.frame_id.empty() ? map_frame_ : pose.header.frame_id;
    pose.pose = feedback->pose;

    pending_goal_pose_ = pose;
    pending_goal_deadline_ =
        now() + rclcpp::Duration::from_seconds(kGoalSyncTimeoutS);
    interactive_goal_dragging_ =
        feedback->event_type !=
        visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP;
    publishInteractiveGoalPose(pose);
  }

  void activeGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_setpoint_pose_ = *msg;
    appendPose(setpoint_path_, *msg);

    if (interactive_goal_dragging_) {
      return;
    }

    if (pending_goal_pose_.has_value()) {
      if (posesMatch(msg->pose, pending_goal_pose_->pose)) {
        pending_goal_pose_.reset();
      } else if (now() < pending_goal_deadline_) {
        return;
      } else {
        pending_goal_pose_.reset();
      }
    }

    upsertInteractiveGoalMarker(*msg);
  }

  void publishVisualization() {
    if (!position_received_ || !attitude_received_) {
      return;
    }

    const auto stamp = now();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position = current_position_;
    pose_msg.pose.orientation = current_attitude_;
    vehicle_pose_pub_->publish(pose_msg);

    appendPose(vehicle_path_, pose_msg);
    vehicle_path_pub_->publish(vehicle_path_);

    setpoint_path_.header.stamp = stamp;
    setpoint_path_.header.frame_id = map_frame_;
    setpoint_path_pub_->publish(setpoint_path_);

    visualization_msgs::msg::Marker velocity_marker;
    velocity_marker.header.stamp = stamp;
    velocity_marker.header.frame_id = map_frame_;
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
    setpoint_marker.header.frame_id = map_frame_;
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
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr active_goal_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_marker_pub_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_marker_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Point current_position_{};
  geometry_msgs::msg::Point current_velocity_{};
  geometry_msgs::msg::Quaternion current_attitude_{};
  std::string fmu_prefix_{"/fmu"};
  std::string controller_prefix_{"/cddp_mpc"};
  std::string visualizer_prefix_{"/px4_visualizer"};
  std::string map_frame_{"map"};
  std::string attitude_topic_{"/fmu/out/vehicle_attitude"};
  std::string local_position_topic_{"/fmu/out/vehicle_local_position"};
  std::string active_goal_topic_{"/cddp_mpc/active_goal"};
  std::string goal_pose_topic_{"/cddp_mpc/goal_pose"};
  std::string interactive_goal_server_name_{"/cddp_mpc/interactive_goal"};
  std::string vehicle_pose_topic_{"/px4_visualizer/vehicle_pose"};
  std::string vehicle_path_topic_{"/px4_visualizer/vehicle_path"};
  std::string setpoint_path_topic_{"/px4_visualizer/setpoint_path"};
  std::string vehicle_velocity_topic_{"/px4_visualizer/vehicle_velocity"};
  std::string setpoint_marker_topic_{"/px4_visualizer/setpoint_marker"};
  bool position_received_{false};
  bool attitude_received_{false};
  nav_msgs::msg::Path vehicle_path_{};
  nav_msgs::msg::Path setpoint_path_{};
  bool interactive_marker_initialized_{false};
  bool interactive_goal_dragging_{false};
  rclcpp::Time pending_goal_deadline_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> current_setpoint_pose_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const auto options =
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<PX4Visualizer>(options));
  rclcpp::shutdown();
  return 0;
}
