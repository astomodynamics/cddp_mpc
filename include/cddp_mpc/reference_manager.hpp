#ifndef CDDP_MPC_REFERENCE_MANAGER_HPP
#define CDDP_MPC_REFERENCE_MANAGER_HPP

#include <optional>
#include <string>

#include <Eigen/Dense>

namespace cddp_mpc {

enum class ReferenceSource {
  Mission,
  GoalPose,
  Teleop,
};

enum class TeleopFrame {
  World,
  Body,
};

struct ReferenceManagerConfig {
  double smoothing_tau_s{0.6};
  double max_xy_speed_mps{1.0};
  double max_z_speed_mps{1.0};
  double max_xy_accel_mps2{1.5};
  double max_z_accel_mps2{1.5};
  double max_yaw_rate_rad_s{0.6};
  double teleop_timeout_s{0.2};
  double teleop_max_xy_speed_mps{1.0};
  double teleop_max_z_speed_mps{0.6};
  double teleop_max_yaw_rate_rad_s{0.8};
  bool teleop_require_deadman{true};
  double goal_timeout_s{1.0};
  double geofence_half_extent_x_m{5.0};
  double geofence_half_extent_y_m{5.0};
  double geofence_min_z_m{0.0};
  double geofence_max_z_m{5.0};
};

struct MissionReference {
  std::string mode{"INIT"};
  Eigen::Vector3d target_position_enu{Eigen::Vector3d::Zero()};
  double target_yaw_rad{0.0};
};

struct ReferenceStatus {
  ReferenceSource source{ReferenceSource::Mission};
  std::string source_label{"mission"};
  Eigen::Vector3d target_position_enu{Eigen::Vector3d::Zero()};
  double target_yaw_rad{0.0};
  bool teleop_active{false};
  bool teleop_deadman_pressed{false};
  bool goal_fresh{false};
  bool geofence_clamped{false};
};

class ReferenceManager {
public:
  explicit ReferenceManager(ReferenceManagerConfig config = {});

  void setConfig(const ReferenceManagerConfig &config);
  void updateTeleopCommand(double time_s, const Eigen::Vector3d &linear_velocity,
                           double yaw_rate_rad_s, TeleopFrame frame);
  void updateDeadmanState(double time_s, bool pressed);
  void updateGoalPose(double time_s, const Eigen::Vector3d &position_enu,
                      double yaw_rad);

  ReferenceStatus update(double time_s, const Eigen::Vector3d &current_position_enu,
                         double current_yaw_rad,
                         const MissionReference &mission_reference);

private:
  struct TeleopCommand {
    double time_s{0.0};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    double yaw_rate_rad_s{0.0};
    TeleopFrame frame{TeleopFrame::Body};
  };

  struct GoalPose {
    double time_s{0.0};
    Eigen::Vector3d position_enu{Eigen::Vector3d::Zero()};
    double yaw_rad{0.0};
  };

  void initializeTarget(const Eigen::Vector3d &position_enu, double yaw_rad);
  Eigen::Vector3d clampToGeofence(const Eigen::Vector3d &position_enu,
                                  bool *clamped) const;
  Eigen::Vector3d teleopVelocityWorld(double current_yaw_rad) const;
  void advanceTarget(const Eigen::Vector3d &desired_position_enu, double desired_yaw_rad,
                     double dt_s);

  ReferenceManagerConfig config_;
  std::optional<TeleopCommand> latest_teleop_command_;
  std::optional<GoalPose> latest_goal_pose_;
  bool deadman_pressed_{false};
  bool initialized_{false};
  std::optional<double> last_update_time_s_;
  Eigen::Vector3d active_target_position_enu_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d active_target_velocity_enu_{Eigen::Vector3d::Zero()};
  double active_target_yaw_rad_{0.0};
};

} // namespace cddp_mpc

#endif
