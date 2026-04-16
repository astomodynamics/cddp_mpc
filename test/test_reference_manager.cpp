#include <gtest/gtest.h>

#include "cddp_mpc/reference_manager.hpp"

namespace {

TEST(ReferenceManagerTest, GoalPoseGetsClampedToGeofence) {
  cddp_mpc::ReferenceManagerConfig config;
  config.geofence_half_extent_x_m = 1.0;
  config.geofence_half_extent_y_m = 2.0;
  config.geofence_min_z_m = 0.0;
  config.geofence_max_z_m = 3.0;

  cddp_mpc::ReferenceManager manager(config);
  manager.updateGoalPose(1.0, Eigen::Vector3d(5.0, -5.0, 10.0), 0.0);

  cddp_mpc::MissionReference mission;
  const cddp_mpc::ReferenceStatus status =
      manager.update(1.0, Eigen::Vector3d::Zero(), 0.0, mission);

  EXPECT_EQ(status.source, cddp_mpc::ReferenceSource::GoalPose);
  EXPECT_TRUE(status.geofence_clamped);
  EXPECT_DOUBLE_EQ(status.target_position_enu.x(), 1.0);
  EXPECT_DOUBLE_EQ(status.target_position_enu.y(), -2.0);
  EXPECT_DOUBLE_EQ(status.target_position_enu.z(), 3.0);
}

TEST(ReferenceManagerTest, TeleopRequiresDeadmanWhenEnabled) {
  cddp_mpc::ReferenceManagerConfig config;
  config.teleop_require_deadman = true;
  cddp_mpc::ReferenceManager manager(config);

  manager.updateTeleopCommand(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), 0.0,
                              cddp_mpc::TeleopFrame::World);

  cddp_mpc::MissionReference mission;
  mission.target_position_enu = Eigen::Vector3d(0.5, 0.0, 0.0);
  const cddp_mpc::ReferenceStatus status =
      manager.update(1.0, Eigen::Vector3d::Zero(), 0.0, mission);

  EXPECT_EQ(status.source, cddp_mpc::ReferenceSource::Mission);
  EXPECT_FALSE(status.teleop_active);
  EXPECT_FALSE(status.teleop_deadman_pressed);
}

TEST(ReferenceManagerTest, TeleopOverridesMissionWhenDeadmanPressed) {
  cddp_mpc::ReferenceManagerConfig config;
  config.teleop_require_deadman = true;
  config.max_xy_speed_mps = 10.0;
  config.max_xy_accel_mps2 = 20.0;
  cddp_mpc::ReferenceManager manager(config);

  manager.updateDeadmanState(1.0, true);
  manager.updateTeleopCommand(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), 0.0,
                              cddp_mpc::TeleopFrame::World);

  cddp_mpc::MissionReference mission;
  mission.target_position_enu = Eigen::Vector3d::Zero();
  manager.update(1.0, Eigen::Vector3d::Zero(), 0.0, mission);
  const cddp_mpc::ReferenceStatus status =
      manager.update(1.1, Eigen::Vector3d::Zero(), 0.0, mission);

  EXPECT_EQ(status.source, cddp_mpc::ReferenceSource::Teleop);
  EXPECT_TRUE(status.teleop_active);
  EXPECT_GT(status.target_position_enu.x(), 0.0);
}

TEST(ReferenceManagerTest, OneShotGoalRemainsActiveAfterFreshnessWindow) {
  cddp_mpc::ReferenceManagerConfig config;
  config.goal_timeout_s = 1.0;
  cddp_mpc::ReferenceManager manager(config);

  manager.updateGoalPose(1.0, Eigen::Vector3d(2.0, 3.0, 4.0), 0.5);

  cddp_mpc::MissionReference mission;
  mission.target_position_enu = Eigen::Vector3d(-1.0, -1.0, -1.0);
  manager.update(1.0, Eigen::Vector3d::Zero(), 0.0, mission);
  const cddp_mpc::ReferenceStatus status =
      manager.update(3.0, Eigen::Vector3d::Zero(), 0.0, mission);

  EXPECT_EQ(status.source, cddp_mpc::ReferenceSource::GoalPose);
  EXPECT_FALSE(status.goal_fresh);
  EXPECT_DOUBLE_EQ(status.target_position_enu.x(), 2.0);
  EXPECT_DOUBLE_EQ(status.target_position_enu.y(), 3.0);
  EXPECT_DOUBLE_EQ(status.target_position_enu.z(), 4.0);
}

} // namespace
