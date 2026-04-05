#ifndef CDDP_MPC_REFERENCE_PROVIDER_HPP
#define CDDP_MPC_REFERENCE_PROVIDER_HPP

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace cddp_mpc {

struct ReferenceTrajectory {
  std::vector<Eigen::VectorXd> states;
  std::vector<Eigen::VectorXd> controls;
};

struct ReferenceConfig {
  int horizon_steps{20};
  double mpc_dt{0.1};
  double hover_thrust_n{0.0};
  double min_thrust_n{0.0};
  double max_thrust_n{20.0};
  double max_axis_speed_mps{1.0};
  double max_axis_accel_mps2{1.5};
  double max_yaw_rate_rad_s{0.6};
  double smoothing_tau_s{0.6};
  double target_yaw_rad{0.0};
  double trajectory_period_s{8.0};
  double circle_radius_m{0.5};
  double figure_eight_amplitude_m{0.5};
  double vertical_amplitude_m{0.0};
  bool allow_reference_jump{false};
  std::string mode{"active_setpoint"};
};

class ReferenceProvider {
public:
  virtual ~ReferenceProvider() = default;

  virtual ReferenceTrajectory build(const Eigen::VectorXd &initial_state) const = 0;
};

class PositionYawReferenceProvider : public ReferenceProvider {
public:
  explicit PositionYawReferenceProvider(ReferenceConfig config);

  ReferenceTrajectory build(const Eigen::VectorXd &initial_state) const override;

private:
  struct AxisProfile {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
  };

  struct TrajectoryPoint {
    Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw_rad{0.0};
  };

  AxisProfile buildAxisProfile(double position_error, double velocity,
                               double target_position_error) const;
  double clampYawStep(double previous_yaw, double desired_yaw) const;
  TrajectoryPoint buildPointFromMode(double time_s, double previous_yaw) const;
  TrajectoryPoint buildPointFromProfiles(const AxisProfile &x_profile,
                                         const AxisProfile &y_profile,
                                         const AxisProfile &z_profile,
                                         std::size_t idx,
                                         double previous_yaw) const;

  ReferenceConfig config_;
};

} // namespace cddp_mpc

#endif
