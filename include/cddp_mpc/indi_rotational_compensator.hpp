#ifndef CDDP_MPC_INDI_ROTATIONAL_COMPENSATOR_HPP
#define CDDP_MPC_INDI_ROTATIONAL_COMPENSATOR_HPP

#include <optional>

#include <Eigen/Dense>

namespace cddp_mpc {

class IndiRotationalCompensator {
public:
  struct Config {
    bool enabled{false};
    double blend_alpha{0.0};
    double rate_lpf_cutoff_hz{30.0};
    double accel_lpf_cutoff_hz{20.0};
    double torque_correction_limit_nm{0.15};
    bool use_rpm_feedback{false};
    bool debug_logging_enabled{false};
  };

  struct Status {
    bool active{false};
    bool using_rpm_feedback{false};
    bool using_command_feedback_fallback{true};
    bool has_measurement{false};
    double measurement_age_s{0.0};
    double update_rate_hz{0.0};
    Eigen::Vector3d filtered_rates{Eigen::Vector3d::Zero()};
    Eigen::Vector3d filtered_accel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d desired_accel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d torque_estimate{Eigen::Vector3d::Zero()};
    Eigen::Vector3d torque_correction{Eigen::Vector3d::Zero()};
    Eigen::Vector3d corrected_torque{Eigen::Vector3d::Zero()};
  };

  explicit IndiRotationalCompensator(const Eigen::Matrix3d &inertia_matrix);

  void setConfig(const Config &config);
  void reset();
  void updateMeasurement(double timestamp_s, const Eigen::Vector3d &body_rates);
  Eigen::Vector3d computeTorqueCommand(double timestamp_s,
                                       const Eigen::Vector3d &nominal_torque,
                                       const Eigen::Vector3d &body_rates,
                                       const Eigen::Vector3d &estimated_realized_torque);
  const Status &status() const;

private:
  static double lowPassAlpha(double cutoff_hz, double dt_s);

  Eigen::Matrix3d inertia_matrix_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertia_inverse_{Eigen::Matrix3d::Identity()};
  Config config_{};
  Status status_{};
  std::optional<double> last_measurement_timestamp_s_;
};

} // namespace cddp_mpc

#endif
