#ifndef CDDP_MPC_ERROR_STATE_QUADROTOR_THRUST_HPP
#define CDDP_MPC_ERROR_STATE_QUADROTOR_THRUST_HPP

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "cddp_mpc/px4_utils.hpp"
#include "dynamics_model/quadrotor.hpp"

namespace cddp_mpc {

class ErrorStateEnuQuadrotorThrust : public cddp::DynamicalSystem {
public:
  ErrorStateEnuQuadrotorThrust(double timestep, double mass,
                               const Eigen::Matrix3d &inertia_matrix,
                               double arm_length_m, Eigen::Vector3d setpoint_enu,
                               std::string integration_type = "rk4")
      : cddp::DynamicalSystem(13, 4, timestep, integration_type),
        abs_enu_model_(std::make_unique<cddp::Quadrotor>(
            timestep, mass, inertia_matrix, arm_length_m, integration_type)),
        arm_length_m_(arm_length_m),
        setpoint_enu_(std::move(setpoint_enu)) {}

  Eigen::VectorXd getContinuousDynamics(const Eigen::VectorXd &state,
                                        const Eigen::VectorXd &control,
                                        double time) const override {
    const Eigen::VectorXd abs_enu_state = errorStateEnuToAbsEnu(state);
    const Eigen::VectorXd motor_control = thrustTorqueToMotorForces(control);
    const Eigen::VectorXd abs_enu_dot =
        abs_enu_model_->getContinuousDynamics(abs_enu_state, motor_control, time);
    return absEnuDerivativeToErrorStateEnu(abs_enu_dot);
  }

  cddp::VectorXdual2nd
  getContinuousDynamicsAutodiff(const cddp::VectorXdual2nd &state,
                                const cddp::VectorXdual2nd &control,
                                double time) const override {
    const cddp::VectorXdual2nd abs_enu_state = errorStateEnuToAbsEnu(state);
    const cddp::VectorXdual2nd motor_control = thrustTorqueToMotorForces(control);
    const cddp::VectorXdual2nd abs_enu_dot =
        abs_enu_model_->getContinuousDynamicsAutodiff(abs_enu_state, motor_control, time);
    return absEnuDerivativeToErrorStateEnu(abs_enu_dot);
  }

  Eigen::VectorXd getDiscreteDynamics(const Eigen::VectorXd &state,
                                      const Eigen::VectorXd &control,
                                      double time) const override {
    const Eigen::VectorXd abs_enu_state = errorStateEnuToAbsEnu(state);
    const Eigen::VectorXd motor_control = thrustTorqueToMotorForces(control);
    const Eigen::VectorXd next_abs_enu =
        abs_enu_model_->getDiscreteDynamics(abs_enu_state, motor_control, time);
    return absEnuToErrorStateEnu(next_abs_enu);
  }

private:
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 1>
  normalizeQuaternionWxyz(const Eigen::MatrixBase<Derived> &quat_wxyz) {
    using Scalar = typename Derived::Scalar;
    const Scalar norm = sqrt(quat_wxyz.squaredNorm() + Scalar(1e-16));
    return quat_wxyz / norm;
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  errorStateEnuToAbsEnu(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &state_error_enu) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_abs_enu(13);
    const Eigen::Matrix<Scalar, 3, 1> setpoint_enu =
        setpoint_enu_.template cast<Scalar>();

    state_abs_enu.template segment<3>(0) =
        state_error_enu.template segment<3>(0) + setpoint_enu;
    state_abs_enu.template segment<4>(3) =
        normalizeQuaternionWxyz(state_error_enu.template segment<4>(3));
    state_abs_enu.template segment<3>(7) = state_error_enu.template segment<3>(7);
    state_abs_enu.template segment<3>(10) = state_error_enu.template segment<3>(10);
    return state_abs_enu;
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  absEnuToErrorStateEnu(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &state_abs_enu) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_error_enu(13);
    const Eigen::Matrix<Scalar, 3, 1> setpoint_enu =
        setpoint_enu_.template cast<Scalar>();

    state_error_enu.template segment<3>(0) =
        state_abs_enu.template segment<3>(0) - setpoint_enu;
    state_error_enu.template segment<4>(3) =
        normalizeQuaternionWxyz(state_abs_enu.template segment<4>(3));
    state_error_enu.template segment<3>(7) = state_abs_enu.template segment<3>(7);
    state_error_enu.template segment<3>(10) = state_abs_enu.template segment<3>(10);
    return state_error_enu;
  }

  template <typename Scalar>
  static Eigen::Matrix<Scalar, Eigen::Dynamic, 1> absEnuDerivativeToErrorStateEnu(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &state_dot_abs_enu) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_dot_error_enu(13);
    state_dot_error_enu.template segment<3>(0) =
        state_dot_abs_enu.template segment<3>(0);
    state_dot_error_enu.template segment<4>(3) =
        state_dot_abs_enu.template segment<4>(3);
    state_dot_error_enu.template segment<3>(7) =
        state_dot_abs_enu.template segment<3>(7);
    state_dot_error_enu.template segment<3>(10) =
        state_dot_abs_enu.template segment<3>(10);
    return state_dot_error_enu;
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  thrustTorqueToMotorForces(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &control) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> motor_forces(4);
    const Scalar safe_arm_length = Scalar(std::max(std::abs(arm_length_m_), 1e-6));
    const Scalar safe_yaw_coeff = Scalar(cddp_mpc::kQuadrotorModelYawMomentCoefficient);

    const Scalar collective = control(0);
    const Scalar tau_x = control(1);
    const Scalar tau_y = control(2);
    const Scalar tau_z = control(3);

    motor_forces(0) = Scalar(0.25) * collective + Scalar(0.5) * tau_x / safe_arm_length +
                      Scalar(0.25) * tau_z / safe_yaw_coeff;
    motor_forces(1) = Scalar(0.25) * collective + Scalar(0.5) * tau_y / safe_arm_length -
                      Scalar(0.25) * tau_z / safe_yaw_coeff;
    motor_forces(2) = Scalar(0.25) * collective - Scalar(0.5) * tau_x / safe_arm_length +
                      Scalar(0.25) * tau_z / safe_yaw_coeff;
    motor_forces(3) = Scalar(0.25) * collective - Scalar(0.5) * tau_y / safe_arm_length -
                      Scalar(0.25) * tau_z / safe_yaw_coeff;
    return motor_forces;
  }

  std::unique_ptr<cddp::Quadrotor> abs_enu_model_;
  double arm_length_m_{0.25};
  Eigen::Vector3d setpoint_enu_;
};

} // namespace cddp_mpc

#endif
