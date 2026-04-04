#ifndef CDDP_MPC_ERROR_STATE_QUADROTOR_RATE_HPP
#define CDDP_MPC_ERROR_STATE_QUADROTOR_RATE_HPP

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "dynamics_model/quadrotor_rate.hpp"

namespace cddp_mpc {

class ErrorStateEnuQuadrotorRate : public cddp::DynamicalSystem {
public:
  ErrorStateEnuQuadrotorRate(double timestep, double mass, double max_thrust,
                             double max_rate, Eigen::Vector3d setpoint_enu,
                             std::string integration_type = "rk4")
      : cddp::DynamicalSystem(10, 4, timestep, integration_type),
        abs_enu_model_(std::make_unique<cddp::QuadrotorRate>(
            timestep, mass, max_thrust, max_rate, std::move(integration_type))),
        setpoint_enu_(std::move(setpoint_enu)) {}

  Eigen::VectorXd getContinuousDynamics(const Eigen::VectorXd &state,
                                        const Eigen::VectorXd &control,
                                        double time) const override {
    const Eigen::VectorXd abs_enu_state = errorStateEnuToAbsEnu(state);
    const Eigen::VectorXd abs_enu_dot =
        abs_enu_model_->getContinuousDynamics(abs_enu_state, control, time);
    return absEnuDerivativeToErrorStateEnu(abs_enu_dot);
  }

  cddp::VectorXdual2nd
  getContinuousDynamicsAutodiff(const cddp::VectorXdual2nd &state,
                                const cddp::VectorXdual2nd &control,
                                double time) const override {
    const cddp::VectorXdual2nd abs_enu_state = errorStateEnuToAbsEnu(state);
    const cddp::VectorXdual2nd abs_enu_dot =
        abs_enu_model_->getContinuousDynamicsAutodiff(abs_enu_state, control, time);
    return absEnuDerivativeToErrorStateEnu(abs_enu_dot);
  }

  Eigen::VectorXd getDiscreteDynamics(const Eigen::VectorXd &state,
                                      const Eigen::VectorXd &control,
                                      double time) const override {
    const Eigen::VectorXd abs_enu_state = errorStateEnuToAbsEnu(state);
    const Eigen::VectorXd next_abs_enu =
        abs_enu_model_->getDiscreteDynamics(abs_enu_state, control, time);
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
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_abs_enu(10);
    const Eigen::Matrix<Scalar, 3, 1> setpoint_enu =
        setpoint_enu_.template cast<Scalar>();
    const Eigen::Matrix<Scalar, 3, 1> position_enu =
        state_error_enu.template segment<3>(0) + setpoint_enu;
    const Eigen::Matrix<Scalar, 3, 1> velocity_enu =
        state_error_enu.template segment<3>(3);
    const Eigen::Matrix<Scalar, 4, 1> quat_enu_wxyz =
        normalizeQuaternionWxyz(state_error_enu.template segment<4>(6));

    state_abs_enu.template segment<3>(0) = position_enu;
    state_abs_enu.template segment<3>(3) = velocity_enu;
    state_abs_enu.template segment<4>(6) = quat_enu_wxyz;
    return state_abs_enu;
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  absEnuToErrorStateEnu(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &state_abs_enu) const {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_error_enu(10);
    const Eigen::Matrix<Scalar, 3, 1> setpoint_enu =
        setpoint_enu_.template cast<Scalar>();
    const Eigen::Matrix<Scalar, 3, 1> position_enu =
        state_abs_enu.template segment<3>(0);
    const Eigen::Matrix<Scalar, 3, 1> velocity_enu =
        state_abs_enu.template segment<3>(3);
    const Eigen::Matrix<Scalar, 4, 1> quat_enu_wxyz =
        normalizeQuaternionWxyz(state_abs_enu.template segment<4>(6));

    state_error_enu.template segment<3>(0) = position_enu - setpoint_enu;
    state_error_enu.template segment<3>(3) = velocity_enu;
    state_error_enu.template segment<4>(6) = quat_enu_wxyz;
    return state_error_enu;
  }

  template <typename Scalar>
  static Eigen::Matrix<Scalar, Eigen::Dynamic, 1> absEnuDerivativeToErrorStateEnu(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &state_dot_abs_enu) {
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> state_dot_error_enu(10);
    state_dot_error_enu.template segment<3>(0) =
        state_dot_abs_enu.template segment<3>(0);
    state_dot_error_enu.template segment<3>(3) =
        state_dot_abs_enu.template segment<3>(3);
    state_dot_error_enu.template segment<4>(6) =
        state_dot_abs_enu.template segment<4>(6);
    return state_dot_error_enu;
  }

  std::unique_ptr<cddp::QuadrotorRate> abs_enu_model_;
  Eigen::Vector3d setpoint_enu_;
};

} // namespace cddp_mpc

#endif
