#ifndef CDDP_MPC_THRUST_ALLOCATION_CONSTRAINT_HPP
#define CDDP_MPC_THRUST_ALLOCATION_CONSTRAINT_HPP

#include <algorithm>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include "cddp_core/constraint.hpp"
#include "cddp_mpc/px4_utils.hpp"

namespace cddp_mpc {

class ThrustAllocationConstraint : public cddp::Constraint {
public:
  ThrustAllocationConstraint(double arm_length_m, double min_motor_thrust_n,
                             double max_motor_thrust_n)
      : cddp::Constraint("ThrustAllocationConstraint"),
        arm_length_m_(arm_length_m),
        min_motor_thrust_n_(min_motor_thrust_n),
        max_motor_thrust_n_(max_motor_thrust_n) {}

  int getDualDim() const override { return 8; }

  Eigen::VectorXd evaluate(const Eigen::VectorXd & /*state*/,
                           const Eigen::VectorXd &control,
                           int /*index*/ = 0) const override {
    const Eigen::Vector4d motors = cddp_mpc::motorForcesFromThrustTorque(
        control(0), control.tail<3>(), arm_length_m_,
        cddp_mpc::kQuadrotorModelYawMomentCoefficient);

    Eigen::VectorXd g(8);
    g.head<4>() = -motors;
    g.tail<4>() = motors;
    return g;
  }

  Eigen::VectorXd getLowerBound() const override {
    return Eigen::VectorXd::Constant(8, -std::numeric_limits<double>::infinity());
  }

  Eigen::VectorXd getUpperBound() const override {
    Eigen::VectorXd bound(8);
    bound.head<4>().setConstant(-min_motor_thrust_n_);
    bound.tail<4>().setConstant(max_motor_thrust_n_);
    return bound;
  }

  Eigen::MatrixXd getStateJacobian(const Eigen::VectorXd &state,
                                   const Eigen::VectorXd & /*control*/,
                                   int /*index*/ = 0) const override {
    return Eigen::MatrixXd::Zero(8, state.size());
  }

  Eigen::MatrixXd getControlJacobian(const Eigen::VectorXd & /*state*/,
                                     const Eigen::VectorXd &control,
                                     int /*index*/ = 0) const override {
    const double safe_arm = std::max(std::abs(arm_length_m_), 1e-6);
    const double safe_yaw = cddp_mpc::kQuadrotorModelYawMomentCoefficient;

    Eigen::Matrix<double, 4, 4> allocation;
    allocation << 0.25, 0.5 / safe_arm, 0.0, 0.25 / safe_yaw,
        0.25, 0.0, 0.5 / safe_arm, -0.25 / safe_yaw,
        0.25, -0.5 / safe_arm, 0.0, 0.25 / safe_yaw,
        0.25, 0.0, -0.5 / safe_arm, -0.25 / safe_yaw;

    Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(8, control.size());
    jac.topRows<4>() = -allocation;
    jac.bottomRows<4>() = allocation;
    return jac;
  }

  double computeViolation(const Eigen::VectorXd &state,
                          const Eigen::VectorXd &control,
                          int index = 0) const override {
    return computeViolationFromValue(evaluate(state, control, index));
  }

  double computeViolationFromValue(const Eigen::VectorXd &g) const override {
    const Eigen::VectorXd upper_bound = getUpperBound();
    const Eigen::VectorXd violation = (g - upper_bound).cwiseMax(0.0);
    return violation.maxCoeff();
  }

private:
  double arm_length_m_{0.25};
  double min_motor_thrust_n_{0.0};
  double max_motor_thrust_n_{5.0};
};

} // namespace cddp_mpc

#endif
