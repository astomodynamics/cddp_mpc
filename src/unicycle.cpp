#include "cddp_mpc/unicycle.hpp"

Unicycle::Unicycle() 
  : x_(0.0), y_(0.0), theta_(0.0), linear_cmd_(0.0), angular_cmd_(0.0) {}

Unicycle::Unicycle(double init_x, double init_y, double init_theta)
  : x_(init_x), y_(init_y), theta_(init_theta), linear_cmd_(0.0), angular_cmd_(0.0) {}

Unicycle::~Unicycle() {}

void Unicycle::setCommand(double linear_vel, double angular_vel) {
  linear_cmd_ = linear_vel;
  angular_cmd_ = angular_vel;
}

std::vector<double> Unicycle::getState() const {
  return {x_, y_, theta_};
}

void Unicycle::computeDerivatives(double x, double y, double theta,
                                  double& dx, double& dy, double& dtheta) const {
  // Unicycle kinematics:
  // dx/dt = v * cos(theta)
  // dy/dt = v * sin(theta)
  // dtheta/dt = omega
  dx = linear_cmd_ * std::cos(theta);
  dy = linear_cmd_ * std::sin(theta);
  dtheta = angular_cmd_;
}

void Unicycle::integrateRK4(double dt) {
  double k1_x, k1_y, k1_theta;
  double k2_x, k2_y, k2_theta;
  double k3_x, k3_y, k3_theta;
  double k4_x, k4_y, k4_theta;

  // k1: initial derivatives.
  computeDerivatives(x_, y_, theta_, k1_x, k1_y, k1_theta);

  // k2: derivatives at midpoint using k1.
  double x_mid = x_ + 0.5 * dt * k1_x;
  double y_mid = y_ + 0.5 * dt * k1_y;
  double theta_mid = theta_ + 0.5 * dt * k1_theta;
  computeDerivatives(x_mid, y_mid, theta_mid, k2_x, k2_y, k2_theta);

  // k3: derivatives at midpoint using k2.
  x_mid = x_ + 0.5 * dt * k2_x;
  y_mid = y_ + 0.5 * dt * k2_y;
  theta_mid = theta_ + 0.5 * dt * k2_theta;
  computeDerivatives(x_mid, y_mid, theta_mid, k3_x, k3_y, k3_theta);

  // k4: derivatives at end using k3.
  double x_end = x_ + dt * k3_x;
  double y_end = y_ + dt * k3_y;
  double theta_end = theta_ + dt * k3_theta;
  computeDerivatives(x_end, y_end, theta_end, k4_x, k4_y, k4_theta);

  // Combine increments.
  x_ += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
  y_ += (dt / 6.0) * (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y);
  theta_ += (dt / 6.0) * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta);
}

void Unicycle::update(double dt) {
  integrateRK4(dt);
}
