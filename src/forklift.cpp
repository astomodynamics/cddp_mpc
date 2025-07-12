#include "cddp_mpc/forklift.hpp"

Forklift::Forklift() 
  : x_(0.0), y_(0.0), theta_(0.0), v_(0.0), delta_(0.0),
    acceleration_cmd_(0.0), steering_rate_cmd_(0.0),
    wheelbase_(1.6), rear_steer_(true), max_steering_angle_(0.9) {}

Forklift::Forklift(double init_x, double init_y, double init_theta, 
                   double wheelbase, bool rear_steer, double max_steering_angle)
  : x_(init_x), y_(init_y), theta_(init_theta), v_(0.0), delta_(0.0),
    acceleration_cmd_(0.0), steering_rate_cmd_(0.0),
    wheelbase_(wheelbase), rear_steer_(rear_steer), max_steering_angle_(max_steering_angle) {}

Forklift::~Forklift() {}

void Forklift::setCommand(double acceleration, double steering_rate) {
  acceleration_cmd_ = acceleration;
  steering_rate_cmd_ = steering_rate;
}

std::vector<double> Forklift::getState() const {
  return {x_, y_, theta_, v_, delta_};
}

void Forklift::setState(const std::vector<double>& state) {
  if (state.size() >= 5) {
    x_ = state[0];
    y_ = state[1];
    theta_ = state[2];
    v_ = state[3];
    delta_ = clampSteeringAngle(state[4]);
  }
}

double Forklift::clampSteeringAngle(double angle) const {
  return std::max(-max_steering_angle_, std::min(max_steering_angle_, angle));
}

void Forklift::computeDerivatives(double x, double y, double theta, double v, double delta,
                                  double& dx, double& dy, double& dtheta, double& dv, double& ddelta) const {
  // Apply rear-steer sign convention
  const double steer_sign = rear_steer_ ? -1.0 : 1.0;
  const double effective_delta = steer_sign * delta;
  
  // Bicycle model kinematics:
  // dx/dt = v * cos(theta)
  // dy/dt = v * sin(theta)
  // dtheta/dt = v * tan(delta) / L
  // dv/dt = acceleration
  // ddelta/dt = steering_rate
  
  dx = v * std::cos(theta);
  dy = v * std::sin(theta);
  dtheta = v * std::tan(effective_delta) / wheelbase_;
  dv = acceleration_cmd_;
  ddelta = steering_rate_cmd_;
}

void Forklift::integrateRK4(double dt) {
  double k1_x, k1_y, k1_theta, k1_v, k1_delta;
  double k2_x, k2_y, k2_theta, k2_v, k2_delta;
  double k3_x, k3_y, k3_theta, k3_v, k3_delta;
  double k4_x, k4_y, k4_theta, k4_v, k4_delta;

  // k1: initial derivatives.
  computeDerivatives(x_, y_, theta_, v_, delta_, k1_x, k1_y, k1_theta, k1_v, k1_delta);

  // k2: derivatives at midpoint using k1.
  double x_mid = x_ + 0.5 * dt * k1_x;
  double y_mid = y_ + 0.5 * dt * k1_y;
  double theta_mid = theta_ + 0.5 * dt * k1_theta;
  double v_mid = v_ + 0.5 * dt * k1_v;
  double delta_mid = clampSteeringAngle(delta_ + 0.5 * dt * k1_delta);
  computeDerivatives(x_mid, y_mid, theta_mid, v_mid, delta_mid, k2_x, k2_y, k2_theta, k2_v, k2_delta);

  // k3: derivatives at midpoint using k2.
  x_mid = x_ + 0.5 * dt * k2_x;
  y_mid = y_ + 0.5 * dt * k2_y;
  theta_mid = theta_ + 0.5 * dt * k2_theta;
  v_mid = v_ + 0.5 * dt * k2_v;
  delta_mid = clampSteeringAngle(delta_ + 0.5 * dt * k2_delta);
  computeDerivatives(x_mid, y_mid, theta_mid, v_mid, delta_mid, k3_x, k3_y, k3_theta, k3_v, k3_delta);

  // k4: derivatives at end using k3.
  double x_end = x_ + dt * k3_x;
  double y_end = y_ + dt * k3_y;
  double theta_end = theta_ + dt * k3_theta;
  double v_end = v_ + dt * k3_v;
  double delta_end = clampSteeringAngle(delta_ + dt * k3_delta);
  computeDerivatives(x_end, y_end, theta_end, v_end, delta_end, k4_x, k4_y, k4_theta, k4_v, k4_delta);

  // Combine increments.
  x_ += (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
  y_ += (dt / 6.0) * (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y);
  theta_ += (dt / 6.0) * (k1_theta + 2.0 * k2_theta + 2.0 * k3_theta + k4_theta);
  v_ += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v);
  delta_ = clampSteeringAngle(delta_ + (dt / 6.0) * (k1_delta + 2.0 * k2_delta + 2.0 * k3_delta + k4_delta));
}

void Forklift::update(double dt) {
  integrateRK4(dt);
}