#ifndef UNICYCLE_HPP
#define UNICYCLE_HPP

#include <vector>
#include <cmath>

/**
 * @brief A simple unicycle kinematics model.
 *
 * The model accepts linear and angular velocity commands
 * and uses a Runge–Kutta 4 (RK4) integration method to update its state.
 */
class Unicycle {
public:
  // Default constructor initializes at the origin with zero heading.
  Unicycle();
  // Initialize with a given starting pose.
  Unicycle(double init_x, double init_y, double init_theta);
  ~Unicycle();

  /// Set the commanded linear (m/s) and angular (rad/s) velocities.
  void setCommand(double linear_vel, double angular_vel);
  
  /// Propagate the state forward by dt seconds.
  void update(double dt);

  /// Return the current state as a vector: [x, y, theta].
  std::vector<double> getState() const;

private:
  // Current state (x, y, heading).
  double x_;
  double y_;
  double theta_;

  // Commanded inputs.
  double linear_cmd_;
  double angular_cmd_;

  // Helper: compute derivatives given state and commands.
  void computeDerivatives(double x, double y, double theta,
                          double& dx, double& dy, double& dtheta) const;

  // Helper: propagate state using RK4 integration.
  void integrateRK4(double dt);
};

#endif  // UNICYCLE_HPP
