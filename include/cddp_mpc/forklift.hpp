#ifndef FORKLIFT_HPP
#define FORKLIFT_HPP

#include <vector>
#include <cmath>

/**
 * @brief A forklift kinematics model with bicycle dynamics.
 *
 * The model accepts acceleration and steering rate commands
 * and uses a Runge–Kutta 4 (RK4) integration method to update its state.
 * State: [x, y, theta, v, delta] where:
 * - x, y: position
 * - theta: heading angle
 * - v: velocity
 * - delta: steering angle
 * Control: [acceleration, steering_rate]
 */
class Forklift {
public:
  // Default constructor initializes at the origin with zero heading and velocity.
  Forklift();
  // Initialize with a given starting state and parameters.
  Forklift(double init_x, double init_y, double init_theta, 
           double wheelbase = 1.6, bool rear_steer = true, 
           double max_steering_angle = 0.9);
  ~Forklift();

  /// Set the commanded acceleration (m/s^2) and steering rate (rad/s).
  void setCommand(double acceleration, double steering_rate);
  
  /// Propagate the state forward by dt seconds.
  void update(double dt);

  /// Return the current state as a vector: [x, y, theta, v, delta].
  std::vector<double> getState() const;

  /// Set the current state from a vector: [x, y, theta, v, delta].
  void setState(const std::vector<double>& state);

  /// Get forklift parameters
  double getWheelbase() const { return wheelbase_; }
  bool isRearSteer() const { return rear_steer_; }
  double getMaxSteeringAngle() const { return max_steering_angle_; }

private:
  // Current state (x, y, heading, velocity, steering_angle).
  double x_;
  double y_;
  double theta_;
  double v_;
  double delta_;

  // Commanded inputs.
  double acceleration_cmd_;
  double steering_rate_cmd_;

  // Forklift parameters
  double wheelbase_;
  bool rear_steer_;
  double max_steering_angle_;

  // Helper: compute derivatives given state and commands.
  void computeDerivatives(double x, double y, double theta, double v, double delta,
                          double& dx, double& dy, double& dtheta, double& dv, double& ddelta) const;

  // Helper: propagate state using RK4 integration.
  void integrateRK4(double dt);

  // Helper: clamp steering angle to limits
  double clampSteeringAngle(double angle) const;
};

#endif  // FORKLIFT_HPP