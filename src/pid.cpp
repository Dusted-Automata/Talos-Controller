#include "pid.hpp"
#include <algorithm>

double PIDController::update(double measured_value, double dt) {
  // auto current_time = std::chrono::steady_clock::now();

  // Calculate time delta
  // double dt;
  // if (first_update_) {
  //   dt = 0.0;
  //   first_update_ = false;
  // } else {
  //   dt = std::chrono::duration<double>(current_time - last_time_).count();
  // }
  // last_time_ = current_time;

  double error = setpoint - measured_value;
  double p_term = gains.k_p * error;

  // Integral term
  integral += error * dt;
  // Apply integral limits (anti-windup)
  integral = std::max(integral_min, std::min(integral_max, integral));
  double i_term = gains.k_i * integral;

  // Derivative term (on measurement to avoid derivative kick)
  double derivative;
  if (dt > 0.0) {
    derivative = (error - prev_error) / dt;
  } else {
    derivative = 0.0;
  }
  double d_term = gains.k_d * derivative;

  // Calculate total output
  double output = p_term + i_term + d_term;

  // Apply output constraints
  output = std::max(output_min, std::min(output_max, output));

  // Store error for next iteration
  prev_error = error;

  return output;
}

void PIDController::reset() {
  prev_error = 0.0;
  integral = 0.0;
}

// public:
//   PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0)
//       : kp(kp), ki(ki), kd(kd), setpoint(0.0), prev_error(0.0),
//       integral(0.0),
//         first_update_(true),
//         output_min_(-std::numeric_limits<double>::infinity()),
//         output_max_(std::numeric_limits<double>::infinity()),
//         integral_min_(-std::numeric_limits<double>::infinity()),
//         integral_max_(std::numeric_limits<double>::infinity()) {
//     reset();
//   }
//
