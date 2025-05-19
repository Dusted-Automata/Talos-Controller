#include "pid.hpp"
#include <algorithm>
#include <iostream>

double
PIDController::update(double setpoint, double measured_value, double dt)
{

    double error = setpoint - measured_value;
    double p_term = gains.k_p * error;

    integral += error * dt;
    // Apply integral limits (anti-windup)
    integral = std::max(integral_min, std::min(integral_max, integral));
    double i_term = gains.k_i * integral;

    double derivative;
    if (dt > 0.0) {
        derivative = (error - prev_error) / dt;
    } else {
        derivative = 0.0;
    }
    double d_term = gains.k_d * derivative;

    double output = p_term + i_term + d_term;

    output = std::max(output_min, std::min(output_max, output));

    prev_error = error;

    return output;
}

void
PIDController::reset()
{
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
