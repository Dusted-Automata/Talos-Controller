#include "pid.hpp"
#include <algorithm>

double
PID::update(double setpoint, double measured_value, double dt)
{

    double error = setpoint - measured_value;
    terms.p = gains.k_p * error;

    integral += error * dt;
    // Apply integral limits (anti-windup)
    integral = std::max(gains.integral_min, std::min(gains.integral_max, integral));
    terms.i = gains.k_i * integral;

    double derivative;
    if (dt > 0.0) {
        derivative = (error - prev_error) / dt;
    } else {
        derivative = 0.0;
    }
    terms.d = gains.k_d * derivative;

    double output = terms.p + terms.i + terms.d;

    output = std::max(gains.output_min, std::min(gains.output_max, output));

    prev_error = error;

    return output;
}

void
PID::reset()
{
    prev_error = 0.0;
    integral = 0.0;
}
