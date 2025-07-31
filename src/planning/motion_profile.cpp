#include "motion_profile.hpp"
#include <cstdlib>
#include <iostream>

void
Motion_Profile::update(double position, double dt)

{
    double old_velocity = velocity;
    velocity = calculate_velocity(position, dt);
    acceleration = (velocity - old_velocity) / dt;
}

void
Motion_Profile::set_setpoint(double sp)
{
    this->setpoint = sp;
}

double
Trapezoidal_Profile::calculate_velocity(const double position, const double dt)
{
    // Check if we need to de-accelerate
    double vel = 0;
    double stopping_distance = velocity / (std::abs(a_min));
    std::cout << "position: " << position << " | stopping_distance " << stopping_distance << std::endl;
    if (std::abs(position) <= stopping_distance) {
        vel = velocity + (a_min * dt);
        if (velocity < 0.0) {
            vel = 0.0;
        }
    } else {
        // We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
        if (velocity < v_max) {
            vel = velocity + a_max * dt;
            if (vel > v_max) {
                vel = v_max;
            }
        } else {
            vel = v_max;
        }
    }
    return vel;
}

void
Motion_Profile::reset()
{
    velocity = 0;
    acceleration = 0;
    setpoint = 0;
}
