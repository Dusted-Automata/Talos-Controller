#include "motion_profile.hpp"
#include <cstdlib>
#include <format>
#include <iostream>

void
Motion_Profile::update(double position, double dt)

{
    isFinished = static_cast<long>(position * (10 ^ compFactor)) == static_cast<long>(setpoint * (10 ^ compFactor));

    if (!isFinished) {
        double old_velocity = velocity;
        velocity = calculate_velocity(position, dt);
        std::cout << velocity << " " << old_velocity << std::endl;
        acceleration = (velocity - old_velocity) / dt;
    }
}

void
Motion_Profile::set_setpoint(double sp)
{
    this->setpoint = sp;
}

// void
// Motion_Profile::calculateConstantVelocityProfile(float setpoint)
// {
//     float suggestedVelocity = (setpoint - position) / delta;
//
//     if (suggestedVelocity > maxVelocity) {
//         position += maxVelocity * delta;
//     } else if (suggestedVelocity < -maxVelocity) {
//         position += -maxVelocity * delta;
//     } else {
//         position += suggestedVelocity * delta;
//     }
// }
//

double
Trapezoidal_Profile::calculate_velocity(const double position, const double dt)
{
    // Check if we need to de-accelerate
    double vel = 0;
    double pos_threshold_for_deccel = (v_max * v_max) / (2 * std::abs(a_min));
    if (setpoint - pos_threshold_for_deccel <= position) {
        // if (velocity < 0) {
        //     vel = velocity + (a_min * dt);
        // } else if (velocity > 0) {
        std::cout << "a_min: " << a_min << std::endl;
        // vel = velocity + (a_min * dt);
        vel = velocity + (pos_threshold_for_deccel * dt);
        if (vel < v_min) {
            vel = v_min;
        }
        // }
    } else {
        std::cout << "IN ELSE" << std::endl;
        // We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
        if (vel < v_max) {
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
Motion_Profile::setCompFactor(int aFactor)
{
    compFactor = aFactor;
}

void
Motion_Profile::reset()
{
    // Reset all state variables
    velocity = 0;
    acceleration = 0;
    setpoint = 0;
}
