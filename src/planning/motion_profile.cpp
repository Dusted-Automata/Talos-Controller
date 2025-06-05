#include "motion_profile.hpp"
#include <cstdlib>
#include <format>
#include <iostream>

void
Motion_Profile::update(double position, double dt)

{
    // isFinished = static_cast<long>(position * (10 ^ compFactor)) == static_cast<long>(setpoint * (10 ^ compFactor));
    //
    // if (!isFinished) {
    double old_velocity = velocity;
    velocity = calculate_velocity(position, dt);
    acceleration = (velocity - old_velocity) / dt;
    // }
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
    double stopping_distance = (velocity * velocity) / (std::abs(a_min));
    double remaining_distance = setpoint - position;
    std::string msg = std::format("stopping distance: {} | remaining distance: {}", stopping_distance,
        remaining_distance);
    std::cout << msg << std::endl;
    // std::cout << position << std::endl;
    if (std::abs(remaining_distance) <= stopping_distance) {
        test--;
        std::cout << test << std::endl;
        // if (velocity < 0) {
        //     vel = velocity + (a_min * dt);
        // } else if (velocity > 0) {
        std::cout << "a_min: " << a_min << std::endl;
        vel = velocity + (a_min * dt);
        if (velocity < 0.0) {
            vel = 0.0;
        }
        // }
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
