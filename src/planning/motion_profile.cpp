#include "motion_profile.hpp"
#include <cstdlib>

float
Motion_Profile::update(float setpoint, double dt)
{
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= sampleTime) {
        // Calculate elapsed time
        bool new_time_point = timeCalculation();

        isFinished = static_cast<long>(position * (10 ^ compFactor)) == static_cast<long>(setpoint * (10 ^ compFactor));

        if (!isFinished) {
            double old_position = position;
            double old_velocity = velocity;

            if (new_time_point) {
                calculate_position(setpoint)
            }

            velocity = (position - old_position) / dt;
            acceleration = (velocity - old_velocity) / dt;
        }
    }
    return position;
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
// void
// Motion_Profile::calculateTrapezoidalProfile(float setpoint)
// {
//     // Check if we need to de-accelerate
//     if (((velocity * velocity) / maxAcceleration) / 2 >= std::abs(setpoint - position)) {
//         if (velocity < 0) {
//             position += (velocity + maxAcceleration * delta) * delta;
//         } else if (velocity > 0) {
//             position += (velocity - maxAcceleration * delta) * delta;
//         }
//     } else {
//         // We're not too close yet, so no need to de-accelerate. Check if we need to accelerate or maintain velocity.
//         if (std::abs(velocity) < maxVelocity || (setpoint < position && velocity > 0)
//             || (setpoint > position && velocity < 0)) {
//             // We need to accelerate, do so but check the maximum acceleration.
//             // Keep velocity constant at the maximum
//             float suggestedVelocity = 0.0;
//             if (setpoint > position) {
//                 suggestedVelocity = velocity + maxAcceleration * delta;
//                 if (suggestedVelocity > maxVelocity) {
//                     suggestedVelocity = maxVelocity;
//                 }
//             } else {
//                 suggestedVelocity = velocity - maxAcceleration * delta;
//                 if (std::abs(suggestedVelocity) > maxVelocity) {
//                     suggestedVelocity = -maxVelocity;
//                 }
//             }
//             position += suggestedVelocity * delta;
//         } else {
//             // Keep velocity constant at the maximum
//             if (setpoint > position) {
//                 position += maxVelocity * delta;
//             } else {
//                 position += -maxVelocity * delta;
//             }
//         }
//     }
// }

void
Motion_Profile::setCompFactor(int aFactor)
{
    compFactor = aFactor;
}

void
Motion_Profile::reset()
{
    // Reset all state variables
    position = 0;
    velocity = 0;
    acceleration = 0;
}
