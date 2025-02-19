#pragma once
#include <array>

struct Robot_State {
  std::array<float, 3>
      position; // (unit: m), from own odometry in inertial frame, usually drift
  std::array<float, 3> velocity; // (unit: m/s), forwardSpeed, sideSpeed,
                                 // rotateSpeed in body frame
  float yawSpeed;                // (unit: rad/s), rotateSpeed in body frame
                                 // std::array<MotorState, 20> motorState;
                                 // IMU imu;
};
