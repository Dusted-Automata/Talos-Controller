#pragma once

#include "types.hpp"

class Motion_Profile
{
  public:
    Motion_Profile(Kinematic_Constraints constraints) : constraints(constraints) {};

    void setCompFactor(int aFactor);
    void reset();
    float update(float aSetpoint, double dt);

    virtual double calculate_position(const double setpoint) = 0;

    bool isFinished = false;

  private:
    using clock = std::chrono::steady_clock;
    Kinematic_Constraints constraints;

    double position;
    double velocity;
    double acceleration;

    unsigned long lastTime = 0;

    int compFactor = 6;
};
