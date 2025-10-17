#pragma once

#include "types.hpp"

class PID
{
  public:
    PID(PIDGains gains) : gains(gains) {};

    double update(double setpoint, double measured_value, double dt);
    void reset();
    double integral;
    double setpoint;
    double prev_error;

    float time = 0;

    PID_Terms terms = {};

  private:
    PIDGains gains;
};
