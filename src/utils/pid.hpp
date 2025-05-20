#pragma once

#include "types.hpp"
struct PIDGains {
    double k_p, k_i, k_d;
};

class PIDController
{
  public:
    PIDController(PIDGains gains) : gains(gains) {};

    double update(double setpoint, double measured_value, double dt);
    void reset();
    double integral;
    double setpoint;
    double prev_error;

    float time = 0;

    double output_min;
    double output_max;
    double integral_min;
    double integral_max;

  private:
    PIDGains gains;
};

class LinearPID : public PIDController
{

  public:
    LinearPID(Robot_Config &config, PIDGains gains) : PIDController(gains)
    {
        output_min = config.motion_constraints.v_min;
        output_max = config.motion_constraints.v_max;
    }
};

class AngularPID : public PIDController
{

  public:
    AngularPID(Robot_Config &config, PIDGains gains) : PIDController(gains)
    {
        output_min = config.motion_constraints.omega_min;
        output_max = config.motion_constraints.omega_max;
    }
};
