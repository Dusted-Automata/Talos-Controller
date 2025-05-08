#pragma once

struct PIDGains {
    double k_p, k_i, k_d;
};

class PIDController
{
  public:
    PIDController(PIDGains gains) : gains(gains) {};

    double update(double measured_value, double dt);
    void reset();
    // Control variables
    double integral;
    double setpoint;
    double prev_error;

    // Generation Tracking
    float time = 0;

    // Constraints
    double output_min;
    double output_max;
    double integral_min;
    double integral_max;

  private:
    PIDGains gains;
};
