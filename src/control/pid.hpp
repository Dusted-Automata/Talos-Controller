#pragma once

struct PIDGains {
    double k_p, k_i, k_d;
    double output_min;
    double output_max;
    double integral_min;
    double integral_max;
};

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

  private:
    PIDGains gains;
};
