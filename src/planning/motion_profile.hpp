#pragma once

#include "types.hpp"

class Motion_Profile
{
  public:
    Motion_Profile(double v_max, double v_min, double a_max, double a_min)
        : v_max(v_max), a_max(a_max), v_min(v_min), a_min(a_min) {};

    Motion_Profile(double v_max, double v_min, double a_max)
        : v_max(v_max), a_max(a_max), v_min(v_min), a_min(-a_max) {};

    Motion_Profile(double v_max, double a_max) : v_max(v_max), a_max(a_max), v_min(-v_max), a_min(-a_max) {};

    void setCompFactor(int aFactor);
    void reset();
    void update(const double position, const double dt);
    void set_setpoint(double setpoint);

    virtual double calculate_velocity(const double position, const double dt) = 0;

    double velocity;
    double acceleration;

  protected:
    double v_max, a_max;
    double v_min, a_min;

    double setpoint;
};

class Trapezoidal_Profile : public Motion_Profile
{
  public:
    Trapezoidal_Profile(double v_max, double v_min, double a_max, double a_min)
        : Motion_Profile(v_max, a_max, v_min, a_min) {};

    Trapezoidal_Profile(double v_max, double v_min, double a_max) : Motion_Profile(v_max, a_max, v_min) {};

    Trapezoidal_Profile(double v_max, double a_max) : Motion_Profile(v_max, a_max) {};
    double calculate_velocity(const double position, const double dt) override;
};
