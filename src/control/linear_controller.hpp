#pragma once
#include "motion_profile.hpp"
#include "pid.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"
#include <vector>

class Linear_Controller
{

  private:
  public:
    static Velocity2d get_cmd(double goal_tolerance, Robot &robot, Trapezoidal_Profile linear_profile, double dt);
};
