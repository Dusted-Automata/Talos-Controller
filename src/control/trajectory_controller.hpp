#pragma once
#include "motion_profile.hpp"
#include "types.hpp"

class Robot;

class Trajectory_Controller
{

  public:
    virtual ~Trajectory_Controller() = default;
    virtual Velocity2d get_cmd() = 0;
    virtual double get_accel() = 0;
    Robot *robot;
};
