#pragma once
#include "types.hpp"

class Robot;

class Trajectory_Controller
{

  public:
    virtual Velocity2d get_cmd() = 0;
    bool path_looping = false;
    Robot *robot;
};
