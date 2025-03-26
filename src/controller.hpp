#pragma once
#include "types.hpp"
class Controller
{
  public:
    virtual Velocity2d get_cmd(Pose_State &state) = 0;
    bool path_looping = false;
};
