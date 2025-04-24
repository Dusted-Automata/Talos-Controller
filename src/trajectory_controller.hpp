#pragma once
#include "frame_controller.hpp"
#include "types.hpp"


class Robot;

class Trajectory_Controller
{

  public:
    virtual Velocity2d get_cmd(Frame_Controller &frame_controller,
                               Thread_Safe_Queue<Ecef_Coord> &path_queue) = 0;
    bool path_looping = false;
    Robot *robot;
};
