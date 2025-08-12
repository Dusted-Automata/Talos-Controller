#pragma once

#include "robot_path.hpp"
class Path_Planner
{
  public:
    Path_Direction path_direction = Path_Direction::NORMAL;
    Robot_Path global_path;
    Robot_Path local_path;
    Path_Planner() {};
    Path_Planner(Robot_Path path) : global_path(path) {};
    void gen_local_path(double meters_per_point);

  private:
};
