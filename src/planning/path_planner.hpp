#pragma once

#include "robot_path.hpp"
class Path_Planner
{
  public:
    Robot_Path path;
    Robot_Path global_path;
    Robot_Path local_path;
    Path_Planner() {};
    Path_Planner(Robot_Path path) : path(path) {};
    void gen_global_path(double meters_per_point);

  private:
};
