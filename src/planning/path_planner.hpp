#pragma once

#include "robot_path.hpp"
class Path_Planner
{
  public:
    Path_Direction path_direction = Path_Direction::NORMAL;
    // Path global_path;
    Path_Cursor* global_cursor;
    Path_Planner() {};
    // Path_Planner(Path path) : global_path(path){ };
    void gen_local_path(double meters_per_point);
    size_t re_identify_position(ENU position);

  private:
};
