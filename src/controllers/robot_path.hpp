#pragma once
#include "types.hpp"

class Robot_Path
{

  private:
    Thread_Safe_Queue<Ecef_Coord> queue;
    std::vector<Ecef_Coord> path_points_all;

  public:
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Ecef_Coord> &waypoints);
    std::optional<Ecef_Coord> get_next();
    void goal_reached();
};
