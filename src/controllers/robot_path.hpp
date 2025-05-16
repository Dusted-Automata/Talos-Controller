#pragma once
#include "types.hpp"

class Robot_Path
{

  private:
    std::vector<Ecef> path_points_all;
    Thread_Safe_Queue<Ecef> queue;

  public:
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Ecef> &waypoints);
    std::optional<Ecef> get_next();
    void goal_reached();
};
