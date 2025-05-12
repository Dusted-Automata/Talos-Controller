#pragma once
#include "types.hpp"
#include <functional>
#include <thread>

class Path_Controller
{

  public:
    Path_Controller() = default;

    bool path_looping = false;
    Thread_Safe_Queue<Ecef_Coord> path_queue;
    std::vector<Ecef_Coord> path_points_all;

    void add_waypoints(const std::vector<Ecef_Coord> &waypoints);
    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> front_two();
    void goal_reached();
};
