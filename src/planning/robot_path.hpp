#pragma once
#include "types.hpp"

class Robot_Path
{

  private:
    Thread_Safe_Queue<Ecef> queue;

  public:
    std::vector<Ecef> path_points_all;
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Ecef> &waypoints);
    std::optional<Ecef> get_next();
    void pop();
    bool read_json_latlon(std::string file_path);
};
