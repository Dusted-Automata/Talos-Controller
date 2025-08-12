#pragma once
#include "types.hpp"

enum class Path_Direction {
    NORMAL,
    REVERSE,
    LOOP,
};

class Robot_Path
{

  private:
    size_t goal_index = 1;
    size_t current_index = 0;
    std::vector<Pose> path;
    bool reverse_forward = false;

  public:
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Pose> &waypoints);
    void add_waypoint(const Pose waypoint);
    Pose current();
    Pose next();
    bool progress(const Path_Direction &dir);
    bool read_json_latlon(std::string file_path);
    size_t size();
    void reset();
    void print();

    Pose &operator[](size_t index);
    const Pose &operator[](size_t index) const;
};
