#pragma once
#include "types.hpp"

class Robot_Path
{

  private:
    size_t goal_index = 1;
    size_t current_index = 0;
    std::vector<Pose> path;

  public:
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Pose> &waypoints);
    std::optional<Pose> current();
    std::optional<Pose> next();
    void progress();
    bool read_json_latlon(std::string file_path);
    size_t size();
    void reset();

    Pose &operator[](size_t index);
    const Pose &operator[](size_t index) const;
};
