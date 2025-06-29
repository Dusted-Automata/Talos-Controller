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
    void add_waypoint(const Pose waypoint);
    std::optional<Pose> current();
    std::optional<Pose> next();
    void progress();
    bool read_json_latlon(std::string file_path);
    size_t size();
    void reset();
    void print();

    Pose &operator[](size_t index);
    const Pose &operator[](size_t index) const;

    auto
    begin()
    {
        return path.begin();
    }
    auto
    end()
    {
        return path.end();
    }
    auto
    begin() const
    {
        return path.begin();
    }
    auto
    end() const
    {
        return path.end();
    }
};
