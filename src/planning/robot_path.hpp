#pragma once
#include "types.hpp"

struct Path_Pose {
    Ecef point;
    ENU local_point;
    f32 heading;
    bool stop;
};

class Robot_Path
{

  private:
    std::vector<Pose> path;
    std::vector<i32> stoppings;
    std::vector<f64> distance_to_next_waypoint; // TODO: Any other way to do this?
    bool reverse_forward = false;

  public:
    size_t current_index = 0;
    size_t goal_index = 1;
    size_t stop_index = 0;
    Robot_Path() = default;

    bool path_looping = false;

    void add_waypoints(const std::vector<Pose> &waypoints);
    void add_waypoint(const Pose waypoint);
    Pose current();
    Pose next();
    Pose next_stop();
    bool progress(const Path_Direction &dir);
    bool read_json_latlon(std::filesystem::path file_path);
    size_t size();
    void reset();
    void print();

    f64 calculate_distance(int waypoint_index, int stop_index);

    Pose &operator[](size_t index);
    const Pose &operator[](size_t index) const;
};
