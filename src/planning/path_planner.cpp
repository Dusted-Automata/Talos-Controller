#include "path_planner.hpp"
#include <iostream>

inline double
dist(const Pose &a, const Pose &b)
{
    double x = std::pow((b.local_point.east() - a.local_point.east()), 2);
    double y = std::pow((b.local_point.north() - a.local_point.north()), 2);
    double z = std::pow((b.local_point.up() - a.local_point.up()), 2);
    double val = std::sqrt(x + y + z);
    return val;
};

void
Path_Planner::gen_local_path(double meters_per_point)
{
    if (global_path.size() == 0) {
        return;
    }

    Pose prev_point = global_path[0];
    for (size_t i = 0; i < global_path.size(); i++) {
        double d = dist(prev_point, global_path[i]);
        int d_count = (int)(d / meters_per_point);
        Vector3d normalized = (global_path[i].local_point - prev_point.local_point) * (1.0 / d);
        for (int i = 1; i <= d_count; i++) {
            Pose i_point = { .local_point = prev_point.local_point + (normalized * i * meters_per_point) };
            local_path.add_waypoint(i_point);
        }
        local_path.add_waypoint(global_path[i]);
        prev_point = global_path[i];
    }

    if (global_path.path_looping) {
        double d = dist(prev_point, global_path[0]);
        int d_count = (int)(d / meters_per_point);
        Vector3d normalized = (global_path[0].local_point - prev_point.local_point) * (1.0 / d);
        for (int i = 1; i <= d_count; i++) {
            Pose i_point = { .local_point = prev_point.local_point + (normalized * i * meters_per_point) };
            local_path.add_waypoint(i_point);
        }
        local_path.add_waypoint(global_path[0]);
    }

    global_path.reset();
    // path.print();
    // global_path.print();
}
