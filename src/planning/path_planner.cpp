#include "path_planner.hpp"
#include <iostream>

inline double
dist(const Pose &a, const Pose &b)
{
    double x = std::pow((b.point.x() - a.point.x()), 2);
    double y = std::pow((b.point.y() - a.point.y()), 2);
    double z = std::pow((b.point.z() - a.point.z()), 2);
    double val = std::sqrt(x + y + z);
    return val;
};
void
Path_Planner::gen_global_path(double meters_per_point)
{
    if (path.size() == 0) {
        return;
    }

    Pose prev_point = path[0];
    // path.progress();
    // for (Pose &point : path) {
    for (size_t i = 0; i < path.size(); i++) {
        double d = dist(prev_point, path[i]);
        int d_count = (int)(d / meters_per_point);
        Vector3d normalized = (path[i].point - prev_point.point) * (1.0 / d);
        for (int i = 1; i <= d_count; i++) {
            Pose i_point = { .point = prev_point.point + (normalized * i * meters_per_point) };
            global_path.add_waypoint(i_point);
        }
        global_path.add_waypoint(path[i]);
        prev_point = path[i];
    }

    if (path.path_looping) {
        double d = dist(prev_point, path[0]);
        int d_count = (int)(d / meters_per_point);
        Vector3d normalized = (path[0].point - prev_point.point) * (1.0 / d);
        for (int i = 1; i <= d_count; i++) {
            Pose i_point = { .point = prev_point.point + (normalized * i * meters_per_point) };
            global_path.add_waypoint(i_point);
        }
        global_path.add_waypoint(path[0]);
    }

    path.reset();
    // path.print();
    // global_path.print();
}
