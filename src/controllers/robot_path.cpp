#include "robot_path.hpp"
#include <iostream>

void
Robot_Path::add_waypoints(const std::vector<Ecef> &waypoints)
{
    for (const Ecef &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << std::fixed;
        std::cout << waypoint.raw().transpose() << std::endl;
        queue.push(waypoint);
        path_points_all.push_back(waypoint);
    }
};

std::optional<Ecef>
Robot_Path::get_next()
{
    return queue.front();
}

void
Robot_Path::goal_reached()
{
    queue.pop();
    if (queue.empty() && path_looping && !path_points_all.empty()) {
        std::cout << "looping!" << std::endl;
        for (Ecef &waypoint : path_points_all) {
            queue.push(waypoint);
        }
    }
}
