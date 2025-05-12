#include "robot_path.hpp"
#include <iostream>

void
Robot_Path::add_waypoints(const std::vector<Ecef_Coord> &waypoints)
{
    for (const Ecef_Coord &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << waypoint.transpose() << std::endl;
        queue.push(waypoint);
        path_points_all.push_back(waypoint);
    }
};

std::optional<Ecef_Coord>
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
        for (Ecef_Coord &waypoint : path_points_all) {
            queue.push(waypoint);
        }
    }
}
