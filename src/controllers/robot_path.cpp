#include "robot_path.hpp"
#include <iostream>

void
Robot_Path::add_waypoints(const std::vector<Ecef_Coord> &waypoints)
{
    for (const Ecef_Coord &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << waypoint.transpose() << std::endl;
        path_queue.push(waypoint);
        path_points_all.push_back(waypoint);
    }
};

std::optional<std::pair<Ecef_Coord, Ecef_Coord>>
Robot_Path::front_two()
{
    return path_queue.front_two();
}

void
Robot_Path::goal_reached()
{
    path_queue.pop();
    if (path_queue.empty() && path_looping && !path_points_all.empty()) {
        std::cout << "looping!" << std::endl;
        for (Ecef_Coord &waypoint : path_points_all) {
            path_queue.push(waypoint);
        }
    }
}
