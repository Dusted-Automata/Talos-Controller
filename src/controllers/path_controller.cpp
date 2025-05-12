#include "path_controller.hpp"
#include <iostream>

void
Path_Controller::add_waypoints(const std::vector<Ecef_Coord> &waypoints)
{
    for (const Ecef_Coord &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << waypoint.transpose() << std::endl;
        path_queue.push(waypoint);
        path_points_all.push_back(waypoint);
    }
};

std::optional<std::pair<Ecef_Coord, Ecef_Coord>>
Path_Controller::front_two()
{
    return path_queue.front_two();
}

void
Path_Controller::goal_reached()
{
    path_queue.pop();
    if (path_looping && !path_points_all.empty()) {
        for (Ecef_Coord &waypoint : path_points_all) {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            path_queue.push(waypoint);
        }
    }
}
