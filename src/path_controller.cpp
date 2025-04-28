#include "path_controller.hpp"
#include <iostream>

void
Path_Controller::add_waypoints(std::vector<Ecef_Coord> &waypoints)
{
    if (waypoints.empty()) {
        return;
    }

    for (Ecef_Coord &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << waypoint.transpose() << std::endl;
        path_queue.push(waypoint);
        path_points_all.push_back(waypoint);
    }
};

void
Path_Controller::path_loop()
{
    if ((path_queue.size() == 1) && path_looping && !path_points_all.empty()) {
        path_queue.pop(); // clear out last path_point, so as not to acumulate garbage points.
        std::cout << path_queue.size() << " EMPTY WAYPOINTS" << std::endl;
        for (Ecef_Coord &waypoint : path_points_all) {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            path_queue.push(waypoint);
        }
    }
}

void
Path_Controller::start()
{

    std::function<void()> bound_path_loop = [this]() { path_loop(); };
    path_loop_thread = std::thread(worker_function, bound_path_loop, 0.030);
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
}
