#include "robot_path.hpp"
#include "json.hpp"
#include <fstream>
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
Robot_Path::pop()
{
    queue.pop();
    if (queue.empty() && path_looping && !path_points_all.empty()) {
        std::cout << "looping!" << std::endl;
        for (Ecef &waypoint : path_points_all) {
            queue.push(waypoint);
        }
    }
}

using json = nlohmann::json;
bool
Robot_Path::read_json_latlon(std::string file_path)
{
    std::ifstream file(file_path);

    if (!file) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return 1;
    }
    json data = json::parse(file);
    double lat, lon;
    std::vector<Ecef> waypoints;
    for (auto point : data["points"]) {
        lat = point["lat"];
        lon = point["lon"];
        Ecef ecef;
        ecef.x() = point["x"];
        ecef.y() = point["y"];
        ecef.z() = point["z"];
        waypoints.push_back(ecef);
        std::cout << "lat: " << lat << " long: " << lon << std::endl;
    }
    add_waypoints(waypoints);
    return 0;
}
