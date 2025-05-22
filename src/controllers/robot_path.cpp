#include "robot_path.hpp"
#include "EGM96.hpp"
#include "cppmap3d.hh"
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
    LLH llh;
    std::vector<Ecef> waypoints;
    for (auto point : data["points"]) {
        llh.lat() = to_radian(point["lat"]);
        llh.lon() = to_radian(point["lon"]);
        llh.alt() = point["alt"];
        double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        llh.alt() += offset;
        Ecef ecef = cppmap3d::geodetic2ecef(llh);
        waypoints.push_back(ecef);
        std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;
    }
    add_waypoints(waypoints);
    return 0;
}
