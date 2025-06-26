#include "robot_path.hpp"
#include "EGM96.hpp"
#include "cppmap3d.hh"
#include "json.hpp"
#include <fstream>
#include <iostream>

void
Robot_Path::add_waypoints(const std::vector<Pose> &waypoints)
{
    for (const Pose &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << std::fixed;
        std::cout << waypoint.point.raw().transpose() << std::endl;
        path.push_back(waypoint);
    }
};

std::optional<Pose>
Robot_Path::current()
{
    std::cout << current_index << " | " << path.size() << std::endl;
    if (current_index >= path.size()) {
        return std::nullopt;
    }

    return path[current_index];
}

std::optional<Pose>
Robot_Path::next()
{
    if (goal_index >= path.size()) {
        return std::nullopt;
    }

    return path[goal_index];
}

void
Robot_Path::progress()
{
    if (path.empty()) {
        return;
    }
    if (path_looping) {
        current_index = (current_index + 1) % path.size();
        goal_index = (goal_index + 1) % path.size();
        std::cout << "LOOPING!" << std::endl;
    } else if (current_index >= path.size() && goal_index >= path.size()) {
        return;
    }
    current_index++;
    goal_index++;
}

void
Robot_Path::reset()
{
    current_index = 0;
    goal_index = 1;
}

size_t
Robot_Path::size()
{
    return path.size();
}

Pose &
Robot_Path::operator[](size_t index)
{
    return path[index];
}

const Pose &
Robot_Path::operator[](size_t index) const
{
    return path[index];
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
    std::vector<Pose> waypoints;
    for (auto point : data["points"]) {
        Pose pose;
        llh.lat() = to_radian(point["lat"]);
        llh.lon() = to_radian(point["lon"]);
        llh.alt() = point["alt"];
        double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        llh.alt() += offset;
        Ecef ecef = cppmap3d::geodetic2ecef(llh);
        pose.point = ecef;
        waypoints.push_back(pose);
        std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;
    }
    add_waypoints(waypoints);
    return 0;
}
