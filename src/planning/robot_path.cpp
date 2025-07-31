#include "robot_path.hpp"
#include "EGM96.hpp"
#include "cppmap3d.hh"
#include "json.hpp"
#include <format>
#include <fstream>
#include <iostream>

void
Robot_Path::add_waypoint(const Pose waypoint)
{
    path.push_back(waypoint);
};

void
Robot_Path::add_waypoints(const std::vector<Pose> &waypoints)
{
    for (const Pose &waypoint : waypoints) {
        std::cout << "Adding Waypoints!" << std::endl;
        std::cout << std::fixed;
        std::cout << waypoint.point.raw().transpose() << std::endl;
        std::cout << waypoint.local_point.raw().transpose() << std::endl;
        path.push_back(waypoint);
    }
};

Pose
Robot_Path::current()
{
    return path[current_index];
}

Pose
Robot_Path::next()
{
    return path[goal_index];
}

bool
Robot_Path::progress()
{
    if (path.empty()) {
        return false;
    }
    if (!path_looping && current_index >= path.size() && goal_index >= path.size()) {
        return false;
    }

    if (path_looping) {
        current_index = (++current_index) % path.size();
        goal_index = (++goal_index) % path.size();
        if (current_index == 0) {
            std::cout << "LOOPING!" << std::endl;
        }

    } else {
        if (current_index < path.size() - 1) {
            ++current_index;
        }
        if (goal_index < path.size() - 1) {
            ++goal_index;
        }
    }

    return true;
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

void
Robot_Path::print()
{
    std::cout << "------------" << std::endl;
    std::string msg = std::format("Index: C{} | G{} - Path_Looping : ", current_index, goal_index, path_looping);
    std::cout << msg << std::endl;
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << path[i].point.raw().transpose() << " | " << path[i].local_point.raw().transpose() << std::endl;
    }
    std::cout << "-----------" << std::endl;
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
    LLH llh, llh_origin;

    // Get origin LLH, to be able to compute ENU differences
    auto point = data["points"].front();
    llh_origin.lat() = to_radian(point["lat"]);
    llh_origin.lon() = to_radian(point["lon"]);
    llh_origin.alt() = point["alt"];
    double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
    llh_origin.alt() += offset;

    std::vector<Pose> waypoints;
    for (auto point : data["points"]) {
        Pose pose;
        llh.lat() = to_radian(point["lat"]);
        llh.lon() = to_radian(point["lon"]);
        llh.alt() = point["alt"];
        double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        llh.alt() += offset;
        std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;

        Ecef ecef = cppmap3d::geodetic2ecef(llh);
        pose.point = ecef;

        ENU local = cppmap3d::ecef2enu(ecef, llh_origin);
        pose.local_point = local;

        waypoints.push_back(pose);
    }
    add_waypoints(waypoints);
    return 0;
}
