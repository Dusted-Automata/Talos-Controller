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
        path.push_back(waypoint);
    }
};

std::optional<Pose>
Robot_Path::current()
{
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
    if (!path_looping && current_index >= path.size() && goal_index >= path.size()) {
        return;
    }
    current_index++;
    goal_index++;
    if (path_looping && (current_index >= path.size() || goal_index >= path.size())) {
        current_index = current_index % path.size();
        goal_index = goal_index % path.size();
        std::cout << "LOOPING!" << std::endl;
    }
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
    for (Pose &point : path) {
        std::cout << point.point.raw().transpose() << std::endl;
    }
    std::cout << "-----------" << std::endl;
}

using json = nlohmann::json;
bool
Robot_Path::read_json_latlon(std::string file_path)
{
    int llh_height_TMP = 241;
    std::ifstream file(file_path);

    if (!file) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return 1;
    }
    json data = json::parse(file);
    LLH llh, llh_origin;
    auto point = data["points"].front();
    llh_origin.lat() = to_radian(point["lat"]);
    llh_origin.lon() = to_radian(point["lon"]);
    llh_origin.alt() = point["alt"];
    double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
    llh_origin.alt() -= offset;

    std::vector<Pose> waypoints;
    for (auto point : data["points"]) {
        Pose pose;
        llh.lat() = to_radian(point["lat"]);
        llh.lon() = to_radian(point["lon"]);
        llh.alt() = point["alt"];
        double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        // llh.alt() += offset;
        llh.alt() = llh_height_TMP;
        std::cout << offset << std::endl;
        Ecef ecef = cppmap3d::geodetic2ecef(llh);
        pose.point = ecef;
        waypoints.push_back(pose);
        std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;

        ENU enu_test = cppmap3d::ecef2enu(ecef, llh_origin);
        Ecef ecef_test2 = cppmap3d::enu2ecef(enu_test, llh_origin);
        std::cout << std::fixed;
        std::cout << " --- " << std::endl;
        std::cout << "ecef: " << ecef.raw().transpose() << std::endl;
        std::cout << "ecef: " << enu_test.raw().transpose() << std::endl;
        std::cout << "ecef: " << ecef_test2.raw().transpose() << std::endl;
        std::cout << " --- " << std::endl;
    }
    add_waypoints(waypoints);
    return 0;
}
