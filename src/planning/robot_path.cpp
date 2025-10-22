#include "robot_path.hpp"
#include "cppmap3d.hh"
#include "json.hpp"
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

Pose
Robot_Path::next_stop()
{
    return path[stop_index]; // TODO: Might not have a stop in the vector
}


f64 Robot_Path::calculate_distance(int waypoint_index, int stop_index) {
    f64 distance = 0;
    for (int i = waypoint_index; i < stoppings[stop_index]; ++i) {
        distance += distance_to_next_waypoint[i];
    }
    return distance;

}

bool
Robot_Path::progress(const Path_Direction &dir)
{
    if (path.empty()) {
        return false;
    }
    switch (dir) {
    case Path_Direction::NORMAL:
        if (current_index >= path.size() && goal_index >= path.size()) {
            return false;
        }
        if (current_index < path.size() - 1) {
            ++current_index;
        }
        if (goal_index < path.size() - 1) {
            ++goal_index;
        }
        return true;
    case Path_Direction::LOOP:
        current_index = (++current_index) % path.size();
        goal_index = (++goal_index) % path.size();
        if (current_index == 0) {
            std::cout << "LOOPING!" << std::endl;
        }
        return true;
    case Path_Direction::REVERSE:
        if (!reverse_forward) {
            if (current_index < path.size() - 1) {
                ++current_index;
            } else {
                reverse_forward = true; // Switch to backward
            }
            if (goal_index < path.size() - 1) {
                ++goal_index;
            }
        } else {
            if (current_index > 0) {
                --current_index;
            } else {
                reverse_forward = false; // Switch to forward
            }
            if (goal_index > 0) {
                --goal_index;
            }
        }
        return true;
    }
    return false;
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
    // std::string msg = std::format("Index: C{} | G{} - Path_Looping : ", current_index, goal_index, path_looping);
    // std::cout << msg << std::endl;
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << path[i].point.raw().transpose() << " | " << path[i].local_point.raw().transpose() << std::endl;
    }
    std::cout << "-----------" << std::endl;
}

using json = nlohmann::json;
bool
Robot_Path::read_json_latlon(std::filesystem::path file_path)
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
    if (point.contains("lat")) {

        llh_origin.lat() = to_radian(point["lat"]);
        llh_origin.lon() = to_radian(point["lon"]);
        llh_origin.alt() = point["alt"];
        // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        // llh_origin.alt() += offset;

        std::vector<Pose> waypoints;
        // for (auto point : data["points"]) {
        for (size_t i = 0; i < data["points"].size(); i++){
            point = data["points"][i];
            Pose pose;
            llh.lat() = to_radian(point["lat"]);
            llh.lon() = to_radian(point["lon"]);
            llh.alt() = point["alt"];
            // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
            // llh.alt() += offset;
            std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;

            Ecef ecef = cppmap3d::geodetic2ecef(llh);
            pose.point = ecef;

            ENU local = cppmap3d::ecef2enu(ecef, llh_origin);
            pose.local_point = local;

            waypoints.push_back(pose);
            if (point["stop"] == true) {
                printf("stop at index: %zu\n", i);
                stoppings.push_back(i);
            }
        }
        add_waypoints(waypoints);
    } else {
        auto x = point["x"];
        auto y = point["y"];
        auto z = point["z"];
        llh_origin = cppmap3d::ecef2geodetic({ x, y, z });

        std::vector<Pose> waypoints;
        for (auto point : data["points"]) {
            Pose pose;
            auto x = point["x"];
            auto y = point["y"];
            auto z = point["z"];

            Ecef ecef = { x, y, z };
            pose.point = ecef;

            ENU local = cppmap3d::ecef2enu(ecef, llh_origin);
            pose.local_point = local;

            waypoints.push_back(pose);
        }
        add_waypoints(waypoints);
    }

    for (size_t i = 0; i < path.size()-1; i++) {
        f64 distance = (path[i].local_point - path[i+1].local_point).norm();
        distance_to_next_waypoint.push_back(distance);
        printf("calced distance %f\n",distance);
    }
    return 0;
}

Path
read_json_latlon(std::filesystem::path file_path)
{
    Path path;
    std::ifstream file(file_path);

    if (!file) {
        std::cerr << "Error opening file: " << file_path << std::endl;
        return path;
    }
    json data = json::parse(file);
    LLH llh, llh_origin;

    // Get origin LLH, to be able to compute ENU differences
    auto point = data["points"].front();
    if (point.contains("lat")) {

        llh_origin.lat() = to_radian(point["lat"]);
        llh_origin.lon() = to_radian(point["lon"]);
        llh_origin.alt() = point["alt"];
        // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
        // llh_origin.alt() += offset;

        for (size_t i = 0; i < data["points"].size(); i++){
            point = data["points"][i];
            Pose pose;
            llh.lat() = to_radian(point["lat"]);
            llh.lon() = to_radian(point["lon"]);
            llh.alt() = point["alt"];
            // double offset = egm96_compute_altitude_offset(llh.lat(), llh.lon());
            // llh.alt() += offset;
            std::cout << "lat: " << llh.lat() << " long: " << llh.lon() << " alt: " << llh.alt() << std::endl;

            Ecef ecef = cppmap3d::geodetic2ecef(llh);
            pose.point = ecef;

            ENU local = cppmap3d::ecef2enu(ecef, llh_origin);
            pose.local_point = local;

            if (point["stop"] == true) {
                printf("stop at index: %zu\n", i);
                path.add_waypoint(pose, true);
            } else {
                path.add_waypoint(pose, false);
            }
        }
    } 
    return path;
}
