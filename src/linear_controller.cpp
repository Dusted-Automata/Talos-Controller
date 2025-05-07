#include "linear_controller.hpp"
#include "cppmap3d.hh"
#include "frame_controller.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>

Velocity2d
Linear_Controller::get_cmd()
{
    double max_vel_x = 2.0;
    double min_vel_x = -2.0;
    double goal_yaw = 0;
    double rotate_dist_threshold = 0.1;

    double yaw_tolerance = 20.0; // degrees
    double goal_tolerance = 0.3; // meters

    double proportional_gain_x = 0.8;
    double proportional_gain_yaw = 1.0;

    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };
    if (!robot->sensor_manager.latest_measurement.read) {
        robot->sensor_manager.latest_measurement.read = true;
        double lat = robot->sensor_manager.latest_measurement.ublox_measurement.latlng.lat;
        double lng = robot->sensor_manager.latest_measurement.ublox_measurement.latlng.lng;
        double alt = robot->sensor_manager.latest_measurement.ublox_measurement.alt;

        if (lat != 0.0 || lng != 0.0 || alt != 0.0) {
            // Vector3d error_vec = robot->frame_controller.get_error_vector_in_NED(lat, lng, alt);
            robot->frame_controller.update_based_on_measurement(lat, lng, alt);
        }
    }

#if 1
    if ((testCounter % 5000) == 0) {
        Vector3d push = { 1.0, 2.0, 0.0 };
        Vector3d fake_measurement = robot->frame_controller.local_frame.pos + push;
        Ecef_Coord ecef = wgsned2ecef_d(fake_measurement, robot->frame_controller.local_frame.origin);
        LLH llh = wgsecef2llh(ecef);
        LLH llh_you;
        cppmap3d::internal::ecef2geodetic_you(ecef[0], ecef[1], ecef[2], llh_you[0], llh_you[1], llh_you[2]);
        LLH llh_olson;
        cppmap3d::internal::ecef2geodetic_olson(ecef[0], ecef[1], ecef[2], llh_olson[0], llh_olson[1], llh_olson[2]);
        std::cout << "DEF: " << "lat: " << llh[0] << " lng: " << llh[1] << " alt: " << llh[2] << std::endl;
        std::cout << "YOU: " << "lat: " << llh_you[0] << " lng: " << llh_you[1] << " alt: " << llh_you[2] << std::endl;
        std::cout << "OLS: " << "lat: " << llh_olson[0] << " lng: " << llh_olson[1] << " alt: " << llh_olson[2]
                  << std::endl;
        std::cout << std::fixed;
        std::cout << "BASE: " << robot->frame_controller.global_frame.pos.transpose() << std::endl;
        robot->frame_controller.update_based_on_measurement(llh[0], llh[1], llh[2]);
    }
    testCounter++;
#endif

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = robot->path_controller.front_two();
    if (!path.has_value()) {
        return cmd;
        // linear_pid.reset();
        // angular_pid.reset();
    }
    Ecef_Coord goal = wgsecef2ned_d(path.value().second, robot->frame_controller.local_frame.origin);

    double dx = goal.x() - robot->frame_controller.local_frame.pos.x();
    double dy = goal.y() - robot->frame_controller.local_frame.pos.y();
    // double dz = goal.z() - robot->frame_controller.local_frame.pos.z();
    double dist = sqrt(dx * dx + dy * dy);

    double yaw = atan2(robot->frame_controller.local_frame.orientation.rotation()(1, 0),
        robot->frame_controller.local_frame.orientation.rotation()(0, 0));

    double dx_odom = cos(yaw) * dx + sin(yaw) * dy;
    double dy_odom = -sin(yaw) * dx + cos(yaw) * dy;

    double vel_x = proportional_gain_x * dx_odom;
    vel_x = std::max(std::min(vel_x, max_vel_x), min_vel_x);
    double dyaw = atan2(dy_odom, dx_odom);
    if (dist < rotate_dist_threshold) {
        // vel_yaw = 0.0;
        dyaw = goal_yaw - yaw;
        if (dyaw > M_PI) {
            dyaw -= 2 * M_PI;
        } else if (dyaw < -M_PI) {
            dyaw += 2 * M_PI;
        }
    }

    double vel_yaw = proportional_gain_yaw * dyaw;

    if (dist < goal_tolerance && std::abs(dyaw) < yaw_tolerance * M_PI / 180.0) {
        robot->path_controller.goal_reached();
        return cmd;
        // goal_reached_msg.data = true;
    } else {
        // goal_reached_msg.data = false;
    }
    cmd.linear.x() = vel_x;
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = vel_yaw;
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    return cmd;
}
