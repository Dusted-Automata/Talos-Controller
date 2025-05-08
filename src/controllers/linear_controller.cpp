#include "linear_controller.hpp"
#include "frames.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>

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
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            robot->frames.update_based_on_measurement(lat, lng, alt);
        }
    }

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = robot->path_controller.front_two();
    if (!path.has_value()) {
        return cmd;
        // linear_pid.reset();
        // angular_pid.reset();
    }
    Ecef_Coord goal = wgsecef2ned_d(path.value().second, robot->frames.local_frame.origin);

    double dx = goal.x() - robot->frames.local_frame.pos.x();
    double dy = goal.y() - robot->frames.local_frame.pos.y();
    // double dz = goal.z() - robot->frames.local_frame.pos.z();
    double dist = sqrt(dx * dx + dy * dy);

    double yaw = atan2(robot->frames.local_frame.orientation.rotation()(1, 0),
        robot->frames.local_frame.orientation.rotation()(0, 0));

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
