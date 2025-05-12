#include "linear_controller.hpp"
#include "frames.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>

inline double
wrapAngle(double rad)
{
    rad = std::fmod(rad + M_PI, 2.0 * M_PI); // shift, mod 2π  → (0, 2π]
    if (rad < 0.0) rad += 2.0 * M_PI;        // keep it positive
    return rad - M_PI;                       // shift back → (−π, π]
}

Velocity2d
Linear_Controller::get_cmd()
{
    double goal_yaw = 0;
    double rotate_dist_threshold = 0.1;

    double yaw_tolerance = 20.0; // degrees
    double goal_tolerance = 0.3; // meters

    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };
    if (robot->sensor_manager.get_latest().has_value()) {
        double lat = robot->sensor_manager.latest_measurement.value().ublox_measurement.latlng.lat;
        double lng = robot->sensor_manager.latest_measurement.value().ublox_measurement.latlng.lng;
        double alt = robot->sensor_manager.latest_measurement.value().ublox_measurement.alt;

        if (lat != 0.0 || lng != 0.0 || alt != 0.0) {
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            robot->frames.update_based_on_measurement(lat, lng, alt);
        }
        robot->sensor_manager.latest_measurement.reset();
    }

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = robot->path_controller.front_two();
    if (!path.has_value()) {
        linear_pid.reset();
        angular_pid.reset();
        return cmd;
    }
    Ecef_Coord goal = wgsecef2ned_d(path.value().second, robot->frames.local_frame.origin);
    Vector3d diff = goal - robot->frames.local_frame.pos;
    double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());

    double yaw = atan2(robot->frames.local_frame.orientation.rotation()(1, 0),
        robot->frames.local_frame.orientation.rotation()(0, 0));

    diff = robot->frames.local_frame.orientation.rotation().transpose() * diff;

    double yaw_error;
    if (dist < rotate_dist_threshold) {
        yaw_error = wrapAngle(goal_yaw - yaw);
    } else {
        yaw_error = atan2(diff.y(), diff.x());
    }

    if (dist < goal_tolerance && std::abs(yaw_error) < yaw_tolerance * M_PI / 180.0) {
        robot->path_controller.goal_reached();
        linear_pid.reset();
        angular_pid.reset();
        return cmd;
    }

    cmd.linear.x() = linear_pid.update(-diff.x(), 0.002);
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = angular_pid.update(-yaw_error, 0.002);
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    return cmd;
}
