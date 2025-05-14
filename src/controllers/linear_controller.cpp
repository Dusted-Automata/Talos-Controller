#include "linear_controller.hpp"
#include "frames.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>

bool
above_epsilon(double lat, double lng, double alt)
{
    if (std::abs(lat) > 0.001 || std::abs(lng) > 0.001 || std::abs(alt) > 0.001) {
        return true;
    }
    return false;
}

Velocity2d
Linear_Controller::get_cmd()
{
    double goal_tolerance = 0.3; // meters
    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

    if (robot->sensor_manager.get_latest(Sensor_Name::UBLOX).has_value()) {
        double lat = robot->sensor_manager.get_latest(Sensor_Name::UBLOX).value().ublox_measurement.latlng.lat;
        double lng = robot->sensor_manager.get_latest(Sensor_Name::UBLOX).value().ublox_measurement.latlng.lng;
        double alt = robot->sensor_manager.get_latest(Sensor_Name::UBLOX).value().ublox_measurement.alt;

        if (above_epsilon(lat, lng, alt)) {
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            // printf("LLH: %f, %f, %f\n", lat, lng, alt);
            robot->frames.update_based_on_measurement(lat, lng, alt);
        }
        robot->sensor_manager.consume_measurement();
    }

    std::optional<Ecef_Coord> target_waypoint = robot->path.get_next();
    if (!target_waypoint.has_value()) {
        linear_pid.reset();
        angular_pid.reset();
        return cmd;
    }
    Ecef_Coord goal = wgsecef2ned_d(target_waypoint.value(), robot->frames.local_frame.origin);
    Vector3d diff = goal - robot->frames.local_frame.pos;
    diff = robot->frames.local_frame.orientation.rotation().transpose() * diff;
    double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());

    double yaw_error = atan2(diff.y(), diff.x());

    if (dist < goal_tolerance) {
        robot->path.goal_reached();
        linear_pid.reset();
        angular_pid.reset();
        return cmd;
    }

    cmd.linear.x() = linear_pid.update(-diff.x(), 1.0 / robot->hz);
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = angular_pid.update(-yaw_error, 1.0 / robot->hz);
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    return cmd;
}
