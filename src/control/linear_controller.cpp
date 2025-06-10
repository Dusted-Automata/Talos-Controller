#include "linear_controller.hpp"
#include "cppmap3d.hh"
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

double
Linear_Controller::get_accel()
{
    return linear_profile.acceleration;
}

Velocity2d
Linear_Controller::get_cmd()
{
    double dt = 1.0 / robot->config.control_loop_hz; // TODO change with real dt
    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

    if (robot->sensor_manager.get_latest(Sensor_Name::UBLOX).has_value()) {
        Measurement measurement = robot->sensor_manager.get_latest(Sensor_Name::UBLOX).value();
        double lat = to_radian(measurement.ublox.latlng.lat);
        double lng = to_radian(measurement.ublox.latlng.lng);
        double alt = measurement.ublox.alt;

        if (above_epsilon(lat, lng, alt)) {
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            robot->frames.update_based_on_measurement({ lat, lng, alt });
        }
        robot->sensor_manager.consume_measurement();
    }

    std::optional<Ecef> target_waypoint = robot->path.get_next();
    if (!target_waypoint.has_value()) {
        linear_pid.reset();
        angular_pid.reset();
        linear_profile.reset();
        angular_profile.reset();
        return cmd;
    }
    ENU goal = cppmap3d::ecef2enu(target_waypoint.value(), robot->frames.local_frame.origin);
    Vector3d diff = goal.raw() - robot->frames.local_frame.pos.raw();
    diff = robot->frames.local_frame.orientation.rotation().transpose() * diff;
    double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());

    double yaw_error = atan2(diff.y(), diff.x());

    if (dist < goal_tolerance) {
        robot->path.pop();
        linear_pid.reset();
        angular_pid.reset();

        linear_profile.reset();
        angular_profile.reset();

        std::optional<Ecef> target_waypoint = robot->path.get_next();
        if (target_waypoint.has_value()) {
            ENU goal = cppmap3d::ecef2enu(target_waypoint.value(), robot->frames.local_frame.origin);
            Vector3d diff = goal.raw() - robot->frames.local_frame.pos.raw();
            diff = robot->frames.local_frame.orientation.rotation().transpose() * diff;
            double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
            linear_profile.set_setpoint(dist);
            angular_profile.set_setpoint(0);
        }
        return cmd;
    }

    std::cout << dist << std::endl;
    linear_profile.update(dist - goal_tolerance, dt);
    // angular_profile.update(yaw_error, dt);

    cmd.linear.x() = linear_pid.update(linear_profile.velocity, robot->pose_state.velocity.linear.x(), dt);
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    // cmd.angular.z() = angular_pid.update(angular_profile.velocity, robot->pose_state.velocity.angular.z(), dt);
    cmd.angular.z() = angular_pid.update(0, -yaw_error, dt);
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;
    return cmd;
}
