#include "trajectory.hpp"
#include "types.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

double compute_displacement(double init_velocity, double final_velocity, double acceleration)
{
    double v0 = (init_velocity * init_velocity);
    double vf = (final_velocity * final_velocity);
    double avg_accel = (2 * acceleration);
    double displacement = (vf - v0) / avg_accel;
    return std::abs(displacement);
}

Trajectory_Point Trajectory_Controller::trajectory_turn(Motion_Step step)
{
    Ecef_Coord difference = step.next - step.current;
    double azimuth_rad_world = atan2(step.difference.y(), step.difference.x());
    double azimuth_rad_robot = atan2(step.robot_frame(1, 0), step.robot_frame(0, 0));
    double azimuth_rad = azimuth_rad_world - azimuth_rad_robot;
    if (azimuth_rad > M_PI)
        azimuth_rad -= 2 * M_PI;
    if (azimuth_rad < -M_PI)
        azimuth_rad += 2 * M_PI;
    // std::cout << "azimuth_rad_world: " << azimuth_rad_world
    //           << " azimuth_rad_robot: " << azimuth_rad_robot
    //           << " azimuth_rad: " << azimuth_rad << std::endl;

    Angular_Velocity angular;
    angular.setZero();
    angular.z() = config.motion_constraints.standing_turn_velocity;
    step.robot_frame.rotate(Eigen::AngleAxisd((azimuth_rad), Vector3d::UnitZ()));
    Velocity2d velocity = {.linear = {}, .angular = angular};
    Affine3d transformation = step.robot_frame;
    Pose pose = {.point = step.current, .transformation_matrix = transformation};
    double turn_duration = azimuth_rad / config.motion_constraints.standing_turn_velocity;
    std::cout << azimuth_rad_world << std::endl;
    step.dt += std::abs(turn_duration);
    Trajectory_Point tp = {.pose = pose, .dt = step.dt, .velocity = velocity};
    return tp;
}

std::vector<Trajectory_Point> Trajectory_Controller::trajectory_ramp_up(Motion_Step step)
{
    // TODO:
    // Not quite correct. I don't know if I will even reach max_velocity
    std::vector<Trajectory_Point> trajectory;
    double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                        config.velocity_profile.acceleration_rate);
    double time_accelerating =
        config.motion_constraints.max_velocity / config.velocity_profile.acceleration_rate;

    Vector3d unit_vector = step.difference.normalized();

    for (int j = 0; j <= sampling_rate; j++)
    {
        double t = static_cast<double>(j) / sampling_rate;
        Ecef_Coord next_point = step.current + ((t * distance_acceleration) * unit_vector);
        Vector3d linear((config.motion_constraints.max_velocity * t), 0.0, 0.0);
        step.dt += (time_accelerating * t);
        Trajectory_Point tp =
            new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
        trajectory.push_back(tp);
    }
    return trajectory;
}

Trajectory_Point Trajectory_Controller::trajectory_cruise(Motion_Step step)
{
    Vector3d unit_vector = step.difference.normalized();
    double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                        config.velocity_profile.acceleration_rate);
    double distance_deceleration = compute_displacement(config.motion_constraints.max_velocity, 0,
                                                        config.velocity_profile.acceleration_rate);
    double distance_cruising =
        step.difference.norm() - (distance_acceleration + distance_deceleration);

    // std::cout << distance_acceleration << std::endl;
    double time_cruising = distance_cruising / config.motion_constraints.max_velocity;

    Ecef_Coord next_point = step.next - (distance_deceleration * unit_vector);
    Vector3d linear((config.motion_constraints.max_velocity), 0.0, 0.0);
    step.dt += time_cruising;
    Trajectory_Point tp =
        new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
    return tp;
}

std::vector<Trajectory_Point> Trajectory_Controller::trajectory_ramp_down(Motion_Step step)
{
    std::vector<Trajectory_Point> trajectory;
    double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                        config.velocity_profile.acceleration_rate);

    double distance_deceleration = compute_displacement(config.motion_constraints.max_velocity, 0,
                                                        config.velocity_profile.acceleration_rate);
    double time_accelerating;
    time_accelerating =
        config.motion_constraints.max_velocity / config.velocity_profile.acceleration_rate;
    double time_decelerating;

    Vector3d unit_vector = step.difference.normalized();

    // time_decelerating t = -max_velocity / -acceleartion_rate
    // - acceleration is deceleration, but i am unsure if i should codify
    // that like that.
    time_decelerating =
        config.motion_constraints.max_velocity / config.velocity_profile.deceleration_rate;
    for (int j = 0; j <= sampling_rate; j++)
    {
        double t = static_cast<double>(j) / sampling_rate;
        Ecef_Coord next_point = step.current + ((t * distance_deceleration) * unit_vector);
        Vector3d linear((config.motion_constraints.max_velocity * (1 - t)), 0.0, 0.0);
        step.dt += (time_accelerating * t);
        Trajectory_Point tp =
            new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
        trajectory.push_back(tp);
    }
    return trajectory;
}

Trajectory_Point Trajectory_Controller::new_trajectory_point(Affine3d &robot_frame,
                                                             Ecef_Coord next_point, Vector3d linear,
                                                             Vector3d angular, double dt)
{
    Ecef_Coord diff_vec = next_point - robot_frame.translation();
    robot_frame.translation() += diff_vec;
    Velocity2d velocity = {.linear = linear, .angular = angular};
    Affine3d transformation = robot_frame;
    Pose pose = {.point = next_point, .transformation_matrix = transformation};
    Trajectory_Point tp = {.pose = pose, .dt = dt, .velocity = velocity};
    return tp;
}

std::vector<Trajectory_Point>
Trajectory_Controller::generate_trajectory(Pose_State &state, Ecef_Coord current, Ecef_Coord next)
{
    std::vector<Trajectory_Point> trajectory = {};
    Affine3d &robot_frame = config.transform_world_to_robot;

    double dt = 0.0;
    Ecef_Coord difference = next - current;
    Motion_Step step = {.current = current,
                        .next = next,
                        .difference = difference,
                        .robot_frame = robot_frame,
                        .dt = dt};

    // { // Turning
    //     Trajectory_Point tp = trajectory_turn(step);
    //     trajectory.push_back(tp);
    // }
    //
    // { // RAMP UP
    //     std::vector<Trajectory_Point> ramp_up = trajectory_ramp_up(step);
    //     trajectory.insert(trajectory.end(), ramp_up.begin(), ramp_up.end());
    // }
    //
    // { // CRUISE
    //     Trajectory_Point tp = trajectory_cruise(step);
    //     trajectory.push_back(tp);
    // }
    // step.current = trajectory.back().pose.point;
    // { // RAMP DOWN
    //     std::vector<Trajectory_Point> ramp_down = trajectory_ramp_down(step);
    //     trajectory.insert(trajectory.end(), ramp_down.begin(), ramp_down.end());
    // }
    /*for (auto traj : trajectory)*/
    /*{*/
    /*    std::cout << traj.dt << std::endl;*/
    /*}*/
    return trajectory;
}

Pose Trajectory_Controller::get_current_pose() { return {}; }

Velocity2d Trajectory_Controller::follow_trajectory(Pose_State &state,
                                                    Thread_Safe_Queue<Ecef_Coord> &path_queue)
{

    Velocity2d cmd = {.linear = Linear_Velocity().setZero(),
                      .angular = Angular_Velocity().setZero()};

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = path_queue.front_two();
    if (trajectories.empty())
    {
        if (path.has_value())
        {

            linear_pid.reset();
            angular_pid.reset();
            trajectory_time = 0.0;

            std::cout << "EMPTY TRAJECTORIES" << std::endl;
            std::vector<Trajectory_Point> trajectory =
                generate_trajectory(state, path.value().first, path.value().second);
            for (auto &point : trajectory)
            {
                trajectories.push(point);
            }
            path_queue.pop();
        }
    }

    std::optional<Trajectory_Point> point = trajectories.front();

    if (!point)
    {
        return cmd;
    }

    // TODO: If the robot is still not done moving after the calculated dt, then
    // it should still move.
    if (point.value().dt < trajectory_time)
    {
        trajectories.pop();
        // unwrap
        point = trajectories.front();
        if (!point)
            return cmd;
        std::cout << "SETTING NEW SETPOINTS" << std::endl;
        linear_pid.setpoint = point.value().velocity.linear.x();
        angular_pid.setpoint = point.value().velocity.angular.z();
    }

    std::cout << "DT: " << point.value().dt << " | t_dt: " << trajectory_time
              << " lin: " << point.value().velocity.linear.x()
              << " ang: " << point.value().velocity.angular.z() << std::endl;

    cmd.linear.x() = linear_pid.update(state.velocity.linear.x(), trajectory_time);
    cmd.angular.z() = angular_pid.update(state.velocity.angular.z(), trajectory_time);

    // cmd.linear.x() = point.value().velocity.linear.x();
    // cmd.angular.z() = point.value().velocity.angular.z();
    trajectory_time += state.dt;
    return cmd;
}

// void Trajectory_Controller::trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &waypoints)
// {
//     std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = waypoints.front_two();
//     if (trajectories.empty())
//     {
//         if (path.has_value())
//         {
//             std::cout << "EMPTY TRAJECTORIES" << std::endl;
//             std::vector<Trajectory_Point> trajectory =
//                 generate_trajectory(path.value().first, path.value().second);
//             for (auto &point : trajectory)
//             {
//                 trajectories.push(point);
//             }
//             waypoints.pop();
//         }
//     }
// }

void Trajectory_Controller::path_loop(Thread_Safe_Queue<Ecef_Coord> &path,
                                      std::vector<Ecef_Coord> &waypoints)
{
    if (waypoints.empty())
    {
        return;
    }
    // std::cout << path.size() << "  WAYPOINTS" << std::endl;
    if (!added_paths)
    {
        std::cout << "EMPTY FIRST WAYPOINTS" << std::endl;
        for (Ecef_Coord &waypoint : waypoints)
        {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            path.push(waypoint);
        }
        added_paths = true;
    }

    // FIXME: One waypoint does not get popped off, so it wont loop
    if ((path.size() == 1) && path_looping)
    {
        std::cout << path.size() << " EMPTY WAYPOINTS" << std::endl;
        for (Ecef_Coord &waypoint : waypoints)
        {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            path.push(waypoint);
        }
    }
}

void Trajectory_Controller::local_replanning() {}

Velocity2d Trajectory_Controller::get_cmd(Pose_State &state,
                                          Thread_Safe_Queue<Ecef_Coord> &path_queue)
{
    double max_vel_x = 2.0;
    double min_vel_x = -2.0;
    double goal_yaw = 0;
    double rotate_dist_threshold = 0.1;
    int odom_waiting_count = 1;

    double yaw_tolerance = 20.0; // degrees
    double goal_tolerance = 0.3; // meters

    double proportional_gain_x = 0.8;
    double proportional_gain_yaw = 1.0;

    Velocity2d cmd = {.linear = Linear_Velocity().setZero(),
                      .angular = Angular_Velocity().setZero()};

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = path_queue.front_two();
    if (!path.has_value())
    {
        return cmd;
        // linear_pid.reset();
        // angular_pid.reset();
        // path_queue.pop();
    }
    Ecef_Coord goal = path.value().second;

    Ecef_Coord difference = path.value().second - path.value().first;

    double dx = goal.x() - state.position.x();
    double dy = goal.y() - state.position.y();
    double dist = sqrt(dx * dx + dy * dy);

    double yaw = atan2(state.orientation.rotation()(1, 0), state.orientation.rotation()(0, 0));

    double dx_odom = cos(yaw) * dx + sin(yaw) * dy;
    double dy_odom = -sin(yaw) * dx + cos(yaw) * dy;

    double vel_x = proportional_gain_x * dx_odom;
    vel_x = std::max(std::min(vel_x, max_vel_x), min_vel_x);
    double dyaw = atan2(dy_odom, dx_odom);
    if (dist < rotate_dist_threshold)
    {
        // vel_yaw = 0.0;
        dyaw = goal_yaw - yaw;
        if (dyaw > M_PI)
        {
            dyaw -= 2 * M_PI;
        }
        else if (dyaw < -M_PI)
        {
            dyaw += 2 * M_PI;
        }
    }

    double vel_yaw = proportional_gain_yaw * dyaw;

    if (dist < goal_tolerance && std::abs(dyaw) < yaw_tolerance * M_PI / 180.0)
    {
        path_queue.pop();
        return cmd;
        // goal_reached_msg.data = true;
    }
    else
    {
        // goal_reached_msg.data = false;
    }
    cmd.linear.x() = vel_x;
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = vel_yaw;
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    trajectory_time += state.dt;

    // Path_Movement path = readPath();
    // Thread_Safe_Queue<Trajectory_Point> trajectories = readPath();
    // Velocity2d cmd = follow_trajectory(state, path_queue);

    std::ofstream traj_file("trajectories", std::ios::app);
    traj_file << 1 << " " << 0 << " " << 0 << " " << state.position.x() << " ";
    traj_file << 0 << " " << 1 << " " << 0 << " " << state.position.y() << " ";
    traj_file << 0 << " " << 0 << " " << 1 << " " << state.position.z();
    traj_file << std::endl;
    std::ofstream time_file("time", std::ios::app);
    time_file << trajectory_time << std::endl;

    return cmd;
}

static bool saveToFile(const std::string &filename, const std::vector<Ecef_Coord> &data)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return false;
    }

    for (const auto &line : data)
    {
        outFile << line.x() << "," << line.y() << "," << line.z() << std::endl;
    }

    outFile.close();
    return true;
}

static bool saveToFile(const std::string &filename, const std::vector<Trajectory_Point> &data)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return false;
    }

    for (const auto &line : data)
    {
        outFile << std::fixed;
        outFile << " ------------------------------------------------- " << std::endl;
        outFile << "dt: " << line.dt << " " << std::endl;
        outFile << "pose: " << line.pose.point.x() << " " << line.pose.point.y() << " "
                << line.pose.point.z() << " " << std::endl;
        outFile << "Velocity-linear: forward: " << line.velocity.linear.x()
                << " lateral: " << line.velocity.linear.y()
                << " vertical: " << line.velocity.linear.z() << std::endl;
        outFile << "Velocity-angular: pitch: " << line.velocity.angular.x()
                << " roll: " << line.velocity.angular.y() << " yaw: " << line.velocity.angular.z()
                << std::endl;
        // outFile << " ------------------------------------------------- "
        // << std::endl;
    }

    outFile.close();
    return true;
}
