#include "pid.cpp"
#include "robot.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using Eigen::Affine3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;
typedef Vector3d Linear_Velocity;
typedef Vector3d Angular_Velocity;

struct Pose {
  Ecef_Coord point;
  Affine3d transformation_matrix; // Change this to Quaternion maybe
};

// struct Angular_Velocity {
//   double roll, pitch, yaw;
// };
// struct Linear_Velocity {
//   double forward, lateral, vertical;
// };
struct Velocity2d {
  Linear_Velocity linear;   // (m/s)
  Angular_Velocity angular; // (rad/s)
};

struct Motion_Constraints {
  double max_velocity;
  double min_velocity;
  double standing_turn_velocity;
  double max_acceleration;
  double max_deceleration;
  double max_jerk;
  double corner_velocity;
};

struct Velocity_Profile {
  double time_to_max_speed;
  double time_to_min_speed;
  double corner_velocity;
  double standing_turn_velocity;
  double acceleration_rate;
  double deceleration_rate;
};

struct Robot_Config {
  int hz;
  Motion_Constraints motion_constraints;
  // TODO: Make a real frame and transformation data structure
  // Matrix4d robot_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0,
  // 1}}; Matrix4d world_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0,
  // 0, 1}};
  Affine3d transform_world_to_robot = Affine3d::Identity();
};

struct Trajectory_Point {
  Pose pose;
  double dt;
  Velocity2d velocity;
};

// Pose cubic_interpolation(double dx, double dy, Ecef_Coord start, Ecef_Coord
// end,
//                          double t) {
//   double h00 = 2 * t * t * t - 3 * t * t + 1;
//   double h10 = t * t * t - 2 * t * t + t;
//   double h01 = -2 * t * t * t + 3 * t * t;
//   double h11 = t * t * t - t * t;
//
//   Pose point = {};
//   point.x = h00 * start.x + h10 * dx + h01 * end.x + h11 * dx;
//   point.y = h00 * start.y + h10 * dy + h01 * end.y + h11 * dy;
//   return point;
// };

double compute_displacement(double init_velocity, double final_velocity,
                            double acceleration) {
  double v0 = (init_velocity * init_velocity);
  double vf = (final_velocity * final_velocity);
  double avg_accel = (2 * acceleration);
  double displacement = (vf - v0) / avg_accel;
  return std::abs(displacement);
}

struct Motion_Step {
  Ecef_Coord &current;
  Ecef_Coord &next;
  Ecef_Coord &difference;
  Affine3d &robot_frame;
  double &dt;
};

class Trajectory_Controller {

  Motion_Constraints motion_constraints;
  Velocity_Profile velocity_profile;
  // PIDController linear_pid;
  // PIDController angular_pid;
  double sampling_rate = 1.0;

  Trajectory_Point trajectory_turn(Motion_Step step) {
    Ecef_Coord difference = step.next - step.current;
    double azimuth_rad_world = atan2(step.difference.y(), step.difference.x());
    double azimuth_rad_robot =
        atan2(step.robot_frame(1, 0), step.robot_frame(0, 0));
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
    angular.z() = motion_constraints.standing_turn_velocity;
    step.robot_frame.rotate(
        Eigen::AngleAxisd((azimuth_rad), Vector3d::UnitZ()));
    Velocity2d velocity = {.linear = {}, .angular = angular};
    Affine3d transformation = step.robot_frame;
    Pose pose = {.point = step.current,
                 .transformation_matrix = transformation};
    double turn_duration =
        azimuth_rad / motion_constraints.standing_turn_velocity;
    step.dt += std::abs(turn_duration);
    Trajectory_Point tp = {.pose = pose, .dt = step.dt, .velocity = velocity};
    return tp;
  }

  std::vector<Trajectory_Point> trajectory_ramp_up(Motion_Step step) {
    // TODO:
    // Not quite correct. I don't know if I will even reach max_velocity
    std::vector<Trajectory_Point> trajectory;
    double distance_acceleration = compute_displacement(
        0, motion_constraints.max_velocity, velocity_profile.acceleration_rate);
    double time_accelerating =
        motion_constraints.max_velocity / velocity_profile.acceleration_rate;

    Vector3d unit_vector = step.difference.normalized();

    for (int j = 0; j <= sampling_rate; j++) {
      double t = static_cast<double>(j) / sampling_rate;
      Ecef_Coord next_point =
          step.current + ((t * distance_acceleration) * unit_vector);
      Vector3d linear((motion_constraints.max_velocity * t), 0.0, 0.0);
      step.dt += (time_accelerating * t);
      Trajectory_Point tp = new_trajectory_point(
          step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
      trajectory.push_back(tp);
    }
    return trajectory;
  }

  Trajectory_Point trajectory_cruise(Motion_Step step) {
    Vector3d unit_vector = step.difference.normalized();
    double distance_acceleration = compute_displacement(
        0, motion_constraints.max_velocity, velocity_profile.acceleration_rate);
    double distance_deceleration = compute_displacement(
        motion_constraints.max_velocity, 0, velocity_profile.acceleration_rate);
    double distance_cruising = step.difference.norm() -
                               (distance_acceleration + distance_deceleration);

    // std::cout << distance_acceleration << std::endl;
    double time_cruising = distance_cruising / motion_constraints.max_velocity;

    Ecef_Coord next_point = step.next - (distance_deceleration * unit_vector);
    Vector3d linear((motion_constraints.max_velocity), 0.0, 0.0);
    step.dt += time_cruising;
    Trajectory_Point tp = new_trajectory_point(
        step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
    return tp;
  }

  std::vector<Trajectory_Point> trajectory_ramp_down(Motion_Step step) {

    std::vector<Trajectory_Point> trajectory;
    double distance_acceleration = compute_displacement(
        0, motion_constraints.max_velocity, velocity_profile.acceleration_rate);

    double distance_deceleration = compute_displacement(
        motion_constraints.max_velocity, 0, velocity_profile.acceleration_rate);
    double time_accelerating;
    time_accelerating =
        motion_constraints.max_velocity / velocity_profile.acceleration_rate;
    double time_decelerating;

    Vector3d unit_vector = step.difference.normalized();

    // time_decelerating t = -max_velocity / -acceleartion_rate
    // - acceleration is deceleration, but i am unsure if i should codify
    // that like that.
    time_decelerating =
        motion_constraints.max_velocity / velocity_profile.deceleration_rate;
    for (int j = 0; j <= sampling_rate; j++) {
      double t = static_cast<double>(j) / sampling_rate;
      Ecef_Coord next_point =
          step.current + ((t * distance_deceleration) * unit_vector);
      Vector3d linear((motion_constraints.max_velocity * (1 - t)), 0.0, 0.0);
      step.dt += (time_accelerating * t);
      Trajectory_Point tp = new_trajectory_point(
          step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
      trajectory.push_back(tp);
    }
    return trajectory;
  }

  Trajectory_Point new_trajectory_point(Affine3d &robot_frame,
                                        Ecef_Coord next_point, Vector3d linear,
                                        Vector3d angular, double dt) {

    Ecef_Coord diff_vec = next_point - robot_frame.translation();
    robot_frame.translation() += diff_vec;
    Velocity2d velocity = {.linear = linear, .angular = angular};
    Affine3d transformation = robot_frame;
    Pose pose = {.point = next_point, .transformation_matrix = transformation};
    Trajectory_Point tp = {.pose = pose, .dt = dt, .velocity = velocity};
    return tp;
  }

public:
  // Trajectory_Controller(Motion_Constraints motion_constraints,
  //                       Velocity_Profile velocity_profile,
  //                       PIDController linear_pid, PIDController angular_pid,
  //                       double sampling_rate)
  //     : motion_constraints(motion_constraints),
  //       velocity_profile(velocity_profile), linear_pid(linear_pid),
  //       angular_pid(angular_pid), sampling_rate(sampling_rate) {};
  Trajectory_Controller(Motion_Constraints motion_constraints,
                        Velocity_Profile velocity_profile, double sampling_rate)
      : motion_constraints(motion_constraints),
        velocity_profile(velocity_profile), sampling_rate(sampling_rate) {};

  std::vector<Trajectory_Point> generate_trajectory(Ecef_Coord current,
                                                    Ecef_Coord next,
                                                    Robot_Config &config) {

    std::vector<Trajectory_Point> trajectory = {};
    Affine3d &robot_frame = config.transform_world_to_robot;

    double dt = 0.0;
    Ecef_Coord difference = next - current;
    Motion_Step step = {.current = current,
                        .next = next,
                        .difference = difference,
                        .robot_frame = robot_frame,
                        .dt = dt};

    { // Turning
      Trajectory_Point tp = trajectory_turn(step);
      trajectory.push_back(tp);
    }

    { // RAMP UP
      std::vector<Trajectory_Point> ramp_up = trajectory_ramp_up(step);
      trajectory.insert(trajectory.end(), ramp_up.begin(), ramp_up.end());
    }

    { // CRUISE
      // trajectory_cruise()
      Trajectory_Point tp = trajectory_cruise(step);
      trajectory.push_back(tp);
    }
    step.current = trajectory.back().pose.point;
    { // RAMP DOWN
      std::vector<Trajectory_Point> ramp_down = trajectory_ramp_down(step);
      trajectory.insert(trajectory.end(), ramp_down.begin(), ramp_down.end());
    }
    return trajectory;
  }

  Pose get_current_pose() { return {}; }
  void follow_trajectory() {
    // In here I go through all poses with velocities and correct the velocities
    // by PID usage
  }
  void sendVelocityCommand(double linear_vel, double angular_vel) {
    // In actual implementation, this would interface with:
    // 1. Robot's motor controllers
    // 2. ROS cmd_vel topic
    // 3. Low-level motor driver
    std::cout << "Linear Velocity: " << linear_vel
              << " Angular Velocity: " << angular_vel << std::endl;
  }

  void local_replanning() {}
};

static bool saveToFile(const std::string &filename,
                       const std::vector<Ecef_Coord> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << line.x() << "," << line.y() << "," << line.z() << std::endl;
  }

  outFile.close();
  return true;
}

static bool saveToFile(const std::string &filename,
                       const std::vector<Trajectory_Point> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << std::fixed;
    outFile << " ------------------------------------------------- "
            << std::endl;
    outFile << "dt: " << line.dt << " " << std::endl;
    outFile << "pose: " << line.pose.point.x() << " " << line.pose.point.y()
            << " " << line.pose.point.z() << " " << std::endl;
    outFile << "Velocity-linear: forward: " << line.velocity.linear.x()
            << " lateral: " << line.velocity.linear.y()
            << " vertical: " << line.velocity.linear.z() << std::endl;
    outFile << "Velocity-angular: pitch: " << line.velocity.angular.x()
            << " roll: " << line.velocity.angular.y()
            << " yaw: " << line.velocity.angular.z() << std::endl;
    // outFile << " ------------------------------------------------- "
    // << std::endl;
  }

  outFile.close();
  return true;
}
