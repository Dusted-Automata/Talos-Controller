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

std::vector<Pose> generate_path(const std::vector<Ecef_Coord> &waypoints,
                                double points_per_meter) {
  Pose default_pose = {.point = {0.0, 0.0, 0.0},
                       .transformation_matrix = Affine3d::Identity()};
  std::vector<Pose> path;
  path.push_back(default_pose);

  for (int i = 0; i < waypoints.size() - 1; i++) {
    Ecef_Coord current = waypoints[i];
    Ecef_Coord next = waypoints[i + 1];
    Vector3d difference = next - current;
    double horizontal_distance = std::sqrt(difference.x() * difference.x() +
                                           difference.y() * difference.y());
    int points = static_cast<int>(horizontal_distance * points_per_meter);

    double azimuth_rad_world = atan2(difference.y(), difference.x());
    double azimuth_degrees = fmod(azimuth_rad_world / M_PI * 180, 360);
    double elevation = std::atan2(difference.z(), horizontal_distance);
    double elevation_degrees = fmod(elevation / M_PI * 180, 360);

    // for (int j = 0; j <= points; j++) {
    //   double t = static_cast<double>(j) / points;
    //   Ecef_Coord next_point = (current * (1 - t)) + (next * t);
    //   Ecef_Coord diff_vec =
    //       next_point - path[i + j].transformation_matrix.translation();
    //   Pose pose = {.point = next_point,
    //                .transformation_matrix = path[i +
    //                j].transformation_matrix};
    //   pose.transformation_matrix.translation() += diff_vec;
    //   path.push_back(pose);
    // }

    // TODO if I add back the loop with the points, then i need to change
    // path[i] to something that considers the newly added poses
    Pose pose = {.point = next,
                 .transformation_matrix = path[i].transformation_matrix};
    pose.transformation_matrix.translation() += difference;
    pose.transformation_matrix.rotate(
        Eigen::AngleAxisd((azimuth_rad_world), Vector3d::UnitZ()));
    path.push_back(pose);
  }
  return path;
}

class Trajectory_Controller {

  Motion_Constraints motion_constraints;
  Velocity_Profile velocity_profile;
  // PIDController linear_pid;
  // PIDController angular_pid;
  double sampling_rate = 1.0;

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
  std::vector<Trajectory_Point>
  generate_trajectory(const std::vector<Ecef_Coord> &path,
                      Robot_Config &config) {
    std::vector<Trajectory_Point> trajectory = {};
    Affine3d &robot_frame = config.transform_world_to_robot;
    if (path.size() < 2) {
      return trajectory;
    }

    for (int i = 0; i < path.size() - 1; i++) {
      Ecef_Coord current = path[i];
      Ecef_Coord next = path[i + 1];
      Ecef_Coord difference = next - current;
      double horizontal_distance = std::sqrt(difference.x() * difference.x() +
                                             difference.y() * difference.y());
      Vector3d unit_vector = difference.normalized();

      // TODO: fix me
      // This is not quite correct since it will always go to max velocity then.
      // trajectory_ramp_up();
      double distance_acceleration =
          compute_displacement(0, motion_constraints.max_velocity,
                               velocity_profile.acceleration_rate);
      Ecef_Coord next_goal = current;
      for (int j = 0; j <= sampling_rate; j++) {
        double t = static_cast<double>(j) / sampling_rate;
        Ecef_Coord next_point =
            current + ((t * distance_acceleration) * unit_vector);
        Vector3d linear((motion_constraints.max_velocity * t), 0.0, 0.0);
        // TODO:
        // need to add dt !!
        new_trajectory_point(trajectory, robot_frame, next_point, linear,
                             {0.0, 0.0, 0.0}, 0.0);
      }

      current = trajectory.back().pose.point;
      // CRUISE
      // trajectory_cruise()
      double distance_deceleration =
          compute_displacement(motion_constraints.max_velocity, 0,
                               velocity_profile.acceleration_rate);
      Ecef_Coord next_point = next + (distance_deceleration * unit_vector);
      Vector3d linear((motion_constraints.max_velocity), 0.0, 0.0);
      double dt = 0.0;
      new_trajectory_point(trajectory, robot_frame, next_point, linear,
                           {0.0, 0.0, 0.0}, dt);

      current = trajectory.back().pose.point;
      // RAMP DOWN
      // trajectory_ramp_down();
      for (int j = 0; j <= sampling_rate; j++) {
        double t = static_cast<double>(j) / sampling_rate;
        Ecef_Coord next_point =
            current - ((t * distance_deceleration) * unit_vector);
        Vector3d linear((motion_constraints.max_velocity * (1 - t)), 0.0, 0.0);
        // TODO:
        // need to add dt !!
        new_trajectory_point(trajectory, robot_frame, next_point, linear,
                             {0.0, 0.0, 0.0}, 0.0);
      }

      // trajectory_turn();
      double azimuth_rad_world = atan2(difference.y(), difference.x());
      double azimuth_rad_robot = atan2(config.transform_world_to_robot(1, 0),
                                       config.transform_world_to_robot(0, 0));
      double azimuth_rad = azimuth_rad_world - azimuth_rad_robot;

      current = trajectory.back().pose.point;
      Angular_Velocity angular;
      angular.setZero();
      angular.z() = motion_constraints.standing_turn_velocity;
      robot_frame.rotate(Eigen::AngleAxisd((azimuth_rad), Vector3d::UnitZ()));
      Velocity2d velocity = {.linear = {}, .angular = angular};
      Affine3d transformation = robot_frame;
      Pose pose = {.point = current, .transformation_matrix = transformation};
      std::cout << pose.point.transpose() << std::endl;
      double turn_duration =
          azimuth_rad / motion_constraints.standing_turn_velocity;
      Trajectory_Point tp = {
          .pose = pose, .dt = (0.0 + turn_duration), .velocity = velocity};
      trajectory.push_back(tp);
    }

    return trajectory;
  }

  double compute_displacement(double init_velocity, double final_velocity,
                              double acceleration) {
    double v0 = (init_velocity * init_velocity);
    double vf = (final_velocity * final_velocity);
    double avg_accel = (2 * acceleration);
    double displacement = (vf - v0) / avg_accel;
    return displacement;
  }

  void trajectory_ramp_up() {}
  void trajectory_cruise() {}
  void trajectory_ramp_down() {}
  void trajectory_turn() {}

  void new_trajectory_point(std::vector<Trajectory_Point> &trajectories,
                            Affine3d &robot_frame, Ecef_Coord next_point,
                            Vector3d linear, Vector3d angular, double dt) {

    Ecef_Coord diff_vec = next_point - robot_frame.translation();
    robot_frame.translation() += diff_vec;
    Velocity2d velocity = {.linear = linear, .angular = angular};
    Affine3d transformation = robot_frame;
    Pose pose = {.point = next_point, .transformation_matrix = transformation};
    Trajectory_Point tp = {.pose = pose, .dt = dt, .velocity = velocity};

    trajectories.push_back(tp);
  }

  // double turning(Robot_Config &config, Pose current, Pose next) {
  //   Affine3d &robot_frame = config.transform_world_to_robot;
  //
  //   Ecef_Coord difference = next.point - current.point;
  //   double horizontal_distance = std::sqrt(difference.x() * difference.x() +
  //                                          difference.y() * difference.y());
  //   double azimuth_rad_world = atan2(difference.y(), difference.x());
  //   double azimuth_degrees = fmod(azimuth_rad_world / M_PI * 180, 360);
  //   double elevation = std::atan2(difference.z(), horizontal_distance);
  //   double elevation_degrees = fmod(elevation / M_PI * 180, 360);
  //
  //   robot_frame.rotate(Eigen::AngleAxisd((azimuth_rad), Vector3d::UnitZ()));
  //
  //   Velocity2d velocity = {.linear = {}, .angular = angular};
  //   Affine3d transformation = robot_frame;
  //   Pose pose = {.point = path[i].point,
  //                .transformation_matrix = transformation};
  //   Trajectory_Point tp = {.pose = pose, .velocity = velocity};
  //
  //   return 0.0;
  // }

  // TODO calculate time for a curve
  double compute_straight_line_travel_time(double distance, Velocity2d v0,
                                           Velocity2d vf) {
    double distance_acceleration =
        ((motion_constraints.max_velocity * motion_constraints.max_velocity) -
         (v0.linear.x() * v0.linear.x())) /
        (2 * velocity_profile.acceleration_rate);

    double distance_deceleration =
        ((motion_constraints.max_velocity * motion_constraints.max_velocity) -
         (vf.linear.x() * vf.linear.x())) /
        (2 * velocity_profile.deceleration_rate);

    double t_accel = (motion_constraints.max_velocity - v0.linear.x()) /
                     velocity_profile.acceleration_rate;
    double t_decel = (motion_constraints.max_velocity - vf.linear.x()) /
                     velocity_profile.deceleration_rate;
    double t_cruise = 0;

    if (distance_acceleration + distance_deceleration > distance) {
      double peak_v = std::sqrt(
          (2 * velocity_profile.acceleration_rate * distance +
           (v0.linear.x() * v0.linear.x() + vf.linear.x() * vf.linear.x())) /
          2);
      t_accel = (peak_v - v0.linear.x()) / velocity_profile.acceleration_rate;
      t_decel = (peak_v - vf.linear.x()) / velocity_profile.deceleration_rate;
    } else {
      double d_cruise =
          distance - (distance_acceleration + distance_deceleration);
      t_cruise = d_cruise / motion_constraints.max_velocity;
    }

    double total_time = t_accel + t_cruise + t_decel;

    return total_time;
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
