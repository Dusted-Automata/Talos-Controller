#include "pid.cpp"
#include "robot.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;
typedef Vector3d Linear_Velocity;
typedef Vector3d Angular_Velocity;

struct Pose {
  Vector3d point;
  Matrix4d Transformation_Matrix; // Change this to Quaternion maybe
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

struct Robot_Config {
  int hz;
  Motion_Constraints motion_constraints;
  // TODO: Make a real frame and transformation data structure
  // Matrix4d robot_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0,
  // 1}}; Matrix4d world_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0,
  // 0, 1}};
  Eigen::Affine3d transform_world_to_robot = Eigen::Affine3d::Identity();
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

Vector3d calculate_velocity(double progress, const Robot_Config &config) {
  Vector3d vec;
  vec.setZero();
  if (progress < 0.2) {
    vec.x() = config.motion_constraints.max_velocity * (progress / 0.2);
    return vec;
  } else if (progress > 0.8) {
    vec.x() = config.motion_constraints.max_velocity * ((1.0 - progress) / 0.2);
    return vec;
  } else {
    vec.x() = config.motion_constraints.max_velocity;
    return vec;
  }
};

// High level Path planning (Maybe, check back later)
std::vector<Trajectory_Point>
generate_geometric_trajectory(const std::vector<Ecef_Coord> &waypoints,
                              const Robot_Config &robot_config) {

  std::vector<Trajectory_Point> path = {};

  if (waypoints.size() < 2) {
    return path;
  }

  double resolution = 1.0 / robot_config.hz;

  for (int i = 0; i < waypoints.size() - 1; i++) {
    Ecef_Coord current = waypoints[i];
    Ecef_Coord next = waypoints[i + 1];
    double dx = next.x() - current.x();
    double dy = next.y() - current.y();
    double distance = std::sqrt(dx * dx + dy * dy);
    int points = static_cast<int>(distance / resolution);

    for (int j = 0; j <= points; j++) {
      double t = static_cast<double>(j) / points;
      // Pose3d point = cubic_interpolation(dx, dy, start, end, t);
      double x = current.x() * (1 - t) + next.x() * t;
      double y = current.y() * (1 - t) + next.y() * t;
      double z = current.z() * (1 - t) + next.z() * t;

      // Vector3d point = {.x = x, .y = y, .z = z};
      // Vector3d point;
      // point << x, y, z;
      Vector3d point(x, y, z);

      Pose pose = {.point = point};
      Trajectory_Point tp = {.pose = pose};
      path.push_back(tp);
    }
  }

  return path;
}

std::vector<Trajectory_Point>
generate_trajectory(const std::vector<Ecef_Coord> &coordinates,
                    Robot_Config &config) {
  std::vector<Trajectory_Point> path = {};

  if (coordinates.size() < 2) {
    return path;
  }

  double resolution = 1.0 / config.hz;
  PIDController linear_pid({1.0, 0.0, 0.0});
  PIDController angular_pid({1.0, 0.0, 0.0});

  for (int i = 0; i < coordinates.size() - 1; i++) {
    Ecef_Coord current = coordinates[i];
    Ecef_Coord next = coordinates[i + 1];
    Vector3d difference = next - current;
    double horizontal_distance = std::sqrt(difference.x() * difference.x() +
                                           difference.y() * difference.y());
    int points = static_cast<int>(horizontal_distance / resolution);

    double azimuth_rad_world = atan2(difference.y(), difference.x());
    double azimuth_rad_robot = atan2(config.transform_world_to_robot(1, 0),
                                     config.transform_world_to_robot(0, 0));
    double azimuth_rad = azimuth_rad_world - azimuth_rad_robot;
    double azimuth_degrees = fmod(azimuth_rad_world / M_PI * 180, 360);
    // Ensure azimuth is between 0 and 360 degrees
    // Calculate elevation angle (vertical angle from x-y plane)
    double elevation = std::atan2(difference.z(), horizontal_distance);
    double elevation_degrees = fmod(elevation / M_PI * 180, 360);

    int turn_points;
    if (config.motion_constraints.standing_turn_velocity != 0) {
      turn_points = static_cast<int>(
          azimuth_rad_world /
          (config.motion_constraints.standing_turn_velocity * resolution));
    } else {
      turn_points = 20;
    }
    // int turn_points =
    // static_cast<int>(config.motion_constraints.standing_turn_velocity /
    // resolution);
    // double test =
    //     azimuth_rad -
    //     (config.motion_constraints.standing_turn_velocity * resolution);
    std::cout << "turn_points: " << turn_points
              << " azimuth_deg: " << azimuth_degrees
              << " azimuth_rad: " << azimuth_rad_world << std::endl;

    // std::cout << "before translation: " << std::endl
    //           << config.transform_world_to_robot << std::endl;

    for (int j = 0; (j < abs(turn_points)); j++) {
      double t = (double)j / points;
      Angular_Velocity angular;
      angular.z() = config.motion_constraints.standing_turn_velocity;
      // if (t < 0.2) {
      //   angular.yaw = config.motion_constraints.max_velocity * (t / 0.2);
      // } else if (t > 0.8) {
      //   angular.yaw =
      //       config.motion_constraints.max_velocity * ((1.0 - t) / 0.2);
      // } else {
      //   angular.yaw = 1.0;
      // }
      // Eigen::Affine3d transform = Eigen::Affine3d::Identity();
      // transform.rotate(
      //     Eigen::AngleAxisd(azimuth_rad, Eigen::Vector3d::UnitZ()));
      config.transform_world_to_robot.rotate(Eigen::AngleAxisd(
          (azimuth_rad * 1 / abs(turn_points)), Vector3d::UnitZ()));

      Velocity2d velocity = {.linear = {}, .angular = angular};
      Matrix4d transformation = config.transform_world_to_robot.matrix();
      Pose pose = {.point = coordinates[i],
                   .Transformation_Matrix = transformation};
      Trajectory_Point tp = {.pose = pose, .velocity = velocity};
      path.push_back(tp);
    }
    // std::cout << "After turning: " << path.size() << std::endl;

    auto test = config.transform_world_to_robot.translation();
    auto test2 = config.transform_world_to_robot;
    for (int j = 0; j <= points; j++) {
      double t = static_cast<double>(j) / points;
      // Pose3d point = cubic_interpolation(dx, dy, start, end, t);

      Ecef_Coord next_point = (current * (1 - t)) + (next * t);
      Ecef_Coord diff_vec =
          next_point - config.transform_world_to_robot.translation();
      // config.transform_world_to_robot.translate(diff_vec);
      // test = diff_vec + test;
      // test2.translate(diff_vec);
      config.transform_world_to_robot.translation() += diff_vec;

      Velocity2d velocity = {.linear = calculate_velocity(t, config),
                             .angular = {}};

      Matrix4d transformation = config.transform_world_to_robot.matrix();
      // std::cout << transformation << std::endl;
      // std::cout << test << std::endl;
      // std::cout << test2.translation() << std::endl;
      // std::cout << " ------------" << std::endl;

      Pose pose = {.point = next_point,
                   .Transformation_Matrix = transformation};
      Trajectory_Point tp = {.pose = pose, .velocity = velocity};
      path.push_back(tp);
    }
    // std::cout << "After linear: " << path.size() << std::endl;
  }
  // std::cout << "After all: " << path.size() << std::endl;

  return path;
}

// std::vector<double>
// generate_velocity_profile(std::vector<Trajectory_Point> &path,
//                           Robot_Config &robot_config) {
//   std::vector<double> velocities(path.size());
//
//   double distance_vel = 0.0;
//   double distance_traj = 0.0;
//   double dx, dy;
//   // trapezoidal velocity profile
//   for (int i = 0; path.size(); i++) {
//     for (size_t j = 0; j < path.size(); j++) {
//       double progress = static_cast<double>(j) / path.size();
//       if (j < path.size() - 1) {
//         dx = path[j + 1].pose.point.x() - path[j].pose.point.x();
//         dy = path[j + 1].pose.point.y() - path[j].pose.point.y();
//         distance_traj += std::sqrt(dx * dx + dy * dy);
//       }
//
//       if (j > 0) {
//         distance_vel += velocities[j - 1] * (1.0 / robot_config.hz);
//       }
//
//       if (progress < 0.2) {
//         velocities[j] =
//             robot_config.motion_constraints.max_velocity * (progress / 0.2);
//       } else if (progress > 0.8) {
//         velocities[j] = robot_config.motion_constraints.max_velocity *
//                         ((1.0 - progress) / 0.2);
//       } else {
//         if (distance_vel < distance_traj) {
//           velocities[j] = robot_config.motion_constraints.max_velocity;
//         }
//         double displacement = std::sqrt(dx * dx + dy * dy);
//         velocities[j] = displacement / (1.0 / robot_config.hz);
//       }
//     }
//   }
//
//   return velocities;
// }

static bool saveToFile(const std::string &filename,
                       const std::vector<Ecef_Coord> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << line.x() << "," << line.y() << std::endl;
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

static bool saveToFileTrajectories(const std::string &filename,
                                   const std::vector<Trajectory_Point> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << std::fixed;
    outFile << 1 << " " << 0 << " " << 0 << " " << line.pose.point.x() << " ";
    outFile << 0 << " " << 1 << " " << 0 << " " << line.pose.point.y() << " ";
    outFile << 0 << " " << 0 << " " << 1 << " " << line.pose.point.z();
    outFile << std::endl;
  }

  outFile.close();
  return true;
}

static bool saveToFile(const std::string &filename,
                       const std::vector<double> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << line << std::endl;
  }

  outFile.close();
  return true;
}

static bool saveToFile(const std::string &filename, const int amount) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (int i = 0; i < amount; i++) {
    double dt = 0.02 * i;
    outFile << dt << std::endl;
  }
  return true;
};
