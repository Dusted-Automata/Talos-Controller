// #include "robot.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;

struct Pose {
  Vector3d point;
  Matrix4d Transformation_Matrix; // Change this to Quaternion maybe
};

struct Angular_Velocity {
  double roll, pitch, yaw;
};
struct Linear_Velocity {
  double forward, lateral, vertical;
};
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
  Matrix4d robot_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  Matrix4d world_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  Matrix4d transform_world_to_robot{
      {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
};

struct Trajectory_Point {
  Pose pose;
  double dt;
  Velocity2d velocity;
};

struct Trajectory_Plan {
  std::vector<Trajectory_Point> trajectory_points;
  std::vector<int> coordinate_indices;
};

double calculate_theta(double dx, double dy) {
  // allow minimum turning radius constraints
  // cubic splines for smoother paths
  // accelleration limits for rotational motion
  double Theta = atan2(dy, dx);
  return Theta;
}

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

double calculate_velocity(double progress, const Robot_Config &config) {
  if (progress < 0.2) {
    return config.motion_constraints.max_velocity * (progress / 0.2);
  } else if (progress > 0.8) {
    return config.motion_constraints.max_velocity * ((1.0 - progress) / 0.2);
  } else {
    return config.motion_constraints.max_velocity;
  }
};

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

      double theta = calculate_theta(dx, dy);
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
                    const Robot_Config &config) {
  std::vector<Trajectory_Point> path = {};

  if (coordinates.size() < 2) {
    return path;
  }

  double resolution = 1.0 / config.hz;

  for (int i = 0; i < coordinates.size() - 1; i++) {
    Ecef_Coord current = coordinates[i];
    Ecef_Coord next = coordinates[i + 1];
    Vector3d difference = next - current;
    double horizontal_distance = std::sqrt(difference.x() * difference.x() +
                                           difference.y() * difference.y());
    int points = static_cast<int>(horizontal_distance / resolution);

    for (int j = 0; j <= points; j++) {
      double t = static_cast<double>(j) / points;
      // Pose3d point = cubic_interpolation(dx, dy, start, end, t);
      double x = current.x() * (1 - t) + next.x() * t;
      double y = current.y() * (1 - t) + next.y() * t;
      double z = current.z() * (1 - t) + next.z() * t;

      Velocity2d velocity = {.linear = {calculate_velocity(t, config)}};

      Vector3d point(x, y, z);
      Pose pose = {.point = point};
      Trajectory_Point tp = {.pose = pose, .velocity = velocity};
      path.push_back(tp);
    }

    double azimuth = atan2(difference.y(), difference.x());
    double azimuth_degrees = fmod(azimuth / M_PI * 180, 360);

    // Ensure azimuth is between 0 and 360 degrees
    // Calculate elevation angle (vertical angle from x-y plane)
    double elevation = std::atan2(difference.z(), horizontal_distance);
    double elevation_degrees = fmod(elevation / M_PI * 180, 360);
    // std::cout << azimuth_degrees << std::endl;
    // std::cout << elevation_degrees << std::endl;

    int turn_points = static_cast<int>(azimuth_degrees / resolution);
    std::cout << turn_points << std::endl;
    for (int j = 0; j < abs(turn_points); j++) {
      // double theta = calculate_theta(difference.x(), difference.y());
      double t = (double)j / points;
      Angular_Velocity angular;
      angular.yaw = 0.02;
      // if (t < 0.2) {
      //   angular.yaw = config.motion_constraints.max_velocity * (t / 0.2);
      // } else if (t > 0.8) {
      //   angular.yaw =
      //       config.motion_constraints.max_velocity * ((1.0 - t) / 0.2);
      // } else {
      //   angular.yaw = 1.0;
      // }

      Velocity2d velocity = {.angular = angular};

      Pose pose = {.point = path[path.size() - 1].pose.point};
      Trajectory_Point tp = {.pose = pose, .velocity = velocity};
      path.push_back(tp);
    }
  }

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
    outFile << "Velocity-linear: forward: " << line.velocity.linear.forward
            << " lateral: " << line.velocity.linear.lateral
            << " vertical: " << line.velocity.linear.vertical << std::endl;
    outFile << "Velocity-angular: pitch: " << line.velocity.angular.pitch
            << " roll: " << line.velocity.angular.roll
            << " yaw: " << line.velocity.angular.yaw << std::endl;
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

// Example usage
// int main() {
//   // Create some example waypoints
//   std::vector<Ecef_Coord> waypoints = {
//       {4100175.625135626, 476368.7899695045, 4846344.356704135},
//       {4100209.6729529747, 476361.2681338759, 4846316.478097512},
//       {4100218.5394949187, 476445.5598077707, 4846300.796185957},
//       {4100241.72195791, 476441.0557096391, 4846281.753675706}};
//
//   Robot_Config config = {.hz = 50,
//                          .motion_constraints = {.max_velocity = 2.0,
//                                                 .max_acceleration = 0.5,
//                                                 .max_deceleration = 0.5,
//                                                 .max_jerk = 0.0,
//                                                 .corner_velocity = 0.0}
//
//   };
//
//   std::vector<Trajectory_Point> trajectories =
//       generate_trajectory(waypoints, config);
//
//   // std::vector<double> velocities =
//   //     generate_velocity_profile(trajectories, config);
//   //
//   saveToFile("waypoints", waypoints);
//   saveToFileTrajectories("trajectories", trajectories);
//   saveToFile("velocities", trajectories);
//
//   // Print trajectory points
//   // for (size_t i = 0; i < trajectories.size(); i++) {
//   //   std::cout << "Point " << i << ": (" << trajectories[i].pose.x << ",
//   //             << trajectories[i].pose.y << ") "
//   //             << "Velocity: " << trajectories[i].velocity.linear <<
//   //             std::endl;
//   // }
//
//   return 0;
// }
