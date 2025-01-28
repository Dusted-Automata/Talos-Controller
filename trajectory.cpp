// #include "robot.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

struct Pose3f {
  float x, y, theta;
};

struct Velocity2d {
  // velocity[0], yawspeed I think in HighCmd
  double linear;  // (m/s)
  double angular; // (rad/s)
};

struct Motion_Constraints {
  double max_velocity;
  double max_acceleration;
  double max_deceleration;
  double max_jerk;
  double corner_velocity;
};

struct config {
  int hz;
  Motion_Constraints motion_constraints;
};
//
// void createTrajectories(std::array<Pose2d, 10> waypoints) {
//   std::cout << waypoints;
// }

struct Trajectory_Point {
  Pose3f pose;
  double dt;
  Velocity2d velocity;
};

struct Ecef_Coord {
  float x, y, z;
};

double calculate_theta(double dx, double dy) {
  // allow minimum turning radius constraints
  // cubic splines for smoother paths
  // accelleration limits for rotational motion
  double Theta = atan2(dy, dx);
  return Theta;
}

Pose3f cubic_interpolation(double dx, double dy, Ecef_Coord start,
                           Ecef_Coord end, double t) {
  double h00 = 2 * t * t * t - 3 * t * t + 1;
  double h10 = t * t * t - 2 * t * t + t;
  double h01 = -2 * t * t * t + 3 * t * t;
  double h11 = t * t * t - t * t;

  Pose3f point = {};
  point.x = h00 * start.x + h10 * dx + h01 * end.x + h11 * dx;
  point.y = h00 * start.y + h10 * dy + h01 * end.y + h11 * dy;
  return point;
};

std::vector<Trajectory_Point>
generate_geometric_trajectory(std::vector<Trajectory_Point> &trajectories,
                              const std::vector<Ecef_Coord> &coordinates,
                              config &robot_config) {

  if (coordinates.size() < 2) {
    return trajectories;
  }

  double resolution = 1.0 / robot_config.hz;

  for (int i = 0; i < coordinates.size() - 1; i++) {
    Ecef_Coord start = coordinates[i];
    Ecef_Coord end = coordinates[i + 1];
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    int points = static_cast<int>(distance / resolution);

    for (int j = 0; j <= points; j++) {
      double t = static_cast<double>(j) / points;
      // Pose3d point = cubic_interpolation(dx, dy, start, end, t);
      float x = start.x + dx * t;
      float y = start.y + dy * t;
      float theta = calculate_theta(dx, dy);
      Pose3f point = {.x = x, .y = y, .theta = theta};
      Trajectory_Point tp = {.pose = point};
      trajectories.push_back(tp);
    }
  }

  return trajectories;
}

double calculate_velocity() { return 0.0; };
// Optional: Add velocity profile to the trajectory
std::vector<double>
generate_velocity_profile(const std::vector<Trajectory_Point> &trajectory,
                          config &robot_config) {
  std::vector<double> velocities(trajectory.size());

  double distance_vel = 0.0;
  double distance_traj = 0.0;
  float dx, dy;
  // Simple trapezoidal velocity profile
  for (size_t i = 0; i < trajectory.size(); i++) {
    double progress = static_cast<double>(i) / trajectory.size();
    if (i < trajectory.size() - 1) {
      dx = trajectory[i + 1].pose.x - trajectory[i].pose.x;
      dy = trajectory[i + 1].pose.y - trajectory[i].pose.y;
      distance_traj += std::sqrt(dx * dx + dy * dy);
    }

    if (i > 0) {
      distance_vel += velocities[i - 1] * (1.0 / robot_config.hz);
    }

    // Accelerate at start, decelerate at end
    if (progress < 0.2) {
      velocities[i] =
          robot_config.motion_constraints.max_velocity * (progress / 0.2);
    } else if (progress > 0.8) {
      velocities[i] = robot_config.motion_constraints.max_velocity *
                      ((1.0 - progress) / 0.2);
    } else {
      if (distance_vel < distance_traj) {
        velocities[i] = robot_config.motion_constraints.max_velocity;
      }
      float displacement = std::sqrt(dx * dx + dy * dy);
      velocities[i] = displacement / (1.0 / robot_config.hz);
    }
  }

  return velocities;
}

static bool saveToFile(const std::string &filename,
                       const std::vector<Ecef_Coord> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << line.x << "," << line.y << std::endl;
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
    outFile << line.dt << "," << line.pose.x << "," << line.pose.y << ","
            << line.pose.theta << std::endl;
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

// Example usage
// int main() {
//   // Create some example waypoints
//   std::vector<Ecef_Coord> waypoints = {
//       {0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 2.0}};
//
//   // Generate trajectory
//   std::vector<Trajectory_Point> trajectories = {};
//   generate_trajectory(trajectories, waypoints, 0.1);
//
//   // Optional: Generate velocity profile
//   std::vector<double> velocities =
//       generate_velocity_profile(trajectories, 1.0, 0.5);
//
//   saveToFile("waypoints", waypoints);
//   saveToFile("trajectories", trajectories);
//
//   // Print trajectory points
//   // for (size_t i = 0; i < trajectories.size(); i++) {
//   //   std::cout << "Point " << i << ": (" << trajectories[i].pose.x << ", "
//   //             << trajectories[i].pose.y << ") "
//   //             << "Velocity: " << trajectories[i].velocity.linear <<
//   //             std::endl;
//   // }
//
//   return 0;
// }
