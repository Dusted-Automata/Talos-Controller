// #include "robot.hpp"
//
// void createTrajectories(std::array<Pose2d, 10> waypoints) {
//   std::cout << waypoints;
// }

#include <cmath>
#include <iostream>
#include <vector>

struct Point2D {
  double x;
  double y;
};

// Generate a smooth path through waypoints using cubic spline interpolation
std::vector<Point2D> generateTrajectory(const std::vector<Point2D> &waypoints,
                                        double resolution = 0.1) {
  std::vector<Point2D> trajectory;

  if (waypoints.size() < 2) {
    return trajectory;
  }

  // Generate points along each segment
  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    Point2D start = waypoints[i];
    Point2D end = waypoints[i + 1];

    // Calculate segment length
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Number of points to generate along this segment
    int points = static_cast<int>(distance / resolution);

    // Generate points along the segment
    for (int j = 0; j <= points; j++) {
      double t = static_cast<double>(j) / points;
      Point2D point;

      // Cubic interpolation for smoother paths
      double h00 = 2 * t * t * t - 3 * t * t + 1;
      double h10 = t * t * t - 2 * t * t + t;
      double h01 = -2 * t * t * t + 3 * t * t;
      double h11 = t * t * t - t * t;

      point.x = h00 * start.x + h10 * dx + h01 * end.x + h11 * dx;
      point.y = h00 * start.y + h10 * dy + h01 * end.y + h11 * dy;

      trajectory.push_back(point);
    }
  }

  return trajectory;
}

// Optional: Add velocity profile to the trajectory
std::vector<double>
generateVelocityProfile(const std::vector<Point2D> &trajectory,
                        double maxVelocity, double maxAcceleration) {
  std::vector<double> velocities(trajectory.size());

  // Simple trapezoidal velocity profile
  for (size_t i = 0; i < trajectory.size(); i++) {
    double progress = static_cast<double>(i) / trajectory.size();

    // Accelerate at start, decelerate at end
    if (progress < 0.2) {
      velocities[i] = maxVelocity * (progress / 0.2);
    } else if (progress > 0.8) {
      velocities[i] = maxVelocity * ((1.0 - progress) / 0.2);
    } else {
      velocities[i] = maxVelocity;
    }
  }

  return velocities;
}

// Example usage
int main() {
  // Create some example waypoints
  std::vector<Point2D> waypoints = {
      {0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 2.0}};

  // Generate trajectory
  std::vector<Point2D> trajectory = generateTrajectory(waypoints, 0.1);

  // Optional: Generate velocity profile
  std::vector<double> velocities =
      generateVelocityProfile(trajectory, 1.0, 0.5);

  // Print trajectory points
  for (size_t i = 0; i < trajectory.size(); i++) {
    std::cout << "Point " << i << ": (" << trajectory[i].x << ", "
              << trajectory[i].y << ") "
              << "Velocity: " << velocities[i] << std::endl;
  }

  return 0;
}
