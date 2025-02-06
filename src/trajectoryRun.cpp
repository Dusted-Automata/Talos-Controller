#include "trajectory.cpp"

int main() {
  std::vector<Ecef_Coord> waypoints = {
      {4100175.625135626, 476368.7899695045, 4846344.356704135},
      {4100209.6729529747, 476361.2681338759, 4846316.478097512},
      {4100218.5394949187, 476445.5598077707, 4846300.796185957},
      {4100241.72195791, 476441.0557096391, 4846281.753675706}};

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 2.0,
                                                .max_acceleration = 0.5,
                                                .max_deceleration = 0.5,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };

  std::vector<Trajectory_Point> trajectories =
      generate_trajectory(waypoints, config);

  // std::vector<double> velocities =
  //     generate_velocity_profile(trajectories, config);
  //
  saveToFile("waypoints", waypoints);
  saveToFileTrajectories("trajectories", trajectories);
  saveToFile("velocities", trajectories);

  // Print trajectory points
  // for (size_t i = 0; i < trajectories.size(); i++) {
  //   std::cout << "Point " << i << ": (" << trajectories[i].pose.x << ",
  //             << trajectories[i].pose.y << ") "
  //             << "Velocity: " << trajectories[i].velocity.linear <<
  //             std::endl;
  // }

  return 0;
}
