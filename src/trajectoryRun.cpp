#include "trajectory.cpp"

int main() {
  // std::vector<Ecef_Coord> waypoints = {
  //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
  //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
  //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
  //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

  std::vector<Ecef_Coord> waypoints = {
      {0.0, 0.0, 0.0}, {4.0, -2.0, 0.0}, {4.0, 2.0, 0.0}, {0.0, 0.0, 0.0}};

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 2.0,
                                                .standing_turn_velocity = 2.0,
                                                .max_acceleration = 0.5,
                                                .max_deceleration = 0.5,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };
  Velocity_Profile vel_profile = {};
  // PIDGains linear_gains = {1.0, 0.0, 0.0};
  // PIDController linear_pid(linear_gains);
  // PIDGains angular_gains = {1.0, 0.0, 0.0};
  // PIDController angular_pid(angular_gains);
  // Trajectory_Controller controller(config.motion_constraints, vel_profile,
  //                                  linear_pid, angular_pid, config.hz);
  Trajectory_Controller controller(config.motion_constraints, vel_profile,
                                   config.hz);

  std::vector<Trajectory_Point> trajectories =
      controller.generate_trajectory(waypoints, config);

  // std::vector<double> velocities =
  //     generate_velocity_profile(trajectories, config);
  //
  saveToFile("trajectories", trajectories);
  // saveToFileTrajectories("trajectories", trajectories);
  // saveToFile("velocities", trajectories);

  // Print trajectory points
  // for (size_t i = 0; i < trajectories.size(); i++) {
  //   std::cout << "Point " << i << ": (" << trajectories[i].pose.x << ",
  //             << trajectories[i].pose.y << ") "
  //             << "Velocity: " << trajectories[i].velocity.linear <<
  //             std::endl;
  // }

  // Eigen::Matrix3d R;
  // double angle = M_PI / 2;
  // R = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  // Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  // std::cout << "Rotation matrix : \n " << transform.affine() << std::endl;

  return 0;
}
