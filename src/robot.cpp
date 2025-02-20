#include "robot.hpp"
#include <Eigen/Dense>
#include <iostream>

void Robot::control_loop() {
  Robot_State state = read_state();
  // std::cout << "DT: " << motiontime << " VEL: " << state.velocity[0] << " YAW: " <<
  // state.yawSpeed
  //           << " | " << std::endl;
  // << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed << std::endl;

  // Path_Movement path = readPath();
  // Thread_Safe_Queue<Trajectory_Point> trajectories = readPath();
  // Sensors sensors = readSensors();

  Velocity2d cmd = trajectory_controller.follow_trajectory(trajectory_queue, state);
  send_velocity_command(cmd);
  motiontime += 1000 / hz;

  // will be 0.0 until i fix the state.
  // std::cout << cmd.linear.x() << " , " << cmd.angular.z() << std::endl;
}

// void Robot::updateState() {}
// Robot_State Robot::read_state() {}
void Robot::read_path() {}
// void Robot::read_sensors() {}
