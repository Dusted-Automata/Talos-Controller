#include "robot.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <thread>

// struct ControlCommand {
//   Eigen::Vector4d effort;
//   float timestamp;
//
//   ControlCommand(const Eigen::VectorXd &eff, float time)
//       : effort(eff), timestamp(time) {}
// };

class Controller {
public:
  virtual ~Controller() = default;

  // virtual double update(const ControlState &currentState,
  //     const ControlState &desiredState) = 0;
  virtual double update(double measured_value, double dt) = 0;

  virtual void reset() = 0;
};

// void Robot::updateState() {}
void Robot::control_loop() {
  // while (1) {
  Robot_State state = read_state();
  // Path_Movement path = readPath();
  // Thread_Safe_Queue<Trajectory_Point> trajectories = readPath();
  // Sensors sensors = readSensors();
  Velocity2d cmd = trajectory_controller.follow_trajectory(trajectory_queue, state);
  // send_velocity_command(cmd);
  motiontime += 1000 / hz;
  // cmd = run_through(cmd, path.trajectories[path.trajectory_index]);

  // std::cout << cmd.linear.x() << " , " << cmd.angular.z() << std::endl;
  // std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // file << "DT: " << motiontime << " VEL: " << cmd.velocity[0]
  //      << " YAW: " << cmd.yawSpeed << " | "
  //      << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed
  //      << std::endl;
  // }
}
// Robot_State Robot::read_state() {}
void Robot::read_path() {}
// void Robot::read_sensors() {}
