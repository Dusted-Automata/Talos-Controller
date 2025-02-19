#pragma once
#include "trajectory.hpp"
#include "types.hpp"
#include <Eigen/Dense>

// struct ControlCommand {
//   Eigen::Vector4d effort;
//   float timestamp;
//
//   ControlCommand(const Eigen::VectorXd &eff, float time)
//       : effort(eff), timestamp(time) {}
// };

// class Controller {
// public:
//   virtual ~Controller() = default;
//
//   virtual ControlCommand computeControl(const ControlState &currentState,
//                                         const ControlState &desiredState) =
//                                         0;
//
//   virtual void reset() = 0;
// };

class Robot {
  int motiontime = 0;
  float hz = 500;
  Trajectory_Controller trajectory_controller;
  // Thread_Safe_Queue<Trajectory_Point> trajectory_queue = {};
  Thread_Safe_Queue<Ecef_Coord> waypoint_queue = {};
  Robot_Config config = {};
  // void (*control_loop)();
  // MAIN CONTROL THREAD
  // PATH GENERATION THREAD
  // SENSOR PROCESSING THREAD
public:
  Robot(Trajectory_Controller trajectory_controller)
      : trajectory_controller(trajectory_controller) {}
  virtual ~Robot() = default;
  Thread_Safe_Queue<Trajectory_Point> trajectory_queue;

  virtual void send_velocity_command(Velocity2d cmd) = 0;
  virtual void update_state() = 0;
  void control_loop();
  virtual Robot_State read_state() = 0;
  void read_path();
  virtual void read_sensors() = 0;
};
