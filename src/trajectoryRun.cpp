#include "robot.hpp"
#include "trajectory.hpp"
#include <iostream>
#include <thread>

class testRobot : public Robot {

public:
  testRobot(Trajectory_Controller trajectory_controller) : Robot(trajectory_controller) {}
  void send_velocity_command(Velocity2d &cmd) override {};
  void update_state() override {};
  void read_sensors() override {};
  Robot_State read_state() override { return {}; };
};

void worker_function(std::function<void()> callback, int period_ms) {
  while (1) {
    callback();
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }
}

int main() {
  // std::vector<Ecef_Coord> waypoints = {
  //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
  //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
  //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
  //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

  std::vector<Ecef_Coord> waypoints = {
      {0.0, 0.0, 0.0}, {4.0, -2.0, 0.0}, {4.0, 2.0, 0.0}, {0.0, 0.0, 0.0}};

  Velocity_Profile vel_profile = {.acceleration_rate = 35.0, .deceleration_rate = 10.0};
  Robot_Config config = {
      .hz = 50,
      .motion_constraints = {.max_velocity = 1.0,
                             .standing_turn_velocity = 2.0,
                             .max_acceleration = 0.5,
                             .max_deceleration = 0.5,
                             .max_jerk = 0.0,
                             .corner_velocity = 0.0},
      .velocity_profile = vel_profile,

  };
  PIDGains linear_gains = {1.0, 0.0, 0.0};
  PIDController linear_pid(linear_gains);
  linear_pid.output_max = 10, 0;
  linear_pid.output_min = 0, 0;
  PIDGains angular_gains = {1.0, 0.0, 0.0};
  PIDController angular_pid(angular_gains);
  angular_pid.output_max = 10, 0;
  angular_pid.output_min = 0, 0;
  Trajectory_Controller controller(config, linear_pid, angular_pid, config.hz);

  Ecef_Coord current = waypoints[0];
  Ecef_Coord next = waypoints[1];

  testRobot robot(controller);

  std::function<void()> bound_path_loop =
      std::bind(&Trajectory_Controller::path_loop, &controller, std::ref(robot.path_queue),
                std::ref(waypoints));
  std::function<void()> bound_trajectory_loop =
      std::bind(&Trajectory_Controller::trajectory_loop, &controller,
                std::ref(robot.trajectory_queue), std::ref(robot.path_queue));
  std::function<void()> bound_control_loop = std::bind(&testRobot::control_loop, &robot);
  std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);
  std::thread trajectory_loop = std::thread(worker_function, bound_trajectory_loop, 100);
  std::thread control_loop_thread = std::thread(worker_function, bound_control_loop, 2);
  // std::thread control_loop_thread = std::thread(&testRobot::control_loop, &robot);

  path_loop.join();
  trajectory_loop.join();
  control_loop_thread.join();

  return 0;
}
