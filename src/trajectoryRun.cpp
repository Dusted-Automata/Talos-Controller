#include "robot.hpp"
#include "trajectory.hpp"
#include <iostream>
#include <thread>

class testRobot : public Robot {

public:
  testRobot(Trajectory_Controller trajectory_controller) : Robot(trajectory_controller) {}
  void send_velocity_command(Velocity2d cmd) override {};
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

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 1.0,
                                                .standing_turn_velocity = 2.0,
                                                .max_acceleration = 0.5,
                                                .max_deceleration = 0.5,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };
  Velocity_Profile vel_profile = {.acceleration_rate = 35.0, .deceleration_rate = 10.0};
  PIDGains linear_gains = {1.0, 0.0, 0.0};
  PIDController linear_pid(linear_gains);
  PIDGains angular_gains = {1.0, 0.0, 0.0};
  PIDController angular_pid(angular_gains);
  Trajectory_Controller controller(config.motion_constraints, vel_profile, linear_pid, angular_pid,
                                   config.hz);

  Ecef_Coord current = waypoints[0];
  Ecef_Coord next = waypoints[1];

  std::vector<Trajectory_Point> trajectories =
      controller.generate_trajectory(current, next, config);

  testRobot robot(controller);
  Thread_Safe_Queue<Trajectory_Point> t_queue = {};
  // robot.trajectory_queue =

  auto bound_path_loop =
      std::bind(&Trajectory_Controller::path_loop, &controller, std::ref(robot.trajectory_queue),
                std::ref(current), std::ref(next), std::ref(config));
  auto bound_control_loop = std::bind(&testRobot::control_loop, &robot);
  std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);
  std::thread control_loop_thread = std::thread(worker_function, bound_control_loop, 2);
  // std::thread control_loop_thread = std::thread(&testRobot::control_loop, &robot);

  path_loop.join();
  control_loop_thread.join();

  // Thread_Safe_Queue<SimplePoint> q;
  // SimplePoint sp{1234.5};
  // if (!q.push(sp)) {
  //   std::cerr << "Push failed?\n";
  // }
  // auto frontOpt = q.front();
  // if (frontOpt) {
  //   std::cout << "Got dt = " << frontOpt->dt << std::endl;
  // } else {
  //   std::cout << "No front?\n";
  // }

  // std::optional<Trajectory_Point> cmd = robot.trajectory_queue.front();
  // std::cout << cmd.value().velocity.linear.x() << " , " << cmd.value().velocity.angular.z()
  //           << std::endl;
  // robot.trajectory_queue.pop();
  // cmd = robot.trajectory_queue.front();
  // std::cout << cmd.value().velocity.linear.x() << " , " << cmd.value().velocity.angular.z()
  //           << std::endl;
  // while (1) {
  // }

  // std::this_thread::sleep_for(std::chrono::seconds(10));

  // std::vector<double> velocities =
  //     generate_velocity_profile(trajectories, config);
  //
  // saveToFile("trajectories", trajectories);
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

  // for (const auto &line : trajectories) {
  //   std::cout << std::fixed;
  //   std::cout << " ------------------------------------------------- " << std::endl;
  //   std::cout << "dt: " << line.dt << " " << std::endl;
  //   std::cout << "pose: " << line.pose.point.x() << " " << line.pose.point.y() << " "
  //             << line.pose.point.z() << " " << std::endl;
  //   std::cout << "Velocity-linear: forward: " << line.velocity.linear.x()
  //             << " lateral: " << line.velocity.linear.y()
  //             << " vertical: " << line.velocity.linear.z() << std::endl;
  //   std::cout << "Velocity-angular: pitch: " << line.velocity.angular.x()
  //             << " roll: " << line.velocity.angular.y() << " yaw: " << line.velocity.angular.z()
  //             << std::endl;
  // }

  return 0;
}
