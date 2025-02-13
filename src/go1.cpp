#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#include "trajectory.cpp"
#include "unitree_legged_sdk/comm.h"

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

// using namespace UNITREE_LEGGED_SDK;
#define UT UNITREE_LEGGED_SDK

enum class GaitType : uint8_t { Idle, Trot, Climb_stair, Trot_obstacle };
enum class Go1_mode : uint8_t {
  Idle,                    // idle, default stand
  Force_stand,             // force stand (controlled by dBodyHeight + ypr)
  Target_velocity_walking, // target velocity walking (controlled by velocity +
                           // yawSpeed)
  Target_position_walking_RESERVED, // target position walking (controlled by
                                    // position + ypr[0]), reserve
  Path_mode_walking_RESERVED, // path mode walking (reserve for future release),
                              // reserve
  Stand_down,
  Stand_up,
  Damping_mode,
  Recovery_stand,
  Backflip_RESREVE, // backflip, reserve
  Jump_yaw, // jumpYaw, only left direction. Note, to use this mode, you need to
            // set
  Straight_hand //  straightHand. Note, to use this mode, you need to set mode =
                //  1 first
};

struct Path_Movement {
  int trajectory_index;
  int waypoint_index;
  int relative_motiontime;
  std::vector<Ecef_Coord> &waypoints;
  std::vector<Trajectory_Point> trajectories;
};

class Go1_Quadruped {
public:
  Go1_Quadruped()
      : safe(UT::LeggedType::Go1),
        // udp(UT::HIGHLEVEL, 8090, "192.168.123.161", 8082) {
        udp(UT::HIGHLEVEL, 8090, "192.168.12.1", 8082) {
    udp.InitCmdData(cmd);
  }

  UT::Safety safe;
  UT::UDP udp;
  UT::HighState state = {0};
  UT::LowState low_state = {0};

  UT::HighCmd cmd = {0};

  Go1_mode mode = Go1_mode::Force_stand;
  GaitType gait_type = GaitType::Trot;
  uint8_t speed_level = 0;
  float foot_raise_height;
  float body_height;

  void control_loop(Path_Movement &path, Trajectory_Controller &t_controller,
                    Robot_Config &config, std::ofstream &file);
  void UDPRecv();
  void UDPSend();

  int motiontime = 0;
  int relative_motiontime = 0;
  float dt = 0.002; // 0.001~0.01

  // Eigen::Matrix4d go1_config_matrix[4];
  const double trunk_length = 0.3762 / 2;
  const double trunk_width = 0.0935 / 2;
  const double l1 = 0.;
  const double l2 = 0.080; // hip
  const double l3 = 0.213; // thigh
  const double l4 = 0.213; // calf
};

void Go1_Quadruped::UDPRecv() { udp.Recv(); }

void Go1_Quadruped::UDPSend() { udp.Send(); }

UT::HighCmd defaultCmd() {
  UT::HighCmd cmd;
  cmd.footRaiseHeight = 0;
  cmd.bodyHeight = 0;
  cmd.euler[0] = 0;
  cmd.euler[1] = 0;
  cmd.euler[2] = 0;
  cmd.velocity[0] = 0.0f;
  cmd.velocity[1] = 0.0f;
  cmd.yawSpeed = 0.0f;
  cmd.reserve = 0;
  return cmd;
}

UT::HighCmd run_through(UT::HighCmd &cmd, Trajectory_Point &trajectory) {
  //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[0] = trajectory.velocity.linear.x();
  cmd.yawSpeed = trajectory.velocity.angular.z();
  // // cmd.velocity[0] = 0.2;
  // cmd.yawSpeed = 0.3;
  // cmd.footRaiseHeight = 0.1;
  return cmd;
}

void Go1_Quadruped::control_loop(Path_Movement &path,
                                 Trajectory_Controller &t_controller,
                                 Robot_Config &config, std::ofstream &file) {
  udp.GetRecv(state);
  // TODO have this folded into Trajectory_Controller
  if (path.trajectory_index >= path.trajectories.size()) {
    // if (path.waypoint_index >= path.waypoints.size()) {
    //   path.waypoint_index = 0;
    // }
    path.trajectories = t_controller.generate_trajectory(
        path.waypoints[path.waypoint_index],
        path.waypoints[path.waypoint_index + 1], config);
    relative_motiontime = 0;
    path.trajectory_index = 0;
    path.waypoint_index = (path.waypoint_index + 1) % path.waypoints.size();
  }
  if (path.trajectories[path.trajectory_index].dt <
      ((double)relative_motiontime / 1000))
    path.trajectory_index++;

  cmd = run_through(cmd, path.trajectories[path.trajectory_index]);

  file << "DT: " << motiontime << " VEL: " << cmd.velocity[0]
       << " YAW: " << cmd.yawSpeed << " | "
       << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed
       << std::endl;

  udp.SetSend(cmd);
  motiontime += 2;
  relative_motiontime += 2;
}

int main(void) {
  std::cout << "Robot level set to: HIGH" << std::endl
            << "WARNING: Make sure the robot is standing on the ground."
            << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  Go1_Quadruped robot;

  // std::vector<Ecef_Coord> waypoints = {
  //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
  //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
  //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
  //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

  std::vector<Ecef_Coord> waypoints_square = {
      {0.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {2.0, 2.0, 0.0},
      {0.0, 2.0, 0.0},
  };

  std::vector<Ecef_Coord> waypoints_circle = {
      {1.5, 0.0, 0.0},       {1.37, 0.61, 0.0},    {1.004, 1.115, 0.0},
      {0.464, 1.427, 0.0},   {-0.157, 1.492, 0.0}, {-0.75, 1.299, 0.0},
      {-1.214, 0.882, 0.0},  {-1.467, 0.312, 0.0}, {-1.467, -0.312, 0.0},
      {-1.214, -0.882, 0.0}, {-0.75, -1.299, 0.0}, {-0.157, -1.492, 0.0},
      {0.464, -1.427, 0.0},  {1.004, -1.115, 0.0}, {1.37, -0.61, 0.0},
      {1.5, -0.0, 0.0}};

  std::vector<Ecef_Coord> waypoints_eight = {
      {1.5, 1.5, 0.0},    {2.074, 2.03, 0.0}, {2.561, 2.25, 0.0},
      {2.886, 2.03, 0.0}, {3.0, 1.5, 0.0},    {2.886, 0.97, 0.0},
      {2.561, 0.75, 0.0}, {2.074, 0.97, 0.0}, {1.5, 1.5, 0.0},
      {0.926, 2.03, 0.0}, {0.439, 2.25, 0.0}, {0.114, 2.03, 0.0},
      {0.0, 1.5, 0.0},    {0.114, 0.97, 0.0}, {0.439, 0.75, 0.0},
      {0.926, 0.97, 0.0}, {1.5, 1.5, 0.0}};

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 0.6,
                                                .standing_turn_velocity = 2.0,
                                                .max_acceleration = 100.0,
                                                .max_deceleration = 100.0,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };

  Velocity_Profile vel_profile = {.acceleration_rate = 200.0,
                                  .deceleration_rate = 200.0};
  // PIDGains linear_gains = {1.0, 0.0, 0.0};
  // PIDController linear_pid(linear_gains);
  // PIDGains angular_gains = {1.0, 0.0, 0.0};
  // PIDController angular_pid(angular_gains);
  // Trajectory_Controller controller(config.motion_constraints, vel_profile,
  //                                  linear_pid, angular_pid, config.hz);
  Trajectory_Controller controller(config.motion_constraints, vel_profile,
                                   config.hz);
  Path_Movement path_movement = {.trajectory_index = 0,
                                 .waypoint_index = 0,
                                 .relative_motiontime = 0,
                                 .waypoints = waypoints_square,
                                 .trajectories = {}};

  std::vector<Trajectory_Point> trajectories = controller.generate_trajectory(
      waypoints_square[0], waypoints_square[1], config);
  std::cout << "T SIZE: " << trajectories.size() << std::endl;

  // saveToFile("trajectories", trajectories);
  std::ofstream info("go1_info");

  UT::LoopFunc loop_control("control_loop", robot.dt,
                            boost::bind(&Go1_Quadruped::control_loop, &robot,
                                        path_movement, controller, config,
                                        boost::ref(info)));
  UT::LoopFunc loop_udpSend("udp_send", robot.dt, 3,
                            boost::bind(&Go1_Quadruped::UDPRecv, &robot));
  UT::LoopFunc loop_udpRecv("udp_recv", robot.dt, 3,
                            boost::bind(&Go1_Quadruped::UDPSend, &robot));

  loop_udpSend.start();
  loop_udpRecv.start();
  loop_control.start();

  while (1) {
    sleep(10);
  }

  return 0;
}
