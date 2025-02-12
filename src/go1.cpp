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

  void control_loop(std::vector<Trajectory_Point> trajectories, int &index,
                    std::ofstream &file);
  void UDPRecv();
  void UDPSend();

  int motiontime = 0;
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

UT::HighCmd run_through(UT::HighCmd &cmd, int dt,
                        Trajectory_Point &trajectory) {
  //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

  cmd.mode = 2;
  cmd.gaitType = 1;
  cmd.velocity[0] = trajectory.velocity.linear.x();
  cmd.yawSpeed = trajectory.velocity.angular.z();
  // // cmd.velocity[0] = 0.2;
  // cmd.yawSpeed = 0.3;
  cmd.footRaiseHeight = 0.1;
  return cmd;
}

void Go1_Quadruped::control_loop(std::vector<Trajectory_Point> trajectories,
                                 int &index, std::ofstream &file) {
  udp.GetRecv(state);
  if (trajectories[index].dt < ((double)motiontime / 1000))
    index++;
  cmd = run_through(cmd, index, trajectories[index]);
  // std::cout << "DT: " << motiontime << " INDEX: " << index << std::endl;
  // std::cout << "DT: " << motiontime
  //           << " VEL: " << trajectories[index].velocity.linear.x()
  //           << " YAW: " << trajectories[index].velocity.angular.z()
  //           << std::endl;
  file << "DT: " << motiontime << " VEL: " << cmd.velocity[0]
       << " YAW: " << cmd.yawSpeed << " | "
       << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed
       << std::endl;

  // std::cout << "S.accel: " << state.mode << std::endl;
  // printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
  udp.SetSend(cmd);
  motiontime += 2;
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

  std::vector<Ecef_Coord> waypoints = {{0.0, 0.0, 0.0},
                                       {2.0, 0.0, 0.0},
                                       {2.0, 2.0, 0.0},
                                       {0.0, 2.0, 0.0},
                                       {0.0, 0.0, 0.0}};

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 0.6,
                                                .standing_turn_velocity = 2.0,
                                                .max_acceleration = 0.5,
                                                .max_deceleration = 0.5,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };

  Velocity_Profile vel_profile = {.acceleration_rate = 35.0,
                                  .deceleration_rate = 10.0};
  // PIDGains linear_gains = {1.0, 0.0, 0.0};
  // PIDController linear_pid(linear_gains);
  // PIDGains angular_gains = {1.0, 0.0, 0.0};
  // PIDController angular_pid(angular_gains);
  // Trajectory_Controller controller(config.motion_constraints, vel_profile,
  //                                  linear_pid, angular_pid, config.hz);
  Trajectory_Controller controller(config.motion_constraints, vel_profile,
                                   config.hz);
  int index = 0;

  std::vector<Trajectory_Point> trajectories =
      controller.generate_trajectory(waypoints, config);

  saveToFile("trajectories", trajectories);
  std::ofstream info("go1_info");

  UT::LoopFunc loop_control("control_loop", robot.dt,
                            boost::bind(&Go1_Quadruped::control_loop, &robot,
                                        trajectories, index, boost::ref(info)));
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
