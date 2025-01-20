#include "include/unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

// using namespace UNITREE_LEGGED_SDK;
#define U0T UNITREE_LEGGED_SDK

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

class Go1_quadruped {
public:
  Go1_quadruped()
      : safe(UT::LeggedType::Go1),
        udp(UT::HIGHLEVEL, 8090, "192.168.123.161", 8082) {
    UT::HighCmd cmd = {0};
    udp.InitCmdData(cmd);
  }

  UT::Safety safe;
  UT::UDP udp;
  UT::HighState state = {0};
  UT::LowState low_state = {0};

  Go1_mode mode = Go1_mode::Force_stand;
  GaitType gait_type = GaitType::Trot;
  uint8_t speed_level = 0;
  float foot_raise_height;
  float body_height;

  void control_loop();

  // Eigen::Matrix4d go1_config_matrix[4];
  const double trunk_length = 0.3762 / 2;
  const double trunk_width = 0.0935 / 2;
  const double l1 = 0.;
  const double l2 = 0.080; // hip
  const double l3 = 0.213; // thigh
  const double l4 = 0.213; // calf
};

// void UDPRecv() {
// 	udp.Recv();
// }
//
// void UDPSend() {
// 	udp.Send();
// }
//

void Go1_quadruped::control_loop() {}

struct Ecef {
  int x;
  int y;
  int z;
};

void run_through(const std::array<Ecef, 5>) {}

int main(void) {
  // std::cout << "Robot level set to: HIGH" << std::endl
  //           << "WARNING: Make sure the robot is standing on the ground."
  //           << std::endl
  //           << "Press Enter to continue..." << std::endl;
  // std::cin.ignore();
  //
  // Go1_quadruped robot;
  //
  // UT::LoopFunc loop_control("control_loop", robot.dt,
  //                           boost::bind(&Go1_quadruped::control_loop,
  //                           &robot));
  // UT::LoopFunc loop_udpSend("udp_send", robot.dt, 3,
  //                           boost::bind(&UT::UDP::Recv, &robot.udp));
  // UT::LoopFunc loop_udpRecv("udp_recv", robot.dt, 3,
  //                           boost::bind(&UT::UDP::Send, &robot.udp));
  //
  // loop_udpSend.start();
  // loop_udpRecv.start();
  // loop_control.start();
  //
  // while (1) {
  //   sleep(10);
  // }
  //
  // return 0;
}
