#pragma once
#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "mppi.hpp"
#include "robot.hpp"
#include "trajectory.hpp"
#include "unitree_legged_sdk/comm.h"
#include <stdint.h>

// #define BOOST_BIND_GLOBAL_PLACEHOLDERS
//
// // using namespace UNITREE_LEGGED_SDK;
// #define UT UNITREE_LEGGED_SDK

enum class GaitType : uint8_t
{
    Idle,
    Trot,
    Climb_stair,
    Trot_obstacle
};
enum class Go1_mode : uint8_t
{
    Idle,                             // idle, default stand
    Force_stand,                      // force stand (controlled by dBodyHeight + ypr)
    Target_velocity_walking,          // target velocity walking (controlled by velocity +
                                      // yawSpeed)
    Target_position_walking_RESERVED, // target position walking (controlled by
                                      // position + ypr[0]), reserve
    Path_mode_walking_RESERVED,       // path mode walking (reserve for future release),
                                      // reserve
    Stand_down,
    Stand_up,
    Damping_mode,
    Recovery_stand,
    Backflip_RESREVE, // backflip, reserve
    Jump_yaw,         // jumpYaw, only left direction. Note, to use this mode, you need to
                      // set
    Straight_hand     //  straightHand. Note, to use this mode, you need to set mode =
                      //  1 first
};

class Go1_Quadruped : public Robot
{
    UT::HighCmd moveCmd(Velocity2d &trajectory);

  public:
    Go1_Quadruped(Controller &t_c)
        : Robot(t_c),
          // udp(UT::HIGHLEVEL, 8090, "192.168.123.161", 8082) {
          safe(UT::LeggedType::Go1), udp(UT::HIGHLEVEL, 8090, "192.168.12.1", 8082)
    {
        udp.InitCmdData(cmd);
    }
    ~Go1_Quadruped() = default;

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

    void UDPRecv();
    void UDPSend();
    void send_velocity_command(Velocity2d &velocity) override;
    void update_state() override;
    void read_sensors() override;
    Pose_State read_state() override;

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
