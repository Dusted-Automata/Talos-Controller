#pragma once
#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "linear_controller.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "unitree_legged_sdk/comm.h"
#include "utils/types.hpp"
#include <stdint.h>

enum class GaitType : uint8_t { Idle, Trot, Climb_stair, Trot_obstacle };
enum class Go1_mode : uint8_t {
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

class Go1 : public Robot
{
    UT::HighCmd moveCmd(const Velocity2d &trajectory);

  public:
    Go1()
        : safe(UT::LeggedType::Go1),
          // udp(UT::UDP(UT::HIGHLEVEL, 8090, "192.168.123.161", 8082))
          udp(UT::UDP(UT::HIGHLEVEL, 8090, "192.168.12.1", 8082))
    {
        udp.InitCmdData(cmd);

        pose_state.position = Eigen::Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        Robot_Config config = {
            .control_loop_hz = 500,
            .kinematic_constraints =
            {
                .v_max = 0.5,
                .v_min = 0.0,
                .omega_max = 2.5,
                .omega_min = -2.5,
                .a_max = 100.0,
                .a_min = -100.0,
                .j_max = 0.0,
            },
        };

        PIDGains linear_gains = { 0.8, 0.05, 0.15 };
        LinearPID linear_pid(config, linear_gains);
        PIDGains angular_gains = { 1.0, 0.01, 0.25 };
        AngularPID angular_pid(config, angular_gains);
        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid);
        trajectory_controller->robot = this;
    }
    ~Go1() = default;

    UT::Safety safe;
    UT::UDP udp;
    UT::HighState state = {};
    UT::LowState low_state = {};

    UT::HighCmd cmd = {};

    Go1_mode mode = Go1_mode::Force_stand;
    GaitType gait_type = GaitType::Trot;
    uint8_t speed_level = 0;
    float foot_raise_height;
    float body_height;

    void UDPRecv();
    void UDPSend();
    void send_velocity_command(Velocity2d &velocity) override;
    Pose_State read_state() override;

    double dt = 0.002; // 0.001~0.01

    // Eigen::Matrix4d go1_config_matrix[4];
    const double trunk_length = 0.3762 / 2;
    const double trunk_width = 0.0935 / 2;
    const double l1 = 0.;
    const double l2 = 0.080; // hip
    const double l3 = 0.213; // thigh
    const double l4 = 0.213; // calf
};

namespace HighCmdOffset
{
enum : size_t {
    // Basic fields
    Head = 0,
    LevelFlag = 2,
    FrameReserve = 3,
    SN = 4,
    Version = 12,
    BandWidth = 20,
    Mode = 22,
    GaitType = 23,
    SpeedLevel = 24,
    FootRaiseHeight = 25,
    BodyHeight = 29,
    Position = 33,
    Euler = 41,
    Velocity = 53,
    YawSpeed = 61,
    Bms = 65,
    Led = 69,
    WirelessRemote = 81,
    Reserve = 121,
    Crc = 125,

    // Individual array elements
    VelocityX = Velocity,
    VelocityY = Velocity + sizeof(float),
    PositionX = Position,
    PositionY = Position + sizeof(float),
    EulerRoll = Euler,
    EulerPitch = Euler + sizeof(float),
    EulerYaw = Euler + sizeof(float) * 2
};
}

namespace HighStateOffset
{
enum : size_t {
    // Basic fields
    Head = 0,                                                                  // std::array<uint8_t, 2>
    LevelFlag = Head + sizeof(std::array<uint8_t, 2>),                         // uint8_t
    FrameReserve = LevelFlag + sizeof(uint8_t),                                // uint8_t
    SN = FrameReserve + sizeof(uint8_t),                                       // std::array<uint32_t, 2>
    Version = SN + sizeof(std::array<uint32_t, 2>),                            // std::array<uint32_t, 2>
    BandWidth = Version + sizeof(std::array<uint32_t, 2>),                     // uint16_t
    IMU = BandWidth + sizeof(uint16_t),                                        // IMU structure
    MotorState = IMU + sizeof(UT::IMU),                                        // std::array<MotorState, 20>
    BmsState = MotorState + sizeof(std::array<UT::MotorState, 20>),            // BmsState structure
    FootForce = BmsState + sizeof(UT::BmsState),                               // std::array<int16_t, 4>
    FootForceEst = FootForce + sizeof(std::array<int16_t, 4>),                 // std::array<int16_t, 4>
    Mode = FootForceEst + sizeof(std::array<int16_t, 4>),                      // uint8_t
    Progress = Mode + sizeof(uint8_t),                                         // float
    GaitType = Progress + sizeof(float),                                       // uint8_t
    FootRaiseHeight = GaitType + sizeof(uint8_t),                              // float
    Position = FootRaiseHeight + sizeof(float),                                // std::array<float, 3>
    BodyHeight = Position + sizeof(std::array<float, 3>),                      // float
    Velocity = BodyHeight + sizeof(float),                                     // std::array<float, 3>
    YawSpeed = Velocity + sizeof(std::array<float, 3>),                        // float
    RangeObstacle = YawSpeed + sizeof(float),                                  // std::array<float, 4>
    FootPosition2Body = RangeObstacle + sizeof(std::array<float, 4>),          // std::array<Cartesian, 4>
    FootSpeed2Body = FootPosition2Body + sizeof(std::array<UT::Cartesian, 4>), // std::array<Cartesian, 4>
    WirelessRemote = FootSpeed2Body + sizeof(std::array<UT::Cartesian, 4>),    // std::array<uint8_t, 40>
    Reserve = WirelessRemote + sizeof(std::array<uint8_t, 40>),                // uint32_t
    Crc = Reserve + sizeof(uint32_t),                                          // uint32_t

    // Individual array elements
    PositionX = Position,                     // Position[0]
    PositionY = Position + sizeof(float),     // Position[1]
    PositionZ = Position + sizeof(float) * 2, // Position[2]

    VelocityX = Velocity,                     // Velocity[0]
    VelocityY = Velocity + sizeof(float),     // Velocity[1]
    VelocityZ = Velocity + sizeof(float) * 2, // Velocity[2]

    EulerRoll = IMU + offsetof(UT::IMU, rpy), // IMU.rpy[0]
    EulerPitch = EulerRoll + sizeof(float),   // IMU.rpy[1]
    EulerYaw = EulerPitch + sizeof(float),    // IMU.rpy[2]

};
}

template<typename T>
void
writeToStruct(uint8_t *bytes, size_t offset, const T &value)
{
    memcpy(bytes + offset, &value, sizeof(T));
}

template<typename T>
T
readFromStruct(const uint8_t *bytes, size_t offset)
{
    T result;
    memcpy(&result, bytes + offset, sizeof(T));
    return result;
}
