#include "go1.hpp"
#include "raylib.h"
#include "utils/sim.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <vector>

using steady_clock = std::chrono::steady_clock; // monotonic, no wall‑clock jumps
using ms = std::chrono::milliseconds;
using time_point = steady_clock::time_point;

void
Go1_Quadruped::UDPRecv()
{
    udp.Recv();
}

void
Go1_Quadruped::UDPSend()
{
    udp.Send();
}

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

UT::HighCmd
Go1_Quadruped ::moveCmd(Velocity2d &velocity)
{
    uint8_t *cmdBytes = reinterpret_cast<uint8_t *>(&cmd);
    float vel_x = static_cast<float>(velocity.linear.x());
    float vel_yaw = static_cast<float>(velocity.angular.z());
    writeToStruct<uint8_t>(cmdBytes, HighCmdOffset::Mode, 2);
    writeToStruct<uint8_t>(cmdBytes, HighCmdOffset::GaitType, 1);
    writeToStruct<float>(cmdBytes, HighCmdOffset::VelocityX, vel_x);
    writeToStruct<float>(cmdBytes, HighCmdOffset::YawSpeed, vel_yaw);
    return cmd;
}

void
Go1_Quadruped::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
};

Pose_State
Go1_Quadruped::read_state()
{
    const uint8_t *stateBytes = reinterpret_cast<const uint8_t *>(&state);
    udp.GetRecv(state);
    Pose_State ps;

    ps.velocity.linear.x() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityX);
    ps.velocity.linear.y() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityY);
    ps.velocity.angular.z() = readFromStruct<float>(stateBytes, HighStateOffset::YawSpeed);

    return ps;
}

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::vector<Ecef_Coord> waypoints = {
        {  4100175.625135626, 476368.7899695045, 4846344.356704135 },
        { 4100209.6729529747, 476361.2681338759, 4846316.478097512 },
        { 4100218.5394949187, 476445.5598077707, 4846300.796185957 },
        {   4100241.72195791, 476441.0557096391, 4846281.753675706 }
    };

    std::vector<Ecef_Coord> waypoints_square = {
        { 0.0, 0.0, 0.0 },
        { 2.0, 0.0, 0.0 },
        { 2.0, 2.0, 0.0 },
        { 0.0, 2.0, 0.0 },
    };

    std::vector<Ecef_Coord> waypoints_circle = {
        {    1.5,    0.0, 0.0 },
        {   1.37,   0.61, 0.0 },
        {  1.004,  1.115, 0.0 },
        {  0.464,  1.427, 0.0 },
        { -0.157,  1.492, 0.0 },
        {  -0.75,  1.299, 0.0 },
        { -1.214,  0.882, 0.0 },
        { -1.467,  0.312, 0.0 },
        { -1.467, -0.312, 0.0 },
        { -1.214, -0.882, 0.0 },
        {  -0.75, -1.299, 0.0 },
        { -0.157, -1.492, 0.0 },
        {  0.464, -1.427, 0.0 },
        {  1.004, -1.115, 0.0 },
        {   1.37,  -0.61, 0.0 },
        {    1.5,   -0.0, 0.0 }
    };

    std::vector<Ecef_Coord> waypoints_eight = {
        {   1.5,  1.5, 0.0 },
        { 2.074, 2.03, 0.0 },
        { 2.561, 2.25, 0.0 },
        { 2.886, 2.03, 0.0 },
        {   3.0,  1.5, 0.0 },
        { 2.886, 0.97, 0.0 },
        { 2.561, 0.75, 0.0 },
        { 2.074, 0.97, 0.0 },
        {   1.5,  1.5, 0.0 },
        { 0.926, 2.03, 0.0 },
        { 0.439, 2.25, 0.0 },
        { 0.114, 2.03, 0.0 },
        {   0.0,  1.5, 0.0 },
        { 0.114, 0.97, 0.0 },
        { 0.439, 0.75, 0.0 },
        { 0.926, 0.97, 0.0 },
        {   1.5,  1.5, 0.0 }
    };

    Go1_Quadruped robot;
    robot.path_controller.path_looping = true;
    robot.path_controller.add_waypoints(waypoints);
    robot.path_controller.start();
    robot.sensor_manager.init();
    robot.frames.init(robot.path_controller.path_points_all.front());

    UT::LoopFunc loop_control("control_loop", (float)(1.0 / robot.hz), 3,
        boost::bind(&Go1_Quadruped::control_loop, &robot));
    UT::LoopFunc loop_udpSend("udp_send", (float)(1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", (float)(1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPSend, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    Sim_Display sim = Sim_Display(robot, waypoints);

    sim.display();

    CloseWindow();

    return 0;
}
