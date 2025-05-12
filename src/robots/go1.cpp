#include "go1.hpp"
#include "raylib.h"
#include "utils/sim.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <vector>

void
Go1::UDPRecv()
{
    udp.Recv();
}

void
Go1::UDPSend()
{
    udp.Send();
}

UT::HighCmd
Go1::moveCmd(const Velocity2d &velocity)
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
Go1::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
};

Pose_State
Go1::read_state()
{
    const uint8_t *stateBytes = reinterpret_cast<const uint8_t *>(&state);
    udp.GetRecv(state);
    Pose_State ps;

    ps.velocity.linear.x() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityX);
    ps.velocity.linear.y() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityY);
    ps.velocity.angular.z() = readFromStruct<float>(stateBytes, HighStateOffset::YawSpeed);

    std::cout << "x: " << ps.velocity.linear.x() << " | yaw: " << ps.velocity.angular.z() << std::endl;

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

    Go1 robot;
    robot.path_controller.path_looping = true;
    robot.path_controller.add_waypoints(waypoints);
    robot.path_controller.start();
    robot.sensor_manager.init();
    robot.frames.init(robot.path_controller.path_queue.front());
    robot.frames.init(robot.path_controller.path_queue.front_two());

    UT::LoopFunc loop_control("control_loop", (float)(1.0 / robot.hz), 3, boost::bind(&Go1::control_loop, &robot));
    UT::LoopFunc loop_udpSend("udp_send", (float)(1.0 / robot.hz), 3, boost::bind(&Go1::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", (float)(1.0 / robot.hz), 3, boost::bind(&Go1::UDPSend, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    Sim_Display sim = Sim_Display(robot, waypoints);

    sim.display();

    CloseWindow();

    return 0;
}
