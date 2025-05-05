#include "go1.hpp"
#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/comm.h"
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

UT::HighCmd
defaultCmd()
{
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

UT::HighCmd
Go1_Quadruped ::moveCmd(Velocity2d &velocity)
{
    // UT::HighCmd cmd = defaultCmd();
    //
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = velocity.linear.x();
    // cmd.velocity[0] = 0.6;
    cmd.yawSpeed = velocity.angular.z();
    return cmd;
}

void
Go1_Quadruped::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
    frame_controller.move_in_local_frame(velocity);
};

Pose_State
Go1_Quadruped::read_state()
{
    udp.GetRecv(state);
    Pose_State ps;
    // Robot_State s;
    /*ps.orientation.w() = state.imu.quaternion[0];*/
    /*ps.orientation.x() = state.imu.quaternion[1];*/
    /*ps.orientation.y() = state.imu.quaternion[2];*/
    /*ps.orientation.z() = state.imu.quaternion[3];*/
    // ps.position.x() = state.position[0];
    // ps.position.y() = state.position[1];
    // ps.position.z() = state.position[2];
    ps.velocity.linear.x() = state.velocity[0];
    ps.velocity.linear.y() = state.velocity[1];
    ps.velocity.linear.z() = state.velocity[2];
    // ps.velocity.angular.x() = state.position[0];
    // ps.velocity.angular.y() = state.position[1];
    ps.velocity.angular.z() = state.yawSpeed;

    // s.position = state.position;
    // s.velocity = state.velocity;
    // s.yawSpeed = state.yawSpeed;
    return ps;
};

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // std::vector<Ecef_Coord> waypoints = {
    //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
    //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
    //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
    //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

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
    robot.path_controller.add_waypoints(waypoints_square);
    robot.path_controller.start();
    robot.sensor_manager.init();

    UT::LoopFunc loop_udpSend("udp_send", (1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", (1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPSend, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();

    ms PERIOD{ static_cast<std::int64_t>(1000 / robot.hz) };

    while (true) {
        auto start = steady_clock::now();
        time_point next_tick = start + PERIOD;
        robot.control_loop();
        auto finish = steady_clock::now();
        auto execution = finish - start;

        if (finish < next_tick) {
            std::this_thread::sleep_until(next_tick);
        } else {
            std::cerr << " overrunning by: " << finish - next_tick << " ms" << std::endl;
        }
    }

    return 0;
}
