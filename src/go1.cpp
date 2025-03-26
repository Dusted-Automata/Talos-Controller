#include "go1.hpp"
#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "mppi.hpp"
#include "trajectory.hpp"
#include "unitree_legged_sdk/comm.h"
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <vector>

void Go1_Quadruped::UDPRecv() { udp.Recv(); }

void Go1_Quadruped::UDPSend() { udp.Send(); }

UT::HighCmd defaultCmd()
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

UT::HighCmd Go1_Quadruped ::moveCmd(Velocity2d &velocity)
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

void Go1_Quadruped::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
};

void Go1_Quadruped::update_state(){};
void Go1_Quadruped::read_sensors(){};
Pose_State Go1_Quadruped::read_state()
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

// void Go1_Quadruped::control_loop() {
//
//   udp.GetRecv(state);
//   Velocity2d velocity = {.linear = {}, .angular = {}};
//   UT::HighCmd cmd = moveCmd(velocity);
//   udp.SetSend(cmd);
//   // file << "DT: " << motiontime << " VEL: " << cmd.velocity[0]
//   //      << " YAW: " << cmd.yawSpeed << " | "
//   //      << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed
//   //      << std::endl;
// }

int main(void)
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
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {2.0, 2.0, 0.0},
        {0.0, 2.0, 0.0},
    };

    std::vector<Ecef_Coord> waypoints_circle = {
        {1.5, 0.0, 0.0},       {1.37, 0.61, 0.0},     {1.004, 1.115, 0.0},  {0.464, 1.427, 0.0},
        {-0.157, 1.492, 0.0},  {-0.75, 1.299, 0.0},   {-1.214, 0.882, 0.0}, {-1.467, 0.312, 0.0},
        {-1.467, -0.312, 0.0}, {-1.214, -0.882, 0.0}, {-0.75, -1.299, 0.0}, {-0.157, -1.492, 0.0},
        {0.464, -1.427, 0.0},  {1.004, -1.115, 0.0},  {1.37, -0.61, 0.0},   {1.5, -0.0, 0.0}};

    std::vector<Ecef_Coord> waypoints_eight = {
        {1.5, 1.5, 0.0}, {2.074, 2.03, 0.0}, {2.561, 2.25, 0.0}, {2.886, 2.03, 0.0},
        {3.0, 1.5, 0.0}, {2.886, 0.97, 0.0}, {2.561, 0.75, 0.0}, {2.074, 0.97, 0.0},
        {1.5, 1.5, 0.0}, {0.926, 2.03, 0.0}, {0.439, 2.25, 0.0}, {0.114, 2.03, 0.0},
        {0.0, 1.5, 0.0}, {0.114, 0.97, 0.0}, {0.439, 0.75, 0.0}, {0.926, 0.97, 0.0},
        {1.5, 1.5, 0.0}};

    Velocity_Profile vel_profile = {.acceleration_rate = 200.0, .deceleration_rate = 200.0};
    Robot_Config config = {.hz = 50,
                           .motion_constraints = {.max_velocity = 0.6,
                                                  .standing_turn_velocity = 2.0,
                                                  .max_acceleration = 100.0,
                                                  .max_deceleration = 100.0,
                                                  .max_jerk = 0.0,
                                                  .corner_velocity = 0.0},
                           .velocity_profile = vel_profile

    };

    PIDGains linear_gains = {0.4, 0.0, 0.0};
    PIDController linear_pid(linear_gains);
    linear_pid.output_max = 10.0;
    linear_pid.output_min = 0.0;
    PIDGains angular_gains = {0.2, 0.0, 0.0};
    PIDController angular_pid(angular_gains);
    angular_pid.output_max = 10.0;
    angular_pid.output_min = 0.0;
    Trajectory_Controller t_c(config, linear_pid, angular_pid, config.hz);
    MPPI_Controller m_c(50, 1000, 0.02, 0.5);
    // Trajectory_Controller controller(config.motion_constraints, vel_profile,
    //                                  config.hz);

    /*Go1_Quadruped robot(t_c, m_c);*/
    Go1_Quadruped robot(t_c);
    robot.controller.path_looping = true;

    // saveToFile("trajectories", trajectories);
    // std::ofstream info("go1_info");

    // UT::LoopFunc loop_control("control_loop", robot.dt,
    //                           boost::bind(&Go1_Quadruped::control_loop, &robot,
    //                                       path_movement, controller, config,
    //                                       boost::ref(info)));

    UT::LoopFunc loop_control("control_loop", robot.dt,
                              boost::bind(&Go1_Quadruped::control_loop, &robot));

    UT::LoopFunc path_loop("path_loop", 0.030,
                           boost::bind(&Trajectory_Controller::path_loop, &t_c,
                                       boost::ref(robot.path_queue), waypoints_square));
    /*UT::LoopFunc traj_loop("traj_loop", 0.030,*/
    /*                       boost::bind(&Trajectory_Controller::trajectory_loop, &t_c,*/
    /*                                   boost::ref(robot.trajectory_queue),*/
    /*                                   boost::ref(robot.path_queue)));*/
    /**/
    UT::LoopFunc loop_udpSend("udp_send", robot.dt, 3,
                              boost::bind(&Go1_Quadruped::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", robot.dt, 3,
                              boost::bind(&Go1_Quadruped::UDPSend, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    path_loop.start();
    /*traj_loop.start();*/
    loop_control.start();

    while (1)
    {
        sleep(10);
    }

    return 0;
}
