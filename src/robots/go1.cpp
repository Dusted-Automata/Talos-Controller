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
    // ps.velocity.linear.y() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityY);
    ps.velocity.angular.z() = readFromStruct<float>(stateBytes, HighStateOffset::YawSpeed);
    // UT::IMU imu = readFromStruct<UT::IMU>(stateBytes, HighStateOffset::IMU);
    // UT::MotorState ms = readFromStruct<UT::MotorState>(stateBytes, HighStateOffset::MotorState);
    // std::array<int16_t, 4> footForce = readFromStruct<std::array<int16_t, 4>>(stateBytes,
    // HighStateOffset::FootForce);
    // std::array<int16_t, 3> position = readFromStruct<std::array<int16_t, 3>>(stateBytes, HighStateOffset::Position);

    // std::cout << "x: " << ps.velocity.linear.x() << " | yaw: " << ps.velocity.angular.z() << std::endl;
    // std::cout << "r: " << imu.rpy[0] << " | p: " << imu.rpy[1] << " | y: " << imu.rpy[2] << std::endl;
    // std::cout << "acc: " << imu.accelerometer[0] << " | " << imu.accelerometer[1] << " |  " << imu.accelerometer[2]
    //           << std::endl;
    // std::cout << "gryo: " << imu.gyroscope[0] << " | " << imu.gyroscope[1] << " |  " << imu.gyroscope[2] <<
    // std::endl;
    // std::cout << "mode: " << (int)ms.mode << " | tauEst: " << ms.tauEst << " | temp: " << (int)ms.temperature
    //           << std::endl;
    // std::cout << "q: " << ms.q << " | dq: " << ms.dq << " | ddq: " << ms.ddq << std::endl;
    // std::cout << "q_raw: " << ms.q_raw << " | dq_raw : " << ms.dq_raw << " | ddq_raw: " << ms.ddq_raw << std::endl;
    // std::cout << "footforce: " << footForce[0] << " | " << footForce[1] << " |  " << footForce[2] << " | "
    //           << footForce[3] << std::endl;
    // std::cout << "position: " << position[0] << " | " << position[1] << " |  " << position[2] << std::endl;

    return ps;
}

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::vector<Ecef> waypoints = {
        { 4100157.662065, 476378.631671, 4846296.665580 },
        { 4100148.734711, 476371.224146, 4846299.782664 },
        { 4100145.848885, 476373.137198, 4846301.104139 },
        { 4100149.701858, 476374.962868, 4846304.499274 },
        { 4100151.938404, 476372.478777, 4846293.038250 },
        { 4100164.617288, 476372.803512, 4846292.699499 },
        { 4100166.832613, 476369.785077, 4846290.870528 },
        { 4100164.228392, 476367.548448, 4846291.181639 }
    };

    Go1 robot;
    robot.path.path_looping = true;
    robot.path.add_waypoints(waypoints);
    robot.sensor_manager.init();
    robot.frames.init(waypoints);

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
