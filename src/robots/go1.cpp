#include "go1.hpp"
#include <math.h>
#include <stdint.h>

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
    float vel_x = static_cast<float>(velocity.linear_vel.x());
    float vel_yaw = static_cast<float>(velocity.angular_vel.z());
    writeToStruct<Go1_mode>(cmdBytes, HighCmdOffset::Mode, Go1_mode::Target_velocity_walking);
    writeToStruct<GaitType>(cmdBytes, HighCmdOffset::GaitType, GaitType::Trot);
    writeToStruct<float>(cmdBytes, HighCmdOffset::VelocityX, vel_x);
    writeToStruct<float>(cmdBytes, HighCmdOffset::YawSpeed, vel_yaw);
    return cmd;
}

void
go1_send_velocity_command(void* ctx, Velocity2d &velocity)
{
    Go1* go1 = (Go1*)ctx;
    go1->moveCmd(velocity);
    go1->udp.SetSend(go1->cmd);
};

LA
go1_read_state(void* ctx)
{
    Go1* go1 = (Go1*)ctx;
    const uint8_t *stateBytes = reinterpret_cast<const uint8_t *>(&go1->state);
    go1->udp.GetRecv(go1->state);
    LA ps;

    ps.linear.velocity.x() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityX);
    // ps.velocity.linear.y() = readFromStruct<float>(stateBytes, HighStateOffset::VelocityY);
    ps.angular.velocity.z() = readFromStruct<float>(stateBytes, HighStateOffset::YawSpeed);
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

void
go1_init(void* ctx, const Robot* robot){
    Go1* go1 = (Go1*)ctx;
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    go1->loop_udpSend = new UT::LoopFunc("udp_send", (float)(1.0 / robot->config.control_loop_hz), 3,
        boost::bind(&Go1::UDPRecv, (Go1*)ctx));
    go1->loop_udpRecv = new UT::LoopFunc("udp_recv", (float)(1.0 / robot->config.control_loop_hz), 3,
        boost::bind(&Go1::UDPSend, (Go1*)ctx));

    go1->loop_udpSend->start();
    go1->loop_udpRecv->start();
}

void
go1_deinit(void* ctx){
    Go1* go1 = (Go1*)ctx;

    go1->loop_udpSend->shutdown();
    go1->loop_udpRecv->shutdown();
}
