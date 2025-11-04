#include "go1.hpp"
#include "frames.hpp"
#include "linear_controller.hpp"
#include "load_config.hpp"
#include "path_planner.hpp"
#include "server.hpp"
#include "sim.hpp"
#include <iostream>
#include <math.h>
#include <stdint.h>
#include "control_loop.cpp"

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
Go1::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
};

LA
read_state(void* ctx)
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

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Go1 robot;
    load_config(robot, "robot_configs/go1.json");
    robot.read_pv = *read_state;

    Server server;
    server_init(server, robot);

    Path_Planner p_planner;
    Path_Cursor p_cursor; 
    Path path = read_json_latlon(robot.config.path_config.filepath); // NEW
    path.set_looping(true);
    p_planner.global_cursor = &p_cursor;

    p_cursor.initialize(&path);
    p_planner.path_direction = robot.config.path_config.direction;
    // }

    frames_init(robot.frames, p_planner.global_cursor->path->waypoint(p_planner.global_cursor->current_waypoint),
                p_planner.global_cursor->get_next_waypoint());

    UT::LoopFunc loop_udpSend("udp_send", (float)(1.0 / robot.config.control_loop_hz), 3,
        boost::bind(&Go1::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", (float)(1.0 / robot.config.control_loop_hz), 3,
        boost::bind(&Go1::UDPSend, &robot));

    {

        std::cout << "ROBOT INIT!" << std::endl;
        bool ublox_start = robot.ublox.start();
        std::cout << "UBLOX: " << ublox_start << std::endl;
        while (!robot.ublox.imu.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (robot.ublox.imu.has_value()) break;
        }
        update_position(robot.ublox, robot.frames);
        update_heading(robot.ublox, robot.frames);
        p_planner.re_identify_position(robot.frames.local_frame.pos);
        if (!ublox_start) {
            return -1;
        }
    }

    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
        robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
        robot.config.kinematic_constraints.deceleration_max);
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

    // robot.init();

    std::thread control_thread(control_loop, std::ref(robot), std::ref(p_planner), std::ref(traj_controller), std::ref(server));

    // control_loop<Wheelchair>(robot, traj_controller);
    Sim_Display sim = Sim_Display(robot, p_planner);
    sim.display();
    //
    // CloseWindow();

    return 0;
}
