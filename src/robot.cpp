#include "robot.hpp"
#include "types.hpp"
#include <Eigen/Dense>
#include <iostream>

void Robot::control_loop()
{
    update_state();
    pose_state = read_state();
    // std::cout << "DT: " << motiontime << " VEL: " << state.velocity[0] << " YAW: " <<
    // state.yawSpeed
    //           << " | " << std::endl;
    // << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed << std::endl;

    // Path_Movement path = readPath();
    // Sensors sensors = readSensors();
    Velocity2d cmd = controller.get_cmd(pose_state, path_queue);
    /*std::cout << cmd.linear.transpose() << std::endl;*/
    send_velocity_command(cmd);
    motiontime += 1000 / hz;
}

// void Robot::updateState() {}
// Robot_State Robot::read_state() {}
void Robot::read_path() {}
// void Robot::read_sensors() {}
