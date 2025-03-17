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
    // Thread_Safe_Queue<Trajectory_Point> trajectories = readPath();
    // Sensors sensors = readSensors();

    mppi_controller.update(pose_state);
    Velocity2d cmd = mppi_controller.getCmd();
    // std::cout << cmd.angular.z() << std::endl;
    // Velocity2d cmd = Controller_Update();
    mppi_controller.shiftControlHorizon();

    // Velocity2d cmd = trajectory_controller.follow_trajectory(trajectory_queue, state);
    send_velocity_command(cmd);
    motiontime += 1000 / hz;

    // will be 0.0 until i fix the state.
    // std::cout << cmd.linear.x() << " , " << cmd.angular.z() << std::endl;
}

// void Robot::updateState() {}
// Robot_State Robot::read_state() {}
void Robot::read_path() {}
// void Robot::read_sensors() {}

void Robot::controller_update()
{
    // // Get control input from MPPI
    // Velocity2d control = mppi_controller.getControl(pose_state);
    //
    // // Apply control to robot
    // QuadrupedModel model;
    // pose_state = model.simulate(pose_state, control, 0.02); // 50Hz update rate
    //
    // // Prepare MPPI for next iteration
    // mppi_controller.shiftControlHorizon();
}
