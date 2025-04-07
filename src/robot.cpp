#include "robot.hpp"
#include "types.hpp"

void Robot::control_loop()
{
    update_state();
    pose_state = read_state();
    logger.savePosesToFile(pose_state);
    logger.saveTimesToFile(motion_time);
    // Path_Movement path = readPath();
    // Sensors sensors = readSensors();

    Velocity2d cmd = trajectory_controller.get_cmd(pose_state, path_queue);
    send_velocity_command(cmd);
    /*motion_time += 1000 / hz;*/
    motion_time += 1 / hz;
}

void Robot::read_path() {}
