#include "robot.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"

void
Robot::control_loop()
{
    pose_state = read_state();
    logger.savePosesToFile(frame_controller);
    logger.saveTimesToFile(motion_time);
    // Path_Movement path = readPath();
    // Sensors sensors = readSensors();
    // sensor_manager.readSensors();
    /*sensor_manager.sm_thread.join();*/

    Velocity2d cmd = trajectory_controller->get_cmd();
    send_velocity_command(cmd);
    /*motion_time += 1000 / hz;*/
    motion_time += 1.0 / hz;
}
