#include "robot.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"

void
Robot::control_loop()
{
    pose_state = read_state();
    logger.savePosesToFile(frame_controller);
    logger.saveTimesToFile(motion_time);

    Velocity2d cmd = trajectory_controller->get_cmd();
    send_velocity_command(cmd);
    motion_time += 1.0 / hz;
}
