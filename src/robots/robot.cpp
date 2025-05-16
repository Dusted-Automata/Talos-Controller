#include "robot.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"

void
Robot::control_loop()
{
    pose_state = read_state();
    pose_state.velocity.linear *= 1.0 / hz;
    pose_state.velocity.angular *= 1.0 / hz;
    sensor_manager.consume();
    frames.move_in_local_frame(pose_state.velocity);
    logger.savePosesToFile(frames);
    logger.saveTimesToFile(motion_time);

    Velocity2d cmd = trajectory_controller->get_cmd();
    send_velocity_command(cmd);
    motion_time += 1.0 / hz;
}
