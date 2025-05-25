#include "wheelchair.hpp"
#include "sim.hpp"
#include "types.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

void
Wheelchair::init()
{

    serial_port = open("/dev/ttyACM0", O_RDWR);

    if (serial_port < 0) {
        std::cerr << "Error: " << errno << " from open " << strerror(errno) << std::endl;
    }

    termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error: " << errno << " from tcgetattr " << strerror(errno) << std::endl;
    }
}

Joystick
Wheelchair::scale_to_joystick(const Velocity2d &vel)
{
    double scaled_x = vel.linear.x();
    double scaled_yaw = vel.angular.y();

    if (vel.linear.x() >= 0) {
        scaled_x = std::min(100.0, (vel.linear.x() / config.kinematic_constraints.v_max) * 100);
    } else {
        double scale = std::abs((vel.linear.x() / config.kinematic_constraints.v_min) * 100);
        scaled_x = 0x9B + scale;
        if (static_cast<uint8_t>(scaled_yaw) == 0x9B) {
            scaled_x = 0;
        }
    }

    // TODO: Check if the wheelchair even has a different yaw speed, or if it needs to just be scaled with the
    // general velocity max range, and just capped.
    if (vel.angular.z() >= 0) {
        scaled_yaw = std::min(100.0, (vel.angular.z() / config.kinematic_constraints.omega_max) * 100);
    } else {
        double scale = std::abs((vel.angular.z() / config.kinematic_constraints.omega_min) * 100);
        scaled_yaw = 0x9B + scale;
        if (static_cast<uint8_t>(scaled_yaw) == 0x9B) {
            scaled_yaw = 0;
        }

        std::cout << scaled_yaw << std::endl;
    }

    Joystick stick = { .x = static_cast<uint8_t>(scaled_x), .y = static_cast<uint8_t>(scaled_yaw) };

    return stick;
}

std::string
Wheelchair::joystick_to_hex(Joystick stick_pos)
{
    std::string hex = std::format("{:02X},{:02X}", stick_pos.x, stick_pos.y);
    return hex;
}

void
Wheelchair::send_velocity_command(Velocity2d &velocity)
{
    pose_state.velocity = velocity; // FOR DEADRECKONING
    Joystick stick = scale_to_joystick(velocity);
    std::string js_hex = joystick_to_hex(stick);
    Command cmd(Command_Action::SET, Command_Target::JOYSTICK, js_hex);
    std::cout << "vel: " << velocity.linear.x() << "|" << velocity.angular.z() << " " << cmd.to_string() << std::endl;
}

Pose_State
Wheelchair::read_state()
{
    return pose_state;
}

int
main(void)
{
    Wheelchair robot;
    robot.path.path_looping = true;
    robot.sensor_manager.init();
    robot.path.read_json_latlon("ecef_points.json");
    robot.frames.init(robot.path.path_points_all);

    Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);

    robot.start();
    sim.display();

    CloseWindow();

    return 0;
}
