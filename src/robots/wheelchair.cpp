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

#define BAUDRATE B115200
#define TTY "/dev/ttyACM0"

    tty_acm_fd = open(TTY, O_RDWR | O_NOCTTY | O_SYNC);

    if (tty_acm_fd < 0) {
        std::cerr << "Error: " << errno << " from open " << strerror(errno) << std::endl;
    }

    termios acm_termios;
    bzero(&acm_termios, sizeof(acm_termios));
    acm_termios.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    acm_termios.c_oflag = ONLCR; // Translate newline to carriage return + newline

    tcflush(tty_acm_fd, TCIFLUSH);
    tcsetattr(tty_acm_fd, TCSANOW, &acm_termios);

    std::array<char, 256> buf;
    std::string incomingMessage = "";

    std::cout << "Waiting for 'IN,Setup' message..." << std::endl;

    bool setup = true;
    while (!setup) {
        int n = read(tty_acm_fd, buf.data(), sizeof(buf));
        if (n > 0) {
            std::string msg = std::format("got a message! Size: {} | msg: {}", n, buf.data());
            std::cout << msg << std::endl;
            for (int i = 0; i < n; i++) {
                incomingMessage += buf[i];
                if (incomingMessage.find("IN,Setup") != std::string::npos) {
                    setup = true;
                }
            }
        }
    }

    Command set_cmd(Command_Action::SET, Command_Target::INPUT, "1");
    ::write(tty_acm_fd, set_cmd.to_string().data(), set_cmd.to_string().size());
    Command listen_cmd(Command_Action::LISTEN, Command_Target::JOYSTICK, "1");
    ::write(tty_acm_fd, listen_cmd.to_string().data(), set_cmd.to_string().size());
}

Joystick
Wheelchair::scale_to_joystick(const Velocity2d &vel)
{
    double scaled_x = vel.linear.x();
    double scaled_yaw = -vel.angular.z();

    if (scaled_x >= 0) {
        scaled_x = std::min(100.0, (scaled_x / config.kinematic_constraints.v_max) * 100);
    } else {
        double scale = std::abs((scaled_x / config.kinematic_constraints.v_min) * 100);
        scaled_x = 0x9B + scale;
        if (static_cast<uint8_t>(scaled_x) == 0x9B) {
            scaled_x = 0;
        }
    }

    // TODO: Check if the wheelchair even has a different yaw speed, or if it needs to just be scaled with the
    // general velocity max range, and just capped.
    if (scaled_yaw >= 0) {
        scaled_yaw = std::min(100.0, (scaled_yaw / config.kinematic_constraints.omega_max) * 100);
    } else {
        double scale = std::abs((scaled_yaw / config.kinematic_constraints.omega_min) * 100);
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
    int written = ::write(tty_acm_fd, cmd.to_string().data(), cmd.to_string().size());
    std::cout << written << std::endl;

    std::cout << "vel: " << velocity.linear.x() << "|" << velocity.angular.z() << " " << cmd.to_string() << std::endl;
    ::read(tty_acm_fd, tty_read_buf.data(), tty_read_buf.size());
    std::cout << tty_read_buf.data() << std::endl;
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
