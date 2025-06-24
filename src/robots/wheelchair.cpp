#include "wheelchair.hpp"
#include "linear_controller.hpp"
#include "motion_profile.hpp"
#include "pid.hpp"
#include "sim.hpp"
#include "types.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
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

    bool setup = false;

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
    int8_t scaled_x;
    int8_t scaled_yaw;

    if (vel.linear.x() >= 0) {
        scaled_x = static_cast<int8_t>(std::min(100.0, (vel.linear.x() / config.kinematic_constraints.v_max) * 100));
    } else {
        scaled_x = static_cast<int8_t>(std::abs((vel.linear.x() / config.kinematic_constraints.v_min) * 100));
    }

    // TODO: Check if the wheelchair even has a different yaw speed, or if it needs to just be scaled with the
    // general velocity max range, and just capped.
    if (-vel.angular.z() >= 0) {
        scaled_yaw = static_cast<int8_t>(
            std::min(100.0, (-vel.angular.z() / config.kinematic_constraints.omega_max) * 100));
    } else {
        scaled_yaw = -static_cast<int8_t>(std::abs((vel.angular.z() / config.kinematic_constraints.omega_min) * 100));
    }

    Joystick stick = { .x = static_cast<uint8_t>(scaled_yaw), .y = static_cast<uint8_t>(scaled_x) };

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

    // std::cout << "vel: " << velocity.linear.x() << "|" << velocity.angular.z() << " " << cmd.to_string() <<
    // std::endl;
    ::read(tty_acm_fd, tty_read_buf.data(), tty_read_buf.size());
    // std::cout << tty_read_buf.data() << std::endl;
}

Pose_State
Wheelchair::read_state()
{
    return pose_state;
}

// inline void
// c_loop(Robot &robot, Trajectory_Controller &trajectory_controller, double dt)
// {
//     robot.pose_state = robot.read_state();
//     robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
//     robot.logger.savePosesToFile(robot.frames);
//     // robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());
//
//     Velocity2d cmd = trajectory_controller.get_cmd();
//     robot.send_velocity_command(cmd);
// }

int
main(void)
{

    Wheelchair robot;

    double dt = 1.0 / robot.config.control_loop_hz; // TODO change with real dt
    PIDGains linear_gains = { 0.8, 0.05, 0.15 };
    PIDGains angular_gains = { 1.0, 0.01, 0.25 };
    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.v_max,
        robot.config.kinematic_constraints.a_max, robot.config.kinematic_constraints.v_min,
        robot.config.kinematic_constraints.a_min);
    Linear_Controller traj_controller(linear_gains, angular_gains, linear_profile, robot.config);

    robot.path.path_looping = true;
    robot.path.read_json_latlon("ecef_points.json");
    robot.frames.init(robot.path.path_points_all);

    Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);
    std::jthread sim_thread(&Sim_Display::display, sim);

    while (robot.running) { // Control loop
        while (!robot.pause && robot.running) {
            robot.pose_state = robot.read_state();
            robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
            robot.logger.savePosesToFile(robot.frames);
            // robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            Velocity2d cmd = traj_controller.get_cmd(robot, dt);
            robot.send_velocity_command(cmd);
        }
    }

    CloseWindow();

    return 0;
}
