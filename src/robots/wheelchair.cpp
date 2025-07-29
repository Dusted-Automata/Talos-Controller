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
#define TTY "/dev/ttyACM1"

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

    std::cout << "Waiting for 'IN,Setup' message..." << std::endl;

    bool setup = false;
    std::string incomingMessage = "";

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

    {

        std::cout << "ROBOT INIT!" << std::endl;
        bool ublox_start = ublox.start();
        std::cout << "UBLOX: " << ublox_start << std::endl;
        double heading_tolerance = 0.5;
        while (true && ublox_start) {
            std::optional<Nav_Att> msg = ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
            if (msg.has_value()) {
                std::cout << "HEADING IS " << msg->heading << std::endl;
                if (msg.value().heading < heading_tolerance || msg.value().heading > 360 - heading_tolerance) {
                    std::cout << "HEADING IS UNDER TOLERANCE!" << std::endl;
                    std::cout << "---------------------------" << std::endl;
                    break;
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }
        running = true;
    }

    Command set_cmd(Command_Action::SET, Command_Target::INPUT, "1");
    ::write(tty_acm_fd, set_cmd.to_string().data(), set_cmd.to_string().size());
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
    // std::cout << written << std::endl;
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

void
control_loop(Wheelchair &robot, Linear_Controller &controller, double dt)
{
    while (robot.running) { // Control loop
        while (!robot.pause && robot.running) {
            robot.pose_state = robot.read_state();
            robot.ublox.update_speed(robot.pose_state.velocity); // Currently blocking!!
            robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
            robot.logger.savePosesToFile(robot.frames);
            // robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() -
            // motion_time_start).count());

            Velocity2d cmd = controller.get_cmd(robot, dt);
            robot.send_velocity_command(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * dt)));
        }
    }
}
int
main(void)
{

    Wheelchair robot;

    double dt = 1.0 / robot.config.control_loop_hz; // TODO change with real dt
    PIDGains linear_gains = { 1.2, 0.05, 0.15 };
    PIDGains angular_gains = { 1.0, 0.01, 0.25 };
    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.v_max,
        robot.config.kinematic_constraints.a_max, robot.config.kinematic_constraints.v_min,
        robot.config.kinematic_constraints.a_min);
    Linear_Controller traj_controller(linear_gains, angular_gains, linear_profile, robot.config);

    robot.path.path_looping = true;
    robot.path.read_json_latlon("Parkinglot_Loop.json");
    robot.frames.init(robot.path);

    std::jthread sim_thread(control_loop, std::ref(robot), std::ref(traj_controller), dt);

    Sim_Display sim = Sim_Display(robot, robot.path);
    sim.display();

    CloseWindow();

    return 0;
}
