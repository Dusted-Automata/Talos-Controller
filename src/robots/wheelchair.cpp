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

        bool ublox_start = ublox.start();
        // std::cout << "UBLOX: " << ublox_start << std::endl;
        // double heading_tolerance = 0.5;
        // while (true && ublox_start) {
        //     std::optional<Nav_Att> msg = ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
        //     if (msg.has_value()) {
        //         ublox.consume(Msg_Type::NAV_ATT);
        //         std::cout << "HEADING IS " << msg->heading << std::endl;
        //         if (msg.value().heading < heading_tolerance || msg.value().heading > 360 - heading_tolerance) {
        //             std::cout << "HEADING IS UNDER TOLERANCE!" << std::endl;
        //             std::cout << "---------------------------" << std::endl;
        //             break;
        //         }
        //     }
        // }
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
    ::write(tty_acm_fd, cmd.to_string().data(), cmd.to_string().size());
    // int written = ::write(tty_acm_fd, cmd.to_string().data(), cmd.to_string().size());
    // std::cout << written << std::endl;
}

Pose_State
Wheelchair::read_state()
{
    return pose_state;
}

void
control_loop(Wheelchair &robot, Linear_Controller &controller)
{

    double dt = 1.0 / robot.config.control_loop_hz; // TODO change with real dt
    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    // auto motion_time_start = clock::now();
    std::chrono::milliseconds period(1000 / robot.config.control_loop_hz);

    while (robot.running) { // Control loop
        update_position(robot.ublox, robot.frames);
        update_heading(robot.ublox, robot.frames);
        if (!robot.pause) {
            robot.pose_state = robot.read_state();
            robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
            robot.logger.savePosesToFile(robot.frames);
            Pose target_waypoint = robot.path.next();
            // std::cout << target_waypoint.local_point.raw().transpose() << " | "
            //           << target_waypoint.point.raw().transpose() << std::endl;
            Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

            Vector3d dif = frames_diff(target_waypoint.local_point, robot.frames);
            if (frames_dist(dif) > robot.config.goal_tolerance_in_meters) {
                cmd = controller.get_cmd(robot.pose_state, dif, dt);
            } else {
                controller.motion_profile.reset();
                controller.linear_pid.reset();
                controller.angular_pid.reset();
                if (robot.path.progress()) {
                    Vector3d dif = frames_diff(target_waypoint.local_point, robot.frames);
                    controller.motion_profile.set_setpoint(frames_dist(dif));
                } else {
                    break;
                }
            }
            robot.send_velocity_command(cmd);
        } else {
        }

        next += period;
        std::this_thread::sleep_until(next);

        if (clock::now() > next + period) {
            std::cerr << "control-loop overrun" << std::endl;
            next = clock::now();
        }
    }
}

int
main(void)
{

    Wheelchair robot;

    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.v_max,
        robot.config.kinematic_constraints.a_max, robot.config.kinematic_constraints.v_min,
        robot.config.kinematic_constraints.a_min);
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile,
        robot.config);

    robot.path.path_looping = true;
    robot.path.read_json_latlon("Parkinglot_Loop.json");
    robot.frames.init(robot.path);
    robot.init();

    std::jthread sim_thread(control_loop, std::ref(robot), std::ref(traj_controller));

    Sim_Display sim = Sim_Display(robot, robot.path);
    sim.display();

    CloseWindow();

    return 0;
}
