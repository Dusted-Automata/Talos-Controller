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
        while (true && ublox_start) {
            std::optional<Nav_Pvat> msg = ublox.get_latest<Nav_Pvat>(Msg_Type::NAV_PVAT);
            if (msg.has_value()) {
                std::cout << "VEH_HEADING IS " << msg->veh_heading << std::endl;
                std::cout << "MOT_HEADING IS " << msg->mot_heading << std::endl;
                heading.heading_offset = msg->veh_heading;
                break;
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

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    auto previous_time = clock::now();
    auto motion_time_start = clock::now();
    std::chrono::milliseconds period(1000 / robot.config.control_loop_hz);

    while (robot.running) {               // Control loop
        auto current_time = clock::now(); // Current iteration time
        std::chrono::duration<double> elapsed = current_time - previous_time;
        double dt = elapsed.count(); // `dt` in seconds
        previous_time = current_time;

        update_position(robot.ublox, robot.frames);
        Eigen::AngleAxisd aa(robot.frames.local_frame.orientation.linear());
        double old_heading = aa.angle();
        update_heading(robot.ublox, robot.frames, robot.heading);
        Eigen::AngleAxisd ab(robot.frames.local_frame.orientation.linear());
        double new_heading = ab.angle();
        // std::cout << "old-new heading: " << old_heading << " | " << new_heading << std::endl;
        if (!robot.pause) {
            robot.pose_state = robot.read_state();
            // bool update_speed = robot.ublox.update_speed(robot.pose_state.velocity); // Currently blocking!!
            // std::cout << "ublox_update_speed: " << update_speed << std::endl;
            frames_move_in_local_frame(robot.frames, robot.pose_state.velocity, dt);
            robot.logger.savePosesToFile(robot.frames);
            robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            // Pose target_waypoint = robot.path.global_path.next();
            Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

            Vector3d global_dif = frames_diff(robot.frames, robot.path.global_path.next().local_point);
            Vector3d local_dif = frames_diff(robot.frames, robot.path.path.next().local_point);
            if (frames_dist(global_dif) > robot.config.goal_tolerance_in_meters) {
                cmd = controller.get_cmd(robot.pose_state, global_dif, local_dif, dt);
                // std::cout << "cmd: " << cmd.angular.transpose() << std::endl;
            } else {
                robot.path.global_path.progress();
            }

            // std::cout << "local_dif: " << local_dif.transpose() << std::endl;
            if (frames_dist(local_dif) < robot.config.goal_tolerance_in_meters) {
                controller.motion_profile.reset();
                if (robot.path.path.progress()) {
                    // Vector3d dif = frames_diff(target_waypoint.local_point, robot.frames);
                    controller.motion_profile.set_setpoint(frames_dist(local_dif));
                } else {
                    break;
                }
            }
            robot.send_velocity_command(cmd);
        }

        next += period;
        std::this_thread::sleep_until(next);

        if (clock::now() > next + period) {
            std::cerr << "control-loop overrun" << std::endl;
            next = clock::now();
            previous_time = next;
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
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

    // robot.path.path.path_looping = true;
    robot.path.path.read_json_latlon("Table_Grab.json");
    robot.path.gen_global_path(2.5);
    frames_init(robot.frames, robot.path.path);
    robot.init();

    std::jthread sim_thread(control_loop, std::ref(robot), std::ref(traj_controller));

    Sim_Display sim = Sim_Display(robot, robot.path);
    sim.display();

    CloseWindow();

    return 0;
}
