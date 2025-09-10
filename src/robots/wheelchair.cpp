#include "wheelchair.hpp"
#include "linear_controller.hpp"
#include "load_config.hpp"
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
Wheelchair::init(Wheelchair &robot)
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

    std::cout << "Waiting for Setup..." << std::endl;

    bool setup = false;
    std::string incomingMessage = "";

    while (!setup) {
        std::string cmd = "GI,\n";
        ::write(tty_acm_fd, cmd.data(), cmd.size());
        int n = read(tty_acm_fd, buf.data(), sizeof(buf));
        if (n > 0) {
            // std::string msg = std::format("got a message! Size: {} | msg: {}", n, buf.data());
            printf("got a message! Size: %i | msg: %s", n, buf.data());
            // std::cout << msg << std::endl;
            for (int i = 0; i < n; i++) {
                incomingMessage += buf[i];
                if (incomingMessage.find("GI,") != std::string::npos) {
                    setup = true;
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        printf("----\n");
    }

    {

        std::cout << "ROBOT INIT!" << std::endl;
        bool ublox_start = ublox.start();
        std::cout << "UBLOX: " << ublox_start << std::endl;
        // while (true && ublox_start) {
        //     std::optional<Nav_Pvat> msg = ublox.get_latest<Nav_Pvat>(Msg_Type::NAV_PVAT);
        //     if (msg.has_value()) {
        //         std::cout << "VEH_HEADING IS " << msg->veh_heading << std::endl;
        //         std::cout << "MOT_HEADING IS " << msg->mot_heading << std::endl;
        //         heading.initial_heading_in_radians = msg->veh_heading;
        //         Eigen::Matrix3d rotationMatrix;
        //         rotationMatrix = Eigen::AngleAxisd(heading.initial_heading_in_radians, Eigen::Vector3d::UnitZ());
        //         frames.local_frame.orientation = rotationMatrix; // NOTE: To be checked!
        //         break;
        //     }
        // }

        running = true;
    }

    std::string cmd = create_command_string(Command_Action::SET, Command_Target::INPUT, "1");
    ::write(tty_acm_fd, cmd.data(), cmd.size());
}

std::string
create_command_string(Command_Action a, Command_Target t, std::optional<std::string> value)
{
    char action_char = action_to_char(a);
    char target_char = target_to_char(t);
    char buffer[256];
    if (value.has_value()) {
        std::snprintf(buffer, sizeof(buffer), "%c%c,%s\n", action_char, target_char, value.value().data());
    } else {
        std::snprintf(buffer, sizeof(buffer), "%c%c,\n", action_char, target_char);
    }
    return std::string(buffer);
}

Joystick
Wheelchair::scale_to_joystick(const Velocity2d &vel)
{
    int8_t scaled_x;
    int8_t scaled_yaw;

    if (vel.linear_vel.x() >= 0) {
        scaled_x = static_cast<int8_t>(
            std::min(100.0, (vel.linear_vel.x() / config.kinematic_constraints.velocity_forward_max) * 100));
    } else {
        scaled_x = static_cast<int8_t>(
            std::abs((vel.linear_vel.x() / config.kinematic_constraints.velocity_backward_max) * 100));
    }

    // TODO: Check if the wheelchair even has a different yaw speed, or if it needs to just be scaled with the
    // general velocity max range, and just capped.
    if (-vel.angular_vel.z() >= 0) {
        scaled_yaw = static_cast<int8_t>(
            std::min(100.0, (-vel.angular_vel.z() / config.kinematic_constraints.velocity_turning_left_max) * 100));
    } else {
        scaled_yaw = -static_cast<int8_t>(
            std::abs((vel.angular_vel.z() / config.kinematic_constraints.velocity_turning_right_max) * 100));
    }

    Joystick stick = { .x = static_cast<uint8_t>(scaled_yaw), .y = static_cast<uint8_t>(scaled_x) };

    return stick;
}

bool
Wheelchair::joystick_to_hex(std::array<char, 10> &buffer, Joystick stick_pos)
{

    size_t written = std::snprintf(buffer.data(), sizeof(buffer), "%02X,%02X", stick_pos.x, stick_pos.y);
    if (written >= buffer.size()) {
        return false;
    }
    return true;
}

void
Wheelchair::send_velocity_command(Velocity2d &velocity)
{
    pose_state.velocity = velocity; // FOR DEADRECKONING
    Joystick stick = scale_to_joystick(velocity);
    std::array<char, 10> js_hex;
    if (joystick_to_hex(js_hex, stick)) {
        std::string hex = std::string(js_hex.data());
        std::string cmd = create_command_string(Command_Action::SET, Command_Target::JOYSTICK, hex);
        ::write(tty_acm_fd, cmd.data(), cmd.size());
    } else {
        std::printf("Could not convert joystick to hex!\n");
    }
    // int written = ::write(tty_acm_fd, cmd.to_string().data(), cmd.to_string().size());
    // std::cout << written << std::endl;
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
    load_config(robot, "robot_configs/wheelchair_profile_3_bar_3.json");
    robot.path.path_direction = robot.config.path_config.direction;
    robot.path.global_path.read_json_latlon(robot.config.path_config.filepath);
    robot.path.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);

    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
        robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
        robot.config.kinematic_constraints.deceleration_max);
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

    frames_init(robot.frames, robot.path.local_path);
    robot.init(robot);

    std::jthread sim_thread(control_loop<Wheelchair>, std::ref(robot), std::ref(traj_controller));

    Sim_Display sim = Sim_Display(robot, robot.path);
    sim.display();

    CloseWindow();

    return 0;
}
