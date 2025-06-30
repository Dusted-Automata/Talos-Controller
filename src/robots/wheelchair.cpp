#include "wheelchair.hpp"
#include "cppmap3d.hh"
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
    std::cout << written << std::endl;
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

    // ============+ TMP ============
    // std::vector<ENU> waypoints = {
    //     { 0, 0, 0 },
    //     { 2, 2, 0 },
    //     { 4, 4, 0 },
    //     { 6, 6, 0 },
    //     { 8, 6, 0 },
    //     { 6, 4, 0 },
    //     { 4, 4, 0 },
    //     { 2, 4, 0 },
    // };

    // std::vector<ENU> waypoints = {
    //     {  0, 0, 0 },
    //     {  4, 0, 0 },
    //     {  8, 4, 0 },
    //     { 12, 8, 0 },
    //     { 16, 8, 0 },
    //     { 12, 4, 0 },
    //     {  8, 4, 0 },
    //     {  4, 4, 0 },
    // };

    // LLH origin = cppmap3d::ecef2geodetic({ 4100154.6326008383, 476355.6958809831, 4846292.543723706 });
    // std::vector<Ecef> transformed;
    // for (auto i : waypoints) {
    //     transformed.push_back(cppmap3d::enu2ecef(i, origin));
    // }

    // robot.path.add_waypoints(transformed);
    // robot.frames.init(transformed);

    // ============+ TMP ============

    // robot.path.read_json_latlon("ecef_points.json");
    robot.path.read_json_latlon("Parkinglot_Loop.json");
    robot.frames.init(robot.path.path_points_all);

    Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);

    robot.start();
    sim.display();

    CloseWindow();

    return 0;
}
