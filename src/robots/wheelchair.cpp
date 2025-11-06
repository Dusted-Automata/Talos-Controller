#include "wheelchair.hpp"
#include "types.hpp"
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>


void
wheelchair_init(void* ctx, const Robot* robot)
{
    Wheelchair* wheelchair = (Wheelchair*)ctx;
    wheelchair->kc = robot->config.kinematic_constraints;


#define BAUDRATE B115200
#define TTY "/dev/wheelchair"

    wheelchair->fd = open(TTY, O_RDWR | O_NOCTTY | O_SYNC);

    if (wheelchair->fd < 0) {
        std::cerr << "Error: " << errno << " from open " << strerror(errno) << std::endl;
    }

    termios acm_termios;
    bzero(&acm_termios, sizeof(acm_termios));
    acm_termios.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    acm_termios.c_oflag = ONLCR; // Translate newline to carriage return + newline

    tcflush(wheelchair->fd, TCIFLUSH);
    tcsetattr(wheelchair->fd, TCSANOW, &acm_termios);

    std::array<char, 256> buf;

    std::cout << "Waiting for Setup..." << std::endl;

    bool setup = false;
    std::string incomingMessage = "";

    while (!setup) {
        std::string cmd = "GI,\n";
        ::write(wheelchair->fd, cmd.data(), cmd.size());
        int n = read(wheelchair->fd, buf.data(), sizeof(buf));
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

    std::string cmd = create_command_string(Command_Action::SET, Command_Target::INPUT, "1");
    ::write(wheelchair->fd, cmd.data(), cmd.size());
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
scale_to_joystick(Kinematic_Constraints kc, const Velocity2d &vel)
{
    int8_t scaled_x;
    int8_t scaled_yaw;

    if (vel.linear_vel.x() >= 0) {
        scaled_x = static_cast<int8_t>(
            std::min(100.0, (vel.linear_vel.x() / kc.velocity_forward_max) * 100));
    } else {
        scaled_x = static_cast<int8_t>(
            std::abs((vel.linear_vel.x() / kc.velocity_backward_max) * 100));
    }

    // TODO: Check if the wheelchair even has a different yaw speed, or if it needs to just be scaled with the
    // general velocity max range, and just capped.
    if (-vel.angular_vel.z() >= 0) {
        scaled_yaw = static_cast<int8_t>(
            std::min(100.0, (-vel.angular_vel.z() / kc.velocity_turning_left_max) * 100));
    } else {
        scaled_yaw = -static_cast<int8_t>(
            std::abs((vel.angular_vel.z() / kc.velocity_turning_right_max) * 100));
    }

    Joystick stick = { .x = static_cast<uint8_t>(scaled_yaw), .y = static_cast<uint8_t>(scaled_x) };

    return stick;
}

bool
joystick_to_hex(std::array<char, 10> &buffer, Joystick stick_pos)
{

    size_t written = std::snprintf(buffer.data(), sizeof(buffer), "%02X,%02X", stick_pos.x, stick_pos.y);
    if (written >= buffer.size()) {
        return false;
    }
    return true;
}

void
wheelchair_deinit(void *ctx)
{
    Wheelchair *wheelchair = (Wheelchair *)ctx;
    std::string cmd = create_command_string(Command_Action::SET, Command_Target::JOYSTICK, "0");
    ::write(wheelchair->fd, cmd.data(), cmd.size());
}

void
wheelchair_send_velocity_command(void* ctx, Velocity2d &velocity)
{
    Wheelchair* wheelchair = (Wheelchair*)ctx;
    wheelchair->current_velocity.linear.velocity = velocity.linear_vel; // FOR DEADRECKONING
    wheelchair->current_velocity.angular.velocity = velocity.angular_vel; // FOR DEADRECKONING
    Joystick stick = scale_to_joystick(wheelchair->kc, velocity);
    std::array<char, 10> js_hex;
    if (joystick_to_hex(js_hex, stick)) {
        std::string hex = std::string(js_hex.data());
        std::string cmd = create_command_string(Command_Action::SET, Command_Target::JOYSTICK, hex);
        ::write(wheelchair->fd, cmd.data(), cmd.size());
    } else {
        std::printf("Could not convert joystick to hex!\n");
    }
    // int written = ::write(tty_acm_fd, cmd.to_string().data(), cmd.to_string().size());
    // std::cout << written << std::endl;
}

LA
wheelchair_read_state(void* ctx)
{
    Wheelchair* wheelchair = (Wheelchair*)ctx;
    return wheelchair->current_velocity;
}

