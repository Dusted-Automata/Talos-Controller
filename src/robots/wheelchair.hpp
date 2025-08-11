#pragma once
#include "robot.hpp"
#include "ublox.hpp"

enum class Command_Action {
    SET,
    GET,
    LISTEN,
    ADVANCE,
};

enum class Command_Target {
    INPUT,
    JOYSTICK,
    MODE,
    PROFILE,
};

inline char
action_to_char(Command_Action action)
{
    switch (action) {
    case Command_Action::SET: return 'S';
    case Command_Action::GET: return 'G';
    case Command_Action::LISTEN: return 'L';
    case Command_Action::ADVANCE: return 'A';
    default: return '0';
    }
}

inline char
target_to_char(Command_Target target)
{
    switch (target) {
    case Command_Target::INPUT: return 'I';
    case Command_Target::JOYSTICK: return 'J';
    case Command_Target::MODE: return 'M';
    case Command_Target::PROFILE: return 'P';
    default: return '0';
    }
}

struct Command {
    Command_Action action;
    Command_Target target;
    std::optional<std::string> value;

    Command(Command_Action action, Command_Target target, std::string val)
        : action(action), target(target), value(std::move(val))
    {
    }
    Command(Command_Action action, Command_Target target) : action(action), target(target), value(std::nullopt) {}

    std::string
    to_string() const
    {
        char action_char = action_to_char(action);
        char target_char = target_to_char(target);
        if (value.has_value()) {
            return std::format("{}{},{}\n", action_char, target_char, value.value());
        } else {
            return std::format("{}{},\n", action_char, target_char);
        }
    }
};

struct Joystick {
    uint8_t x;
    uint8_t y;
};

class Wheelchair : public Robot
{
  private:
    int tty_acm_fd = -1;
    Joystick scale_to_joystick(const Velocity2d &vel);
    std::string joystick_to_hex(Joystick stick_pos);
    std::array<char, 128> tty_read_buf;

  public:
    Wheelchair()
    {
        pose_state.position = Eigen::Vector3d(0, 0, 0); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear_vel = Vector3d::Zero();
        pose_state.velocity.angular_vel = Vector3d::Zero();
    }

    Ublox ublox = {};
    std::atomic<bool> running = false;
    std::atomic<bool> pause = false;
    ~Wheelchair()
    {
        Command set_cmd(Command_Action::SET, Command_Target::INPUT, "0");
        ::write(tty_acm_fd, set_cmd.to_string().data(), set_cmd.to_string().size());
    };

    void send_velocity_command(Velocity2d &velocity) override;
    Pose_State read_state() override;
    void init();
};
