#pragma once
#include "robot.hpp"

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

std::string create_command_string(Command_Action a, Command_Target t, std::optional<std::string> value);

struct Joystick {
    uint8_t x;
    uint8_t y;
};

class Wheelchair : public Robot
{
  private:
    int tty_acm_fd = -1;
    Joystick scale_to_joystick(const Velocity2d &vel);
    bool joystick_to_hex(std::array<char, 10> &buffer, Joystick stick_pos);
    // std::array<char, 128> tty_read_buf;

  public:
    Wheelchair()
    {
        pose_state.position = Eigen::Vector3d(0, 0, 0); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear_vel = Vector3d::Zero();
        pose_state.velocity.angular_vel = Vector3d::Zero();
    }

    ~Wheelchair()
    {
        std::string cmd = create_command_string(Command_Action::SET, Command_Target::JOYSTICK, "0");
        ::write(tty_acm_fd, cmd.data(), cmd.size());
    };

    void send_velocity_command(Velocity2d &velocity) override;
    Pose_State read_state() override;
    void init();
};
