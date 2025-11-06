#pragma once
#include "robot.hpp"
#include <unistd.h>

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

struct Wheelchair
{

    int fd = -1;
    Kinematic_Constraints kc;
    LA current_velocity = {};

};
Joystick scale_to_joystick(const Robot& robot, const Velocity2d &vel);
bool joystick_to_hex(std::array<char, 10> &buffer, Joystick stick_pos);

void wheelchair_send_velocity_command(void* ctx, Velocity2d &velocity);
void wheelchair_init(void* ctx, const Robot* robot);
void wheelchair_deinit(void* ctx);
LA wheelchair_read_state(void* ctx);
