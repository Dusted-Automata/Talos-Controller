#pragma once
#include "linear_controller.hpp"
#include "pid.hpp"
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
            return std::format("{}{},{}", action_char, target_char, value.value());
        } else {
            return std::format("{}{}", action_char, target_char);
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
    void init();
    Joystick scale_to_joystick(const Velocity2d &vel);
    std::string joystick_to_hex(Joystick stick_pos);

  public:
    Wheelchair()
    {
        pose_state.position = Eigen::Vector3d(0, 0, 0); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        config = {
            .control_loop_hz = 500,
            .kinematic_constraints =
            {
                .v_max = 2.5,
                .v_min = 0.0,
                .omega_max = 2.5,
                .omega_min = -2.5,
                .a_max = 100.0,
                .a_min = -100.0,
                .j_max = 0.0,
            },
        };

        PIDGains linear_gains = { 0.8, 0.05, 0.15 };
        LinearPID linear_pid(config, linear_gains);
        PIDGains angular_gains = { 1.0, 0.01, 0.25 };
        AngularPID angular_pid(config, angular_gains);
        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid);
        trajectory_controller->robot = this;
    }

    ~Wheelchair() { shutdown(); };

    void send_velocity_command(Velocity2d &velocity) override;
    Pose_State read_state() override;
};
