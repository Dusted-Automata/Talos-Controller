#include "robot.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"

// using namespace std::chrono;
void
Robot::control_loop()
{

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    motion_time_start = clock::now();
    std::chrono::milliseconds period(1000 / config.control_loop_hz);

    while (running) {
        while (!pause && running) {
            pose_state = read_state();
            pose_state.velocity.linear *= 1.0 / config.control_loop_hz;
            pose_state.velocity.angular *= 1.0 / config.control_loop_hz;
            frames.move_in_local_frame(pose_state.velocity);
            logger.savePosesToFile(frames);
            logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            Velocity2d cmd = trajectory_controller->get_cmd();
            send_velocity_command(cmd);

            next += period;
            std::this_thread::sleep_until(next);

            if (clock::now() > next + period) {
                std::cerr << "control-loop overrun" << std::endl;
                next = clock::now();
            }
        }
    }
}

void
Robot::start()
{

    if (running) return;
    running = true;

    if (control_loop_thread.joinable()) {
        control_loop_thread.join();
    }

    control_loop_thread = std::thread(&Robot::control_loop, this);
}

void
Robot::shutdown()
{
    running = false;
    if (control_loop_thread.joinable()) {
        control_loop_thread.join();
    }
}

bool
Robot::init()
{

    std::cout << "ROBOT INIT!" << std::endl;
    bool ublox_start = ublox.start();
    std::cout << "UBLOX: " << ublox_start << std::endl;
    double heading_tolerance = 0.002;
    while (true) {
        std::optional<Nav_Att> msg = ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
        if (msg.has_value()) {
            if (msg.value().heading < heading_tolerance && msg.value().heading > -heading_tolerance) {
                std::cout << "HEADING IS UNDER TOLERANCE!" << std::endl;
                std::cout << "---------------------------" << std::endl;
                break;
            }
        }
    }
    return true;
}
