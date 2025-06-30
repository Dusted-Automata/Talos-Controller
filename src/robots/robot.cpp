#include "robot.hpp"
#include "trajectory_controller.hpp"

// using namespace std::chrono;

bool
Robot::init()
{

    std::cout << "ROBOT INIT!" << std::endl;
    bool ublox_start = ublox.start();
    std::cout << "UBLOX: " << ublox_start << std::endl;
    double heading_tolerance = 0.5;
    // while (true && ublox_start) {
    //     std::optional<Nav_Att> msg = ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
    //     if (msg.has_value()) {
    //         std::cout << "HEADING IS " << msg->heading << std::endl;
    //         if (msg.value().heading < heading_tolerance || msg.value().heading > 360 - heading_tolerance) {
    //             std::cout << "HEADING IS UNDER TOLERANCE!" << std::endl;
    //             std::cout << "---------------------------" << std::endl;
    //             break;
    //         } else {
    //             std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //         }
    //     }
    // }
    running = true;
    return true;
}
