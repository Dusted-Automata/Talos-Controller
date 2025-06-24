#include "robot.hpp"
#include "trajectory_controller.hpp"

// using namespace std::chrono;

bool
Robot::init()
{

    ublox.start();
    std::cout << "ROBOT INIT!" << std::endl;
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
