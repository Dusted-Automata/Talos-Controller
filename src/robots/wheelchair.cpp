#include "wheelchair.hpp"
#include "sim.hpp"

std::string
Wheelchair::scale_joystick(Velocity2d vel)
{
    double scaled_x = vel.linear.x();
    double scaled_yaw = vel.angular.y();

    if (vel.linear.x() >= 0) {
        scaled_x = vel.linear.x() / (config.kinematic_constraints.v_max / 100);
    } else {
        scaled_x = vel.linear.x() / (config.kinematic_constraints.v_min / 100);
    }

    if (vel.angular.y() >= 0) {
        scaled_yaw = vel.angular.y() / (config.kinematic_constraints.omega_max / 100);
    } else {
        scaled_yaw = vel.angular.y() / (config.kinematic_constraints.omega_min / 100);
    }

    // std::string hex = std::format("{:02X},{:02X}", scaled_x, scaled_yaw);
    std::string hex = std::format("{:02X},{:02X}", static_cast<int>(scaled_x), static_cast<int>(scaled_yaw));

    return hex;
}

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Wheelchair robot;
    robot.path.path_looping = true;
    robot.sensor_manager.init();
    robot.path.read_json_latlon("ecef_points.json");
    robot.frames.init(robot.path.path_points_all);

    Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);
    sim.display();

    CloseWindow();

    return 0;
}
