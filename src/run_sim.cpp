#include "raylib.h"
#include "sim.hpp"

int
main()
{
    std::vector<Ecef_Coord> waypoints = {
        { 4100175.6251356260, 476368.7899695045, 4846344.356704135 },
        { 4100209.6729529747, 476361.2681338759, 4846316.478097512 },
        { 4100218.5394949187, 476445.5598077707, 4846300.796185957 },
        { 4100241.7219579100, 476441.0557096391, 4846281.753675706 }
    };

    Sim_Quadruped robot;

    robot.path_controller.path_looping = true;
    robot.path_controller.add_waypoints(waypoints);
    robot.path_controller.start();
    robot.sensor_manager.init();
    robot.frames.init(robot.path_controller.path_points_all.front());

    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 19, -Vector3d::UnitY()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 2, -Vector3d::UnitZ()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI, Vector3d::UnitY()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 50, Vector3d::UnitY()));
    /*std::cout << robot.frames.global_frame.orientation.rotation() << std::endl;*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/

    Sim_Display sim = Sim_Display(robot, waypoints);

    sim.display();

    CloseWindow();
    return 0;
}
