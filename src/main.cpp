// #include "linear_controller.hpp"
// #include "load_config.hpp"
// #include "motion_profile.hpp"
// #include "server.hpp"
// #include "sim.hpp"
// #include "wheelchair.hpp"
#include <getopt.h>
#include "control_loop.cpp"
#include "load_config.hpp"
#include <stdlib.h>
#include <getopt.h>

enum {
    ROBOT_CONFIG_FILEPATH = 1000,  // use values > 255 to avoid clashing with chars
    WAYPOINT_PATH,
    PATH_LOOPING
};

void
load_robot_config(Robot* robot, char* robot_path) {
    load_config(*robot, robot_path);
}

int
main(int argc, char* argv[])
{

    Robot robot;
    Path_Planner p_planner;
    Path_Cursor p_cursor; 
    Path path;
            // read_json_latlon(robot.config.path_config.filepath); // NEW
    
    const char *input1 = NULL;
    const char *input2 = NULL;
    const char *input3 = NULL;

    static struct option long_opts[] = {
        {"robot", required_argument, 0, ROBOT_CONFIG_FILEPATH},
        {"waypoints", required_argument, 0, WAYPOINT_PATH},
        {"looping", optional_argument, 0, PATH_LOOPING},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "", long_opts, NULL)) != -1) {
        switch (opt) {
            case ROBOT_CONFIG_FILEPATH: {
                load_config(robot, optarg);
                break;
            }
            case WAYPOINT_PATH:  {
                path = read_json_latlon(optarg);
                break;
            }
            case PATH_LOOPING: {
                path.set_looping(true);
                break;
            }
            default:
                return EXIT_FAILURE;
        }
    }

    if (!input1 || !input2) {
        fprintf(stderr, "Error: both --robot and --waypoints are required.\n");
        return EXIT_FAILURE;
    }

    printf("Input1: %s\n", input1);
    printf("Input2: %s\n", input2);




    // Maybe also load the path/map through the args?
    // {
    // Wheelchair robot;
    // load_config(robot, "robot_configs/wheelchair_profile_3_bar_3.json");
    // // } 
    // // {
    // Server server;
    // server_init(server, robot);
    // // }
    // // {
    // Path_Planner p_planner;
    // Path_Cursor p_cursor; 
    // Path path = read_json_latlon(robot.config.path_config.filepath); // NEW
    // path.set_looping(true);
    // p_planner.global_cursor = &p_cursor;
    //
    // p_cursor.initialize(&path);
    // p_planner.path_direction = robot.config.path_config.direction;
    // p_planner.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);
    // // }
    //
    // frames_init(robot.frames, p_planner.global_cursor->path->waypoint(p_planner.global_cursor->current_waypoint),
    //             p_planner.global_cursor->get_next_waypoint());
    // {
    //
    //     std::cout << "ROBOT INIT!" << std::endl;
    //     bool ublox_start = robot.ublox.start();
    //     std::cout << "UBLOX: " << ublox_start << std::endl;
    //     while (!robot.ublox.imu.has_value()) {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         if (robot.ublox.imu.has_value()) break;
    //     }
    //     update_position(robot.ublox, robot.frames);
    //     update_heading(robot.ublox, robot.frames);
    //     p_planner.re_identify_position(robot.frames.local_frame.pos);
    //     if (!ublox_start) {
    //         return -1;
    //     }
    // }
    //
    // Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
    //     robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
    //     robot.config.kinematic_constraints.deceleration_max);
    // Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);
    //
    // robot.init();
    //
    // std::thread control_thread(control_loop, std::ref(robot), std::ref(p_planner), std::ref(traj_controller), std::ref(server));
    //
    // // control_loop<Wheelchair>(robot, traj_controller);
    // Sim_Display sim = Sim_Display(robot, p_planner);
    // sim.display();
    // //
    // // CloseWindow();

    return 0;
}
