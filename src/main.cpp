#include "sim.hpp"
#include "types.hpp"
#include "wheelchair.hpp"
#include "go1.hpp"
#include "sim_bot.hpp"
#include "load_config.hpp"
#include <getopt.h>
#include <stdlib.h>
#include <getopt.h>
#include "control_loop.cpp"

enum {
    ROBOT_CONFIG_FILEPATH = 1000,  // use values > 255 to avoid clashing with chars
    WAYPOINT_PATH,
    PATH_LOOPING
};

void
init(Robot& robot) {
    printf("robot.type = %d\n", robot.config.type);
    switch (robot.config.type) {
        case GO1:
            robot.ctx = new Go1(8090, "192.168.12.1", 8082);
            robot.read_pv = *go1_read_state;
            robot.send_velocity_command = *go1_send_velocity_command;
            robot.deinit = *go1_deinit;
            robot.init(go1_init);
            break;
        case G1:
            // ro.t->read_pv = *g1_read_state;
            // ro.t->send_velocity_command = *go1_send_velocity_command;
            break;
        case WHEELCHAIR:
            robot.ctx = new Wheelchair;
            robot.read_pv = *wheelchair_read_state;
            robot.send_velocity_command = *wheelchair_send_velocity_command;
            robot.deinit = *wheelchair_deinit;
            robot.init(wheelchair_init);
            break;
        case SIM:
            robot.ctx = new Sim_Bot;
            robot.read_pv = *sim_read_state;
            robot.send_velocity_command = *sim_send_velocity_command;
            robot.init(sim_init);
            break;
    }
}

int
main(int argc, char* argv[])
{

    Robot robot;
    Path_Planner p_planner;
    Path_Cursor p_cursor; 
    Path path;
            // read_json_latlon(robot.config.path_config.filepath); // NEW
    
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

    // if (!robot || !waypoints) {
    //     fprintf(stderr, "Error: both --robot and --waypoints are required.\n");
    //     return EXIT_FAILURE;
    // }

    Server server;
    server_init(server, robot);

    p_planner.global_cursor = &p_cursor;

    p_cursor.initialize(&path);
    p_planner.path_direction = robot.config.path_config.direction;
    // }

    frames_init(robot.frames, p_planner.global_cursor->path->waypoint(p_planner.global_cursor->current_waypoint),
                p_planner.global_cursor->get_next_waypoint());

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
    //
    // }

    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
        robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
        robot.config.kinematic_constraints.deceleration_max);
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

    init(robot);

    std::thread control_thread(control_loop, std::ref(robot), std::ref(p_planner), std::ref(traj_controller), std::ref(server));

    // control_loop<Wheelchair>(robot, traj_controller);
    Sim_Display sim = Sim_Display(robot, p_planner);
    sim.display();

    robot.running = false;
    control_thread.join();
    server_deinit(server);
    if (robot.deinit) robot.deinit(robot.ctx);

    return 0;
}
