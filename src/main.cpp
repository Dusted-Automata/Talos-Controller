#include "load_config.hpp"
#include "sim.hpp"
#include "wheelchair.hpp"

int
main(int argc, char* argv[])
{
    // Load robot given by the arg and the profile.
    // Maybe also load the path/map through the args?
    // {
    Wheelchair robot;
    load_config(robot, "robot_configs/wheelchair_profile_3_bar_3.json");
    // } 
    // {
    Server server;
    server_init(server, robot);
    // }
    // {
    Path_Planner p_planner;
    Path_Cursor p_cursor; 
    Path path = read_json_latlon(robot.config.path_config.filepath); // NEW
    path.set_looping(true);
    p_planner.global_cursor = &p_cursor;

    p_cursor.initialize(&path);
    p_planner.path_direction = robot.config.path_config.direction;
    p_planner.global_path.read_json_latlon(robot.config.path_config.filepath);
    p_planner.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);
    // }

    frames_init(robot.frames, p_planner.local_path);

    {

        std::cout << "ROBOT INIT!" << std::endl;
        bool ublox_start = robot.ublox.start();
        std::cout << "UBLOX: " << ublox_start << std::endl;
        while (!robot.ublox.imu.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (robot.ublox.imu.has_value()) break;
        }
        update_position(robot.ublox, robot.frames);
        update_heading(robot.ublox, robot.frames);
        p_planner.re_identify_position(robot.frames.local_frame.pos);
        if (!ublox_start) {
            return -1;
        }
    }

    Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
        robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
        robot.config.kinematic_constraints.deceleration_max);
    Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

    robot.init();

    std::thread control_thread(control_loop<Wheelchair>, std::ref(robot), std::ref(p_planner), std::ref(traj_controller), std::ref(server));

    // control_loop<Wheelchair>(robot, traj_controller);
    Sim_Display sim = Sim_Display(robot, p_planner);
    sim.display();
    //
    // CloseWindow();

    return 0;
}
