#include "linear_controller.hpp"
#include "pid.hpp"
#include "robot.hpp"
#include "trajectory_controller.hpp"
#include <thread>

class testRobot : public Robot
{

  public:
    testRobot() {

    Robot_Config config = {
        .hz = 50,
        .motion_constraints =
            {
                .max_velocity = 1.0,
                .max_acceleration = 0.5,
                .max_deceleration = 0.5,
                .max_jerk = 0.0,
            },

    };

    PIDGains linear_gains = {1.0, 0.0, 0.0};
    PIDController linear_pid(linear_gains);
    linear_pid.output_max = 10, 0;
    linear_pid.output_min = 0, 0;
    PIDGains angular_gains = {1.0, 0.0, 0.0};
    PIDController angular_pid(angular_gains);
    angular_pid.output_max = 10, 0;
    angular_pid.output_min = 0, 0;
	trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid, config.hz);



	}
    void send_velocity_command(Velocity2d &cmd) override{};
    Pose_State read_state() override { return {}; };
};

int main()
{
    // std::vector<Ecef_Coord> waypoints = {
    //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
    //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
    //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
    //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

    std::vector<Ecef_Coord> waypoints = {
        {0.0, 0.0, 0.0}, {4.0, -2.0, 0.0}, {4.0, 2.0, 0.0}, {0.0, 0.0, 0.0}};


    Ecef_Coord current = waypoints[0];
    Ecef_Coord next = waypoints[1];

    testRobot robot;


    std::function<void()> bound_path_loop = [&robot, &waypoints]() {
		static_cast<Linear_Controller*>(robot.trajectory_controller.get())->path_loop(waypoints);
	
	};
    std::function<void()> bound_control_loop = std::bind(&testRobot::control_loop, &robot);
    std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);
    std::thread control_loop_thread = std::thread(worker_function, bound_control_loop, 2);
    // std::thread control_loop_thread = std::thread(&testRobot::control_loop, &robot);

    path_loop.join();
    /*trajectory_loop.join();*/
    control_loop_thread.join();

    return 0;
}
