// class Trajectory_Controller {
//
//   Motion_Constraints motion_Constraints;
//   Velocity_Profile velocity_profile;
//   PIDController linear_pid;
//   PIDController angular_pid;
//
// public:
//   void follow_trajectory() {}
//
//   std::vector<Trajectory_Point>
//   generate_trajectory(const std::vector<Ecef_Coord> &coordinates,
//                       Robot_Config &config) {
//     std::vector<Trajectory_Point> path = {};
//     return path;
//   }
//
//   Pose get_current_pose() { return {}; }
//   void sendVelocityCommand(double linear_vel, double angular_vel) {
//     // In actual implementation, this would interface with:
//     // 1. Robot's motor controllers
//     // 2. ROS cmd_vel topic
//     // 3. Low-level motor driver
//     std::cout << "Linear Velocity: " << linear_vel
//               << " Angular Velocity: " << angular_vel << std::endl;
//   }
//
//   void local_replanning() {}
// };
