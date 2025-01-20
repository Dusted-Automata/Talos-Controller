#include <Eigen/Dense>

struct Pose2d {
  double x, y, theta;
};

struct Velocity2D {
  // velocity[0], yawspeed I think in HighCmd
  double linear, angular;
};

struct ControlState {
  Eigen::Vector4d position;
  Eigen::Vector4d velocity;
  float time;
};

// struct ControlCommand {
//   Eigen::Vector4d effort;
//   float timestamp;
//
//   ControlCommand(const Eigen::VectorXd &eff, float time)
//       : effort(eff), timestamp(time) {}
// };

class Controller {
public:
  virtual ~Controller() = default;

  // virtual double update(const ControlState &currentState,
  //     const ControlState &desiredState) = 0;
  virtual double update(double measured_value, double dt) = 0;

  virtual void reset() = 0;
};

class Robot {
public:
  Robot() {}

  void updateState() {}

  int motiontime = 0;
  float dt = 0.002;

  Controller *controller;
};

// class Robot {
//   public:
//        UT::HighCmd   robot_high_cmd;
//        UT::HighState base_high_state;
//        UT::LowCmd    robot_low_cmd;
//        UT::LowState  base_low_state;
//        uint8_t mode = 2;
//        uint8_t gait_type = 2;
//        uint8_t speed_level = 0;
//        float foot_raise_height;
//        float body_height;
//        std::array<float, 2> position;
//        std::array<float, 3> euler;
//        go1_legged_msgs::msg::HighState extractHighStateMessage();
//        go1_legged_msgs::msg::LowState  extractLowStateMessage();
//        go1_legged_msgs::msg::MotorStateArray extractMotorStateMessage();
//        sensor_msgs::msg::Imu extractImuMessage();
//        sensor_msgs::msg::BatteryState extractBatteryStateMessage();
//        std::tuple<nav_msgs::msg::Odometry,
//        geometry_msgs::msg::TransformStamped> extractOdometryMessage();
// };

// class Quadruped : public qre::Robot {
// 	Eigen::Matrix4d go1_config_matrix[4];
// 	const double trunk_length = 0.3762/2;
// 	const double trunk_width  = 0.0935/2;
// 	const double l1 = 0.;
// 	const double l2 = 0.080; // hip
// 	const double l3 = 0.213; // thigh
// 	const double l4 = 0.213; // calf
//
// public:
// 	Quadruped();
// 	~Quadruped();
// 	Eigen::Matrix<double, 4, 1>* footTransformsFromPositions();
//   	Eigen::Matrix<double, 4, 3>
//   jointAnglesFromFootPositions(Eigen::Matrix<double, 4, 1> *foot_positions);
// 	sensor_msgs::msg::JointState getJointStates();
// 	sensor_msgs::msg::JointState extractJointAngles();
// 	Eigen::Matrix3d rotx(double alpha);
// 	Eigen::Matrix3d roty(double beta);
// 	Eigen::Matrix3d rotz(double gamma);
// 	Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);
// };
//

// class Base : public rclcpp::Node, public Quadruped {
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
//     cmd_vel_subscriber;
//     rclcpp::Subscription<go1_legged_msgs::msg::JointCmd>::SharedPtr
//     joint_cmd_subscriber;
//     rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
//     joint_state_publisher;
//     rclcpp::Publisher<go1_legged_msgs::msg::HighState>::SharedPtr
//     high_state_publisher;
//     rclcpp::Publisher<go1_legged_msgs::msg::LowState>::SharedPtr
//     low_state_publisher; rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
//     imu_publisher;
//     rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
//     battery_state_publisher;
//     rclcpp::Publisher<go1_legged_msgs::msg::MotorStateArray>::SharedPtr
//     motor_state_publisher;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
//     rclcpp::Service<go1_legged_msgs::srv::SetMode>::SharedPtr set_mode;
//     rclcpp::Service<go1_legged_msgs::srv::SetControl>::SharedPtr set_control;
//     rclcpp::TimerBase::SharedPtr timer_;
//
//     UT::Safety robot_safety;
//     UT::UDP* comm_bus;
//     uint16_t level;
//     std::string modes[4] = {"Position", "Velocity", "Torque", "Full"};
//     std::string low_level_control_type = "Position";
//     char udp_ip[16];
//     void timerCallback();
//
// public:
//     float dt = 0.002;     // 0.001~0.01
//     Base();
//     Base(UT::HighCmd cmd, UT::HighState state);
//     Base(UT::LowCmd  cmd, UT::LowState  state);
//     void HighLevelControl();
//     void LowLevelControl();
//     void packetReceive();
//     void packetSend();
//     void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
//     void jointCommandCallback(const go1_legged_msgs::msg::JointCmd::SharedPtr
//     msg); bool
//     setModeCallback(std::shared_ptr<go1_legged_msgs::srv::SetMode::Request>
//     req, std::shared_ptr<go1_legged_msgs::srv::SetMode::Response> res); bool
//     setControlCallback(std::shared_ptr<go1_legged_msgs::srv::SetControl::Request>
//     req, std::shared_ptr<go1_legged_msgs::srv::SetControl::Response> res);
// };
