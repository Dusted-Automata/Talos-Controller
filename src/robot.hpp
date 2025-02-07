#pragma once
#include <Eigen/Dense>

struct ControlState {
  Eigen::Vector4d position;
  Eigen::Vector4d velocity;
  float time;
};

struct ControlCommand {
  Eigen::Vector4d effort;
  float timestamp;

  ControlCommand(const Eigen::VectorXd &eff, float time)
      : effort(eff), timestamp(time) {}
};

class Controller {
public:
  virtual ~Controller() = default;

  virtual ControlCommand computeControl(const ControlState &currentState,
                                        const ControlState &desiredState) = 0;

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
