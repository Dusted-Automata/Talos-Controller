#pragma once

#include "types.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

class Logger
{
  public:
    Logger()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t now_t = std::chrono::system_clock::to_time_t(now);
        std::tm *localTime = std::localtime(&now_t);
        char buffer[64];
        std::strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H%M%S", localTime);
        std::string timeString(buffer);

        pose_file.open("poses", std::ios::out | std::ios::trunc);
        time_file.open("times", std::ios::out | std::ios::trunc);
        /*logging_file.open("LOG_" + timeString, std::ios::out | std::ios::trunc);*/
    }
    std::ofstream pose_file;
    std::ofstream time_file;
    std::ofstream logging_file;

    bool savePosesToFile(const Pose_State &state);
    bool saveTimesToFile(const double &timestamp);
};

// std::cout << "DT: " << motiontime << " VEL: " << state.velocity[0] << " YAW: " <<
// state.yawSpeed
//           << " | " << std::endl;
// << "S.VEL: " << state.velocity[0] << " S.YAW: " << state.yawSpeed << std::endl;
