
#include "logger.hpp"

bool Logger::savePosesToFile(const Pose_State &state)
{
    if (!pose_file.is_open())
    {
        std::cerr << "Unable to open file for writing: " << "poses" << std::endl;
        return false;
    }
    pose_file << std::fixed;
    pose_file << 1 << " " << 0 << " " << 0 << " " << state.position.x() << " ";
    pose_file << 0 << " " << 1 << " " << 0 << " " << state.position.y() << " ";
    pose_file << 0 << " " << 0 << " " << 1 << " " << state.position.z();
    pose_file << std::endl;
    // traj_file << 0 << " " << 0 << " " << 1 << " " << interpolated;
    return true;
}

bool Logger::saveTimesToFile(const double &timestamp)
{
    if (!time_file.is_open())
    {
        std::cerr << "Unable to open file for writing: " << "times" << std::endl;
        return false;
    }

    time_file << std::fixed;
    time_file << timestamp << std::endl;
    return true;
    // time_file << trajectory_time << std::endl;
}

static bool saveToFile(const std::string &filename, const std::vector<Trajectory_Point> &data)
{
    std::ofstream outFile(filename);
    if (!outFile.is_open())
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return false;
    }

    for (const auto &line : data)
    {
        outFile << std::fixed;
        outFile << " ------------------------------------------------- " << std::endl;
        outFile << "dt: " << line.dt << " " << std::endl;
        outFile << "pose: " << line.pose.point.x() << " " << line.pose.point.y() << " "
                << line.pose.point.z() << " " << std::endl;
        outFile << "Velocity-linear: forward: " << line.velocity.linear.x()
                << " lateral: " << line.velocity.linear.y()
                << " vertical: " << line.velocity.linear.z() << std::endl;
        outFile << "Velocity-angular: pitch: " << line.velocity.angular.x()
                << " roll: " << line.velocity.angular.y() << " yaw: " << line.velocity.angular.z()
                << std::endl;
        // outFile << " ------------------------------------------------- "
        // << std::endl;
    }

    outFile.close();
    return true;
}
