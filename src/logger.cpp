
#include "logger.hpp"
#include "frame_controller.hpp"

bool Logger::savePosesToFile(const Frame_Controller &controller)
{
    if (!pose_file.is_open())
    {
        std::cerr << "Unable to open file for writing: "
                  << "poses" << std::endl;
        return false;
    }
    pose_file << std::fixed;
    pose_file << controller.local_frame.orientation(0, 0) << " "
              << controller.local_frame.orientation(0, 1) << " "
              << controller.local_frame.orientation(0, 2) << " " << controller.global_frame.pos.x()
              << " ";
    pose_file << controller.local_frame.orientation(1, 0) << " "
              << controller.local_frame.orientation(1, 1) << " "
              << controller.local_frame.orientation(1, 2) << " " << controller.global_frame.pos.y()
              << " ";
    pose_file << controller.local_frame.orientation(2, 0) << " "
              << controller.local_frame.orientation(2, 1) << " "
              << controller.local_frame.orientation(2, 2) << " " << controller.global_frame.pos.z()
              << " ";

    // tracks movement in space from start position.
    // pose_file << state.orientation(0, 3) << " " << state.orientation(1, 3) << " "
    //           << state.orientation(2, 3) << " " << state.orientation(3, 3) << " ";
    pose_file << std::endl;
    // traj_file << 0 << " " << 0 << " " << 1 << " " << interpolated;
    return true;
}

bool Logger::saveTimesToFile(const double &timestamp)
{
    if (!time_file.is_open())
    {
        std::cerr << "Unable to open file for writing: "
                  << "times" << std::endl;
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
