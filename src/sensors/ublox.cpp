
#include "ublox.hpp"
#include "cppmap3d.hh"
#include "sensor.hpp"
#include "math.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <thread>
#include <unistd.h>


static std::string
fixToString(gnss_fix fix)
{
    switch (fix) {
    case gnss_fix::NONE: return "NONE";
    case gnss_fix::GPS: return "GPS";
    case gnss_fix::DGPS: return "DGPS";
    case gnss_fix::PPS: return "PPS";
    case gnss_fix::RTK: return "RTK";
    case gnss_fix::RTK_FLOAT: return "RTK_FLOAT";
    case gnss_fix::ESTIMATED: return "ESTIMATED";
    case gnss_fix::MANUAL: return "MANUAL";
    case gnss_fix::SIMULATION: return "SIMULATION";
    default: return "Unknown";
    }
}

void
print_GGA(gnss_msg msg)
{
    std::cout << "time: " << (int)msg.time.hh << ":" << (int)msg.time.mm << ":" << (int)msg.time.ss << ":" << (int)msg.time.ms
              << std::endl;
    std::cout << "LLH: " << msg.llh.lat() << " , " << msg.llh.lon() << " , " << msg.llh.alt() << std::endl;
    std::cout << "fix: " << fixToString(msg.fix) << std::endl;
    std::cout << "num_satellites: " << static_cast<int>(msg.num_satalites) << std::endl;
    std::cout << "hdop: " << msg.hdop << std::endl;
    std::cout << "altitude: " << msg.alt << " meters" << std::endl;
    std::cout << "geoid_separation: " << msg.geoid_seperation << " meters" << std::endl;
    std::cout << "differential_age: " << msg.diff_age << " seconds" << std::endl;
    std::cout << "differential_station: " << msg.diff_station << std::endl;
}


gnss_msg
parse_GGA(json j){
    gnss_msg msg = {};

    msg.hdop = j["HDOP"];
    std::string lat_dir = j["NS"];
    std::string lon_dir = j["EW"];
    msg.alt = j["alt"];
    // diff_age = j["diffAge"];
    if (!(j["diffStation"] == "")) {
        msg.diff_station = j["diffStation"];
    }
    double lat = j["lat"];
    double lon = j["lon"];
    if (!(lat == 0.0 || lat_dir == "")) {
        if (lat_dir == "S") {
            lat *= -1.0;
        }
        lat = to_radian(lat);
        msg.llh.lat() = lat;
    }

    if (!(lon == 0.0) && !(lon_dir == "")) {
        if (lon_dir == "W") {
            lon *= -1.0;
        }

        lon = to_radian(lon);
        msg.llh.lon() = lon;
    }

    msg.num_satalites = j["numSV"];
    // msg.fix = static_cast<gnss_fix>(j["quality"]);
    msg.alt = j["alt"];
    msg.geoid_seperation = j["sep"];
    msg.llh.alt() = msg.alt + msg.geoid_seperation;

    return msg;
}

imu_msg
parse_Nav_PVAT(json j){
    imu_msg msg = {};

    msg.accHeading = j["accHeading"];
    msg.accPitch = j["accPitch"];
    msg.accRoll = j["accRoll"];
    double angle_heading = j["vehHeading"];
    double radian_heading = to_radian(angle_heading);
    double positive_radian = convert_to_positive_radians(M_PI / 2 - radian_heading);
    // veh_heading = convert_to_positive_radians(to_radian(angle_heading);
    msg.veh_heading = positive_radian;
    msg.heading = positive_radian;
    std::cout << "angle: " << angle_heading << " | radian: " << radian_heading
              << " | positive_radian: " << positive_radian << std::endl;
    msg.mot_heading = j["motHeading"];
    msg.mot_heading = convert_to_positive_radians(to_radian(msg.mot_heading));
    msg.pitch = j["vehPitch"];
    msg.roll = j["vehRoll"];

    msg.time.hh = j["hour"];
    msg.time.mm = j["min"];
    msg.time.ss = j["sec"];
    msg.time.ms = j["iTOW"];
    // llh.lat() = j["lat"];
    // llh.lat() = to_radian(llh.lat());
    // llh.lon() = j["lon"];
    // llh.lon() = to_radian(llh.lon());
    // llh.alt() = j["height"];      // in mm
    // llh.alt() = llh.alt() / 1000; // in M
    
    return msg;
}

void
parse_janik_msg(json j, std::optional<gnss_msg>& gnss, std::optional<imu_msg>& imu){
    imu.emplace();
    imu->accHeading = j["accHeading"];
    double angle_heading = j["vehHeading"];
    double radian_heading = to_radian(angle_heading);
    double positive_radian = convert_to_positive_radians(M_PI / 2 - radian_heading);
    imu->heading = positive_radian;

    gnss.emplace();
    gnss->llh.lat() = to_radian(j["lat"]);
    gnss->llh.lon() =to_radian(j["lon"]);
    gnss->llh.alt() = j["height"];
    // gnss->llh.alt() = gnss->llh.alt();
}

void
Ublox::loop()
{

    while (!(socket.get_fd() < 0) && running) {
        if (!socket.recv(buf)) {
            running = false;
            socket.disconnect();
        }
        for (size_t i = 0; i < buf.count(); i++) {
            if (buf[i] == '\n') {
                int len = i + 1; // i + 1 to include the newline
                std::string msg(len, '\0');
                buf.read(std::span(msg.data(), len));
                i = 0;
                json j;
                try {
                    j = json::parse(msg);
                } catch (nlohmann::json::parse_error &e) {
                    std::cerr << "Parse error at byte " << e.byte << ": " << e.what() << std::endl;
                    std::cout << msg.size() << " | " << msg.substr((e.byte - 10), 20) << std::endl;
                    std::cout << msg << std::endl;
                    break;
                } catch (nlohmann::json::exception &e) {
                    std::cerr << "Other JSON error: " << e.what() << std::endl;
                    break;
                }

                std::string id = j["identity"];
                if (id == "GPGGA") {
                    std::unique_lock<std::mutex> lock(mutex);
                    gnss = parse_GGA(j);
                }
                if (id == "NAV-PVAT") {
                    // std::cout << j.dump(4) << std::endl;
                    std::unique_lock<std::mutex> lock(mutex);
                    imu = parse_Nav_PVAT(j);
                }
                if (id == "refinePose") {
                    std::unique_lock<std::mutex> lock(mutex);
                    parse_janik_msg(j, gnss, imu);
                }
            }
        }
    }
}

bool
Ublox::start()
{

    std::cerr << "Starting Ublox" << std::endl;
    if (running) return true;
    if (!socket.connect()) {
        std::cerr << "Ublx couldn't connect to socket" << std::endl;
        return false;
    };

    if (socket.get_fd() < 0) {
        return false;
    }

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Ublox::loop, this);
    running = true;
    return true;
}





void
update_position(Ublox &ublox, Frames &frames)
{
    std::unique_lock<std::mutex> lock(ublox.mutex);
    if (ublox.gnss.has_value()) {
        if (   above_epsilon(0.001, ublox.gnss->llh.lat()) 
            || above_epsilon(0.001, ublox.gnss->llh.lon())
            || above_epsilon(0.001, ublox.gnss->llh.alt())
        ) {
            frames_update_based_on_measurement(frames, ublox.gnss->llh);
        }
        ublox.gnss.reset();
    }
}

void
update_heading(Ublox &ublox, Frames &frames)
{
    std::unique_lock<std::mutex> lock(ublox.mutex);
    if (ublox.imu.has_value()) {
        if (ublox.imu->accHeading < 30.0) { 
            Eigen::Affine3d rotationMatrix;
            rotationMatrix = Eigen::AngleAxisd(ublox.imu->heading, Eigen::Vector3d::UnitZ());
            frames.local_frame.orientation = rotationMatrix; 
        }
        ublox.imu.reset();
    }
}

bool
Ublox::update_speed(Velocity2d vel)
{
    json j;
    j["identity"] = "ESF-MEAS-SPEED";
    j["speed"] = vel.linear_vel.norm();
    std::string msg = j.dump();
    msg.append("\n");
    // std::cout << socket.get_fd() << std::endl;
    if (!(socket.get_fd() < 0)) {
        // std::cout << msg << std::endl;
        if (socket.send(msg.c_str(), msg.size())) {
            return true;
        }
    }
    return false;
}
