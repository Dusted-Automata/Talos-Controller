#pragma once
#include "frames.hpp"
#include "sensor.hpp"
#include "socket.hpp"
#include <iostream>
#include <json.hpp>
#include <thread>

using nlohmann::json;

struct UTC_Time {
    uint8_t hh = 0;
    uint8_t mm = 0;
    uint8_t ss = 0;
    uint16_t ms = 0;
};

enum class NMEA_Cmd {
    UNKNOWN,
    GGA,
};

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D

enum class gnss_fix {
    NONE = 0,      ///< No fix.
    GPS = 1,       ///< GPS fix.
    DGPS = 2,      ///< Differential GPS fix.
    PPS = 3,       ///< Pulse Per Second (PPS) fix.
    RTK = 4,       ///< Real-Time Kinematic (RTK) fix.
    RTK_FLOAT = 5, ///< Real-Time Kinematic (RTK) floating point fix.
    ESTIMATED = 6, ///< Estimated dead reckoning fix.
    MANUAL = 7,    ///< Manual input mode.
    SIMULATION = 8 ///< Simulation mode.
};


using nlohmann::json;


struct gnss_msg {
    LLH llh;
    UTC_Time time; // UTC time - hhmmss.ss
    gnss_fix fix;                // Quality indicator for position fix
    uint8_t num_satalites;  // Number of satellites used (0-12)
    float hdop;             // Horizontal Dilution of Precision
    double alt;             // Altitude above mean sea level - meters
    float geoid_seperation; // difference between ellipsoid and mean sea level
    float diff_age;         // Age of differential corrections (null when DGPS is not used)
    int diff_station;       // ID of station providing differential corrections (0 when DGPS not used)
};

struct imu_msg {
    // LLH llh;
    UTC_Time time; // UTC time - hhmmss.ss
    double accHeading = 0.0;
    double accPitch   = 0.0;
    double accRoll    = 0.0;

    double veh_heading = 0.0;
    double heading     = 0.0;
    double mot_heading = 0.0;
    double pitch       = 0.0;
    double roll        = 0.0;
    double height      = 0.0;
};


class Ublox
{
private:
    std::atomic_bool running = false;
    std::thread sensor_thread;
  public:
    std::optional<gnss_msg> gnss;
    std::optional<imu_msg> imu;
    Ublox() {};
    TCP_Client socket = TCP_Client("127.0.0.1", 50020);
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> buf;

    void loop();
    bool start();

    bool update_speed(Velocity2d vel);
    std::mutex mutex;
};

void update_position(Ublox &ublox, Frames &frames);
void update_heading(Ublox &ublox, Frames &frames);
