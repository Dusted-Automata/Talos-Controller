#pragma once
#include "socket.hpp"
#include <cstdint>
#include <iostream>
#include <queue>

struct UTC_Time
{
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint16_t ms;
};

struct LatLng
{
    double lat;
    double lng;
};

enum class NMEA_Cmd
{
    UNKNOWN,
    GGA,
};

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D

struct GGA
{
    enum class Fix
    {
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
    // All of the char arrays, have space for null termination.
    UTC_Time time; // UTC time - hhmmss.ss
    LatLng latlng;
    Fix fix;                // Quality indicator for position fix
    uint8_t num_satalites;  // Number of satellites used (0-12)
    float hddp;             // Horizontal Dilution of Precision
    float alt;              // Altitude above mean sea level - meters
    float geoid_seperation; // difference between ellipsoid and mean sea level
    float diff_age;         // Age of differential corrections (null when DGPS is not used)
    float diff_station; // ID of station providing differential corrections (0 when DGPS not used)

    void print()
    {
        std::cout << "time: " << (int)time.hh << ":" << (int)time.mm << ":" << (int)time.ss << ":"
                  << (int)time.ms << std::endl;
        std::cout << "latlng: " << latlng.lat << " , " << latlng.lng << std::endl;
        std::cout << "fix: " << static_cast<int>(fix) << std::endl;
        std::cout << "num_satellites: " << static_cast<int>(num_satalites) << std::endl;
        std::cout << "hdop: " << hddp << std::endl;
        std::cout << "altitude: " << alt << " meters" << std::endl;
        std::cout << "geoid_separation: " << geoid_seperation << " meters" << std::endl;
        std::cout << "differential_age: " << diff_age << " seconds" << std::endl;
        std::cout << "differential_station: " << diff_station << std::endl;
    }
};


class Ublox
{
    NMEA_Parser parser = {};
    TCP_Socket tcp = TCP_Socket("127.0.0.1", 50010, parser);

  public:
    Ublox()
    {
        if (!tcp.connect())
        {
            std::cerr << "Ublx couldn't connect" << std::endl;
        };
    };
	std::queue<std::string> buf;
	std::queue<GGA> msgs;

    void poll();
};
