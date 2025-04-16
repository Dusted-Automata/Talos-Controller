#pragma once
#include "socket.hpp"
#include <cstdint>
#include <iostream>

struct utc_time
{
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint16_t ms;
};

struct latlng
{
    double lat;
    double lng;
};

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
    utc_time time; // UTC time - hhmmss.ss
    latlng latlng;
    Fix fix;                // Quality indicator for position fix
    uint8_t num_satalites;  // Number of satellites used (0-12)
    float hddp;             // Horizontal Dilution of Precision
    float alt;              // Altitude above mean sea level - meters
    float geoid_seperation; // difference between ellipsoid and mean sea level
    float diff_age;         // Age of differential corrections (null when DGPS is not used)
    float diff_station; // ID of station providing differential corrections (0 when DGPS not used)
};

class Ublox
{
    TCP_Socket tcp = TCP_Socket("127.0.0.1", 50010);

  public:
    Ublox()
    {
        if (!tcp.connect())
        {
            std::cerr << "Ublx couldn't connect" << std::endl;
        };
    };

    bool poll();
    GGA read();
};
