#pragma once
#include "json.hpp"
#include <cstdint>
#include <iostream>

struct UTC_Time {
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint16_t ms;
};

struct LatLng {
    double lat;
    double lng;
};

enum class NMEA_Cmd {
    UNKNOWN,
    GGA,
};

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D

using nlohmann::json;
struct GGA {
    enum class Fix {
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

    static std::string
    fixToString(Fix fix)
    {
        switch (fix) {
        case Fix::NONE: return "NONE";
        case Fix::GPS: return "GPS";
        case Fix::DGPS: return "DGPS";
        case Fix::PPS: return "PPS";
        case Fix::RTK: return "RTK";
        case Fix::RTK_FLOAT: return "RTK_FLOAT";
        case Fix::ESTIMATED: return "ESTIMATED";
        case Fix::MANUAL: return "MANUAL";
        case Fix::SIMULATION: return "SIMULATION";
        default: return "Unknown";
        }
    }

    // All of the char arrays, have space for null termination.
    UTC_Time time; // UTC time - hhmmss.ss
    LatLng latlng;
    Fix fix;                // Quality indicator for position fix
    uint8_t num_satalites;  // Number of satellites used (0-12)
    float hdop;             // Horizontal Dilution of Precision
    double alt;             // Altitude above mean sea level - meters
    float geoid_seperation; // difference between ellipsoid and mean sea level
    float diff_age;         // Age of differential corrections (null when DGPS is not used)
    int diff_station;       // ID of station providing differential corrections (0 when DGPS not used)

    GGA(json j)
    {
        hdop = j["HDOP"];
        std::string lat_dir = j["NS"];
        std::string lon_dir = j["EW"];
        alt = j["alt"];
        // diff_age = j["diffAge"];
        diff_station = j["diffStation"];
        double lat = j["lat"];
        double lon = j["lon"];
        if (!(lat == 0.0 || lat_dir == "")) {
            if (lat_dir == "S") {
                lat *= -1.0;
            }
            latlng.lat = lat;
        }

        if (!(lon == 0.0) && !(lon_dir == "")) {
            if (lon_dir == "W") {
                lon *= -1.0;
            }

            latlng.lng = lon;
        }

        num_satalites = j["numSV"];
        // j["msgID"];
        // j["msgmode"];
        // fix = static_cast<GGA::Fix>(j["quality"]);
    }

    void
    print()
    {
        std::cout << "time: " << (int)time.hh << ":" << (int)time.mm << ":" << (int)time.ss << ":" << (int)time.ms
                  << std::endl;
        std::cout << "latlng: " << latlng.lat << " , " << latlng.lng << std::endl;
        std::cout << "fix: " << fixToString(fix) << std::endl;
        std::cout << "num_satellites: " << static_cast<int>(num_satalites) << std::endl;
        std::cout << "hdop: " << hdop << std::endl;
        std::cout << "altitude: " << alt << " meters" << std::endl;
        std::cout << "geoid_separation: " << geoid_seperation << " meters" << std::endl;
        std::cout << "differential_age: " << diff_age << " seconds" << std::endl;
        std::cout << "differential_station: " << diff_station << std::endl;
    }
};
