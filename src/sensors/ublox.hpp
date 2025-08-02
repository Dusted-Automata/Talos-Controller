#pragma once
#include "frames.hpp"
#include "sensor.hpp"
#include "socket.hpp"
#include <iostream>
#include <json.hpp>

using nlohmann::json;

struct UTC_Time {
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint16_t ms;
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
    LLH llh;
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
        if (!(j["diffStation"] == "")) {
            diff_station = j["diffStation"];
        }
        double lat = j["lat"];
        double lon = j["lon"];
        if (!(lat == 0.0 || lat_dir == "")) {
            if (lat_dir == "S") {
                lat *= -1.0;
            }
            lat = to_radian(lat);
            llh.lat() = lat;
        }

        if (!(lon == 0.0) && !(lon_dir == "")) {
            if (lon_dir == "W") {
                lon *= -1.0;
            }

            lon = to_radian(lon);
            llh.lon() = lon;
        }

        num_satalites = j["numSV"];
        // j["msgID"];
        // j["msgmode"];
        // fix = static_cast<GGA::Fix>(j["quality"]);
        alt = j["alt"];
        geoid_seperation = j["sep"];
        llh.alt() = alt + geoid_seperation;
    }

    void
    print()
    {
        std::cout << "time: " << (int)time.hh << ":" << (int)time.mm << ":" << (int)time.ss << ":" << (int)time.ms
                  << std::endl;
        std::cout << "LLH: " << llh.lat() << " , " << llh.lon() << " , " << llh.alt() << std::endl;
        std::cout << "fix: " << fixToString(fix) << std::endl;
        std::cout << "num_satellites: " << static_cast<int>(num_satalites) << std::endl;
        std::cout << "hdop: " << hdop << std::endl;
        std::cout << "altitude: " << alt << " meters" << std::endl;
        std::cout << "geoid_separation: " << geoid_seperation << " meters" << std::endl;
        std::cout << "differential_age: " << diff_age << " seconds" << std::endl;
        std::cout << "differential_station: " << diff_station << std::endl;
    }
};

struct Nav_Att {
    Nav_Att(json j)
    {
        accHeading = j["accHeading"];
        accPitch = j["accPitch"];
        accRoll = j["accRoll"];
        heading = j["heading"];
        pitch = j["pitch"];
        roll = j["roll"];
        iTOW = j["iTOW"];
        length = j["length"];
        msgmode = j["msgmode"];
    }

    double accHeading;
    double accPitch;
    double accRoll;
    double heading;
    double pitch;
    double roll;
    int32_t iTOW;
    int32_t length;
    int8_t msgmode;
};

struct Nav_Pvat {
    Nav_Pvat(json j)
    {
        accHeading = j["accHeading"];
        accPitch = j["accPitch"];
        accRoll = j["accRoll"];
        veh_heading = j["vehHeading"];
        mot_heading = j["motHeading"];
        pitch = j["vehPitch"];
        roll = j["vehRoll"];

        time.hh = j["hour"];
        time.mm = j["min"];
        time.ss = j["sec"];
        time.ms = j["iTOW"];
        //
        num_satalites = j["numSV"];
        llh.lat() = j["lat"];
        llh.lat() = to_radian(llh.lat());
        llh.lon() = j["lon"];
        llh.lon() = to_radian(llh.lon());
        llh.alt() = j["height"];      // in mm
        llh.alt() = llh.alt() / 1000; // in M
        // j["msgID"];
        // j["msgmode"];
        // fix = static_cast<GGA::Fix>(j["quality"]);
    }

    UTC_Time time;         // UTC time - hhmmss.ss
    uint8_t num_satalites; // Number of satellites used (0-12)

    double accHeading;
    double accPitch;
    double accRoll;
    double veh_heading;
    double mot_heading;
    double pitch;
    double roll;
    LLH llh;
    double height;
};

struct Esf_Ins {
    int32_t iTOW;
    int32_t length;
    int8_t msgmode;
    int8_t version;
    double xAccel;
    double xAccelValid;
    double xAngRate;
    double xAngRateValid;
    double yAccel;
    double yAccelValid;
    double yAngRate;
    double yAngRateValid;
    double zAccel;
    double zAccelValid;
    double zAngRate;
    double zAngRateValid;
};

struct Ublox_Msgs {
    Nav_Att nav_att;
    Nav_Pvat nav_pvat;
    GGA gga;
    // Esf_Ins esf_ins;
};

class Ublox : public Sensor<Ublox_Msgs>
{
    // Ublox_Msgs msg = {};

    std::optional<Nav_Pvat> nav_pvat;
    std::optional<Nav_Att> nav_att;
    std::optional<GGA> gga;
    // std::optional<Esf_Ins> esf_ins;

  public:
    Ublox() {};
    TCP_Socket socket = TCP_Socket("127.0.0.1", 50020);
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> buf;

    void loop() override;
    bool start() override;

    void consume(Msg_Type sensor);
    template<typename T> std::optional<T> get_latest(Msg_Type sensor);
    bool update_speed(Velocity2d vel);
};

void update_position(Ublox &ublox, Frames &frames);
void update_heading(Ublox &ublox, Frames &frames, Heading &heading);
