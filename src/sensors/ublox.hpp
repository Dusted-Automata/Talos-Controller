#pragma once
#include "NMEA.hpp"
#include "sensor.hpp"
#include "socket.hpp"
#include <json.hpp>
#include <queue>
#include <vector>

using nlohmann::json;
class Ublox_GGA : public Sensor<GGA>
{
    NMEA_Parser parser = {};
    Sensor_Name name = Sensor_Name::UBLOX_GGA;

  public:
    Ublox_GGA() {};
    TCP_Socket socket = TCP_Socket("127.0.0.1", 50010, parser);
    // TCP_Socket socket = TCP_Socket("192.168.123.161", 50010, parser);
    // TCP_Socket socket = TCP_Socket("192.168.12.1", 50010, parser);
    std::queue<std::string> buf;
    std::vector<GGA> msgs;

    void process() override;
    void loop() override;
    void start() override;
};

struct NAV_ATT {
    // std::string identity": "NAV-ATT",
    NAV_ATT(json j)
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

struct ESF_INS {
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

struct Simple_Ublox {
    NAV_ATT nav_att;
    ESF_INS esf_ins;
};

class Ublox_simple : public Sensor<json>
{
    JSON_Parser parser = {};
    Sensor_Name name = Sensor_Name::UBLOX_SIMPLE;

  public:
    Ublox_simple() {};
    TCP_Socket socket = TCP_Socket("127.0.0.1", 50020, parser);
    // TCP_Socket socket = TCP_Socket("192.168.123.161", 50010, parser);
    // TCP_Socket socket = TCP_Socket("192.168.12.1", 50010, parser);
    std::queue<std::string> buf;
    std::vector<json> msgs;

    void process() override;
    void loop() override;
    void start() override;
};
