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

class Ublox_JSON : public Sensor<json>
{
    JSON_Parser parser = {};
    Sensor_Name name = Sensor_Name::UBLOX_JSON;

  public:
    Ublox_JSON() {};
    TCP_Socket socket = TCP_Socket("127.0.0.1", 50020, parser);
    // TCP_Socket socket = TCP_Socket("192.168.123.161", 50010, parser);
    // TCP_Socket socket = TCP_Socket("192.168.12.1", 50010, parser);
    std::queue<std::string> buf;
    std::vector<json> msgs;

    void process() override;
    void loop() override;
    void start() override;
};
