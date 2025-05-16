#pragma once
#include "NMEA.hpp"
#include "sensor.hpp"
#include "socket.hpp"
#include <iostream>
#include <queue>
#include <vector>

class Ublox : public Sensor
{
    NMEA_Parser parser = {};
    Sensor_Name name = Sensor_Name::UBLOX;

  public:
    Ublox() {};
    // TCP_Socket socket = TCP_Socket("127.0.0.1", 50010, parser);
    TCP_Socket socket = TCP_Socket("192.168.123.161", 50010, parser);
    // TCP_Socket socket = TCP_Socket("192.168.12.1", 50010, parser);
    std::queue<std::string> buf;
    std::vector<GGA> msgs;

    void process() override;
    void loop() override;
    void start() override;
};
