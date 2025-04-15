#pragma once
#include "socket.hpp"
#include <cstdint>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <thread>

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D
struct GGA
{
    char Time[10];   // UTC time - hhmmss.ss
    char Lat[13];    // Latitude (degrees & minutes) - ddmm.mmmmm
    char NS;         // North/South indicator
    char Long[14];   // Longitude (degrees & minutes) - dddmm.mmmmm
    char EW;         // East/West indicator
    uint8_t Quality; // Quality indicator for position fix
    uint8_t NumSV;   // Number of satellites used (0-12)
    float HDOP;      // Horizontal Dilution of Precision
    float Alt;       // Altitude above mean sea level - meters
};

#define GGA_BUFFER_LENGTH 128
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

class Sensor_Manager
{
    Ublox ublox;

  public:
    Sensor_Manager() { sensors_thread = std::thread(&Sensor_Manager::loop, this); }
    ~Sensor_Manager() { sensors_thread.detach(); }
    std::thread sensors_thread;
    void loop();
    void readSensors();
};
