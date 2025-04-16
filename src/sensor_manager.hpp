#pragma once
#include "ublox.hpp"
#include <netinet/in.h>
#include <thread>

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D

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
