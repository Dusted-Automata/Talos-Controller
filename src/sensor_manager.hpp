#pragma once
#include "types.hpp"
#include "ublox.hpp"
#include <netinet/in.h>
#include <optional>
#include <thread>

// https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf#%5B%7B%22num%22%3A64%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C0%2C609.45%2Cnull%5D

enum Sensor_Name { UBLOX };
struct Latest_Measurement {
    GGA ublox_measurement;
};

class Sensor
{
  public:
    virtual ~Sensor() = default;
    virtual std::string name() const = 0;
    virtual double nominal_rate_hz() const = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    // virtual std::optional<Measurement<>> poll() = 0;
};

class Sensor_Manager
{
    std::vector<std::unique_ptr<Sensor> > sensors;
    std::atomic_bool running = false;
    Ublox ublox;

  public:
    Sensor_Manager() { sensors_thread = std::thread(&Sensor_Manager::loop, this); }
    ~Sensor_Manager() { sensors_thread.detach(); }
    Latest_Measurement latest_measurement = {};
    std::thread sensors_thread;
    void loop();
    void readSensors();
};
