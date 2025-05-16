#pragma once
#include "NMEA.hpp"
#include <atomic>
#include <optional>
#include <thread>

enum Sensor_Name { UBLOX };

struct Measurement {
    GGA ublox;
    Sensor_Name source;
};

class Sensor
{
  public:
    virtual ~Sensor() { stop(); };
    virtual void start();
    virtual void stop();
    virtual void process() = 0;
    virtual void loop() = 0;
    // virtual std::optional<Measurement<>> poll() = 0;
  protected:
    std::atomic_bool running = false;
    std::thread sensor_thread;
    std::mutex measurement_mutex;
    std::optional<Measurement> latest_measurement = {};
    Sensor_Name name;
};
