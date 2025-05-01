#pragma once
#include "types.hpp"
#include "ublox.hpp"
#include <netinet/in.h>
#include <optional>
#include <thread>

enum Sensor_Name { UBLOX };
struct Latest_Measurement {
    GGA ublox_measurement;
    bool read;
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
    std::vector<std::unique_ptr<Sensor>> sensors;
    std::atomic_bool running = false;
    Ublox ublox;

  public:
    Sensor_Manager() {}
    ~Sensor_Manager() { shutdown(); }
    Latest_Measurement latest_measurement = {};
    std::thread sensors_thread;
    void loop();
    void readSensors();
    void init();
    void shutdown();
};
