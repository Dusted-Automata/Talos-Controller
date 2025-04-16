#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <unistd.h>

void Sensor_Manager::loop()
{
    ublox.poll();
    /*while (true)*/
    /*{*/
    /*    ublox.listen();*/
    /*}*/
}
void Sensor_Manager::readSensors() { ublox.read(); }
