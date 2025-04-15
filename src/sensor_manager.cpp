#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>

bool Ublox::parseMessage(std::array<char, 4096> buf)
{
    int i = 0;
    while (i < buf.size() - 1)
    {
        if (buf.at(i) == '\r' && buf.at(i + 1) == '\n')
            break;
        i++;
    }
    std::string string(buf.data(), i);
    std::cout << string << std::endl;

    return true;
}

bool Ublox::connect()
{
    if (!tcp.connect())
    {
        return false;
    }
    return true;
}

bool Ublox::listen()
{
    if (!tcp.listen(buf.data(), buf.size()))

    {
        return false;
    }
    return true;
}

GGA Ublox::read() { return GGA{}; };

void Sensor_Manager::loop()
{
    while (true)
    {
        ublox.listen();
    }
}
void Sensor_Manager::readSensors() { ublox.read(); }
