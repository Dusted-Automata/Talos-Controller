#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>
#include <vector>

bool Ublox::poll()
{
    std::vector<std::string> msgs = tcp.recv_all();
    for (auto i : msgs)
    {
        std::cout << i << std::endl;
    }
    /*if (!tcp.recv())*/
    /*{*/
    /*    return false;*/
    /*}*/
    /*tcp.ublox_Message(buf, buf.size());*/
    return true;
}

GGA Ublox::read() { return GGA{}; };

void Sensor_Manager::loop()
{
    ublox.poll();
    /*while (true)*/
    /*{*/
    /*    ublox.listen();*/
    /*}*/
}
void Sensor_Manager::readSensors() { ublox.read(); }
