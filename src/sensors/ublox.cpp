
#include "ublox.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
void
Ublox::loop()
{

    while (!(socket.get_fd() < 0) && running) {
        std::optional<json> j = socket.recv();
        if (j.has_value()) {
            if (j.value()["identity"] == "NAV-ATT") {
                std::cout << j.value().dump(4) << std::endl;
                nav_att = Nav_Att(j.value());
            }
        }
    }
}

void
Ublox::start()
{

    if (!socket.connect()) {
        std::cerr << "Ublx couldn't connect to json_socket" << std::endl;
        return;
    };

    if (socket.get_fd() < 0) {
        return;
    }

    if (running) return;
    running = true;

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Sensor::loop, this);
}
