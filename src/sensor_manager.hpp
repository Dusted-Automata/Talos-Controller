#pragma once
#include <array>
#include <netinet/in.h>
#include <string>

struct Ubx_Nav_Pvt
{
};

class Ublox
{

    int socket_fd;
    struct sockaddr_in server;
    std::string server_ip = "127.0.0.1";
    int port = 50012;
    std::array<Ubx_Nav_Pvt, 16> ring_buffer;
    int buf_head = 0;
    int buf_tail = 0;
    Ubx_Nav_Pvt rec_buf;

  public:
    Ublox() {};

    bool connect();
    bool listen();
    Ubx_Nav_Pvt read();
};

class Sensor_Manager
{

    void run();
};
