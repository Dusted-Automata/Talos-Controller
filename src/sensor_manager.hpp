#pragma once
#include <array>
#include <netinet/in.h>
#include <string>
#include <thread>

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
    Ublox(){};

    bool connect();
    bool listen();
    Ubx_Nav_Pvt read();
};

class Sensor_Manager
{
    Ublox ublox;

  public:
    Sensor_Manager()
    {
        ublox.connect();
        sensors_thread = std::thread(&Sensor_Manager::loop, this);
    }
    ~Sensor_Manager() { sensors_thread.detach(); }
    std::thread sensors_thread;
    void loop();
    void readSensors();
};
