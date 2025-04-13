#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <unistd.h>

bool Ublox::connect()
{
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1)
    {
        std::cerr << "Could not create socket" << std::endl;
        return false;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(server_ip.c_str());
    server.sin_port = htons(port);

    int con = ::connect(socket_fd, (struct sockaddr *)&server, sizeof(server));

    if (con < 0)
    {
        std::cerr << "Connection failed to " << server_ip << ":" << port << std::endl;
        ::close(socket_fd);
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;
    return true;
}

bool recv_all(int sockfd, void *buffer, size_t length)
{
    char *ptr = static_cast<char *>(buffer);
    size_t total = 0;

    while (total < length)
    {
        ssize_t n = recv(sockfd, ptr + total, length - total, 0);
        if (n <= 0)
            return false;
        total += n;
    }
    return true;
}

bool Ublox::listen()
{
    if (!recv_all(socket_fd, &rec_buf, sizeof(rec_buf)))
    {
        std::cerr << "recv failed" << std::endl;
        close(socket_fd);
        return false;
    }
    ring_buffer.at(buf_head) = rec_buf;
    buf_head = (++buf_head % ring_buffer.size());
    return true;
}

Ubx_Nav_Pvt Ublox::read()
{
    Ubx_Nav_Pvt val = ring_buffer.at(buf_tail);
    if (buf_tail != buf_head)
        buf_tail = (++buf_tail % ring_buffer.size());
    return val;
}

void Sensor_Manager::run() {}
