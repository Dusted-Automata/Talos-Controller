#include "socket.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

bool TCP_Subscriber::connect()
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

bool TCP_Subscriber::listen()
{
    ssize_t bytes_received = recv(socket_fd, buf.data(), buf.size(), 0);
    if (bytes_received <= 0)
    {
        std::cerr << "no bytes received" << std::endl;
        close(socket_fd);
        return false;
    }
    return true;
}
