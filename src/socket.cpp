#include "socket.hpp"
#include <iostream>
#include <unistd.h>

bool TCP_Socket::connect()
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

bool TCP_Socket::listen(char *buffer, size_t buffer_size)
{
    ssize_t bytes_received = recv(socket_fd, buffer, buffer_size, 0);
    if (bytes_received < 1)
    {
        std::cerr << "recv failed" << std::endl;
        close(socket_fd);
        return false;
    }

    /*if (!recv_all(socket_fd, buf.data(), buf.size()))*/
    /*{*/
    /*    std::cerr << "recv failed" << std::endl;*/
    /*    close(socket_fd);*/
    /*    return false;*/
    /*}*/

    return true;
}
