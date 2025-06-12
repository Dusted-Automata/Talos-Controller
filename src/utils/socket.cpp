#include "socket.hpp"
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

bool
TCP_Socket::connect()
{
    disconnect();
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd == -1) {
        std::cerr << "Could not create socket" << std::endl;
        return false;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(server_ip.c_str());
    server.sin_port = htons(port);

    int con = ::connect(socket_fd, (struct sockaddr *)&server, sizeof(server));

    if (con < 0) {
        std::cerr << "Connection failed to " << server_ip << ":" << port << std::endl;
        disconnect();
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;

    // int flags = fcntl(socket_fd, F_GETFL, 0);
    // fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);

    return true;
}

void
TCP_Socket::disconnect()
{
    if (socket_fd != -1) {
        ::close(socket_fd);
        socket_fd = -1;
    }
}

bool
TCP_Socket::recv(std::queue<std::string> &msgs)
{
    if (socket_fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    bytes_received = ::recv(socket_fd, recv_buf.data(), recv_buf.size(), 0);
    if (bytes_received == 0) {
        std::cerr << "socket closed" << std::endl;
        disconnect();
        return false;
    }
    if (bytes_received < 0) {
        std::cerr << "recv failed" << std::endl;
        disconnect();
        return false;
    }
    parser.push(msgs, std::span(recv_buf.data(), bytes_received));

    return true;
}
