#include "socket.hpp"
#include <fcntl.h>
#include <iostream>
#include <optional>
#include <unistd.h>

bool
TCP_Socket::connect()
{
    disconnect();
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        std::cerr << "Could not create socket" << std::endl;
        return false;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(server_ip.c_str());
    server.sin_port = htons(port);

    int con = ::connect(fd, (struct sockaddr *)&server, sizeof(server));

    if (con < 0) {
        std::cerr << "Connection failed to " << server_ip << ":" << port << std::endl;
        disconnect();
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;

    return true;
}

int
TCP_Socket::get_fd()
{
    return fd;
}

void
TCP_Socket::disconnect()
{
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

std::optional<json>
TCP_Socket::recv()
{
    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    bytes_received = ::recv(fd, recv_buf.data() + buf_index, recv_buf.size(), 0);

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

    for (int i = buf_index; i < buf_index + bytes_received; i++) {
        if (recv_buf[i] == '\n') {
            std::string msg(recv_buf.data(), i + 1); // i + 1 to include the newline
            json j = json::parse(msg);
            buf_index = 0;
            return std::optional<json>(j);
        }
    }

    return std::nullopt;
}
