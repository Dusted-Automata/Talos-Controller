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

    for (size_t i = 0; i < ring.count(); i++) {
        if (ring[i] == '\n') {
            int len = i + 1; // i + 1 to include the newline
            std::string msg(len, '\0');
            ring.read(std::span(msg.data(), len));
            json j = json::parse(msg);
            if (j["identity"] == "GPGGA" || j["identity"] == "NAV-ATT") {
                return std::optional(j);
            }
        }
    }

    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    // bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
    bytes_received = ::recv(fd, ring.data(), ring.contigues_space_from_head(), 0);
    // ring.write(std::span(recv_buf.data(), bytes_received));

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

    return std::nullopt;
}
