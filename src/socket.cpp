#include "socket.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>

bool
TCP_Socket::connect()
{
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
        ::close(socket_fd);
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;
    return true;
}

inline bool
NMEA_Parser::findStart(Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &buf, size_t &i)
{
    for (; i < buf.count(); i++) {
        if (buf[i] == '$') {
            ++i;
            return true;
        }
    }
    return false;
}

inline bool
NMEA_Parser::findEnd(Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &buf, size_t &i)
{

    for (; i < buf.count(); i++) {
        if (buf[i - 1] == '\r' && buf[i] == '\n') {
            return true;
        }
    }
    return false;
}

void
NMEA_Parser::process_data(
    std::queue<std::string> &msgs, std::array<char, TCP_BUFFER_LENGTH> &buf, size_t bytes_received)
{
    ring.write(std::span(buf.data(), bytes_received));

    size_t index = 0;
    while (index < ring.count()) {
        index = 0;
        if (!findStart(ring, index)) {
            ring.clear();
            break;
        }

        if (!findEnd(ring, index)) {
            break;
        }

        size_t len = index + 1;
        std::string sentence(len, '\0');
        ring.read(std::span(sentence.data(), len));
        msgs.push(std::move(sentence));
    }
};

bool
TCP_Socket::recv(std::queue<std::string> &msgs)
{
    ssize_t bytes_received;
    while ((bytes_received = ::recv(socket_fd, recv_buf.data(), recv_buf.size(), 0)) > 0) {
        // std::cout << "bytes_received: " << bytes_received << std::endl;
        // std::cout << "recv: " << recv_buf.data() << std::endl;
        parser.process_data(msgs, recv_buf, bytes_received);
    }

    if (bytes_received == 0) {
        std::cerr << "socket closed" << std::endl;
        close(socket_fd);
        return true;
    }

    if (bytes_received < 0) {
        std::cerr << "recv failed" << std::endl;
        close(socket_fd);
        return false;
    }

    return true;
}

/*std::string TCP_Socket::ublox_Message(char *buffer, size_t buffer_size)*/
/*{*/
/*    return parseMessage(buffer, buffer_size);*/
/*};*/
