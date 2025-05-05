#include "socket.hpp"
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

uint8_t
NMEA_Parser::hex_Val(char c)
{
    return static_cast<uint8_t>(std::isdigit(c) ? c - '0' : std::toupper(c) - 'A' + 10);
}

bool
NMEA_Parser::verify_Checksum(std::string_view s)
{
    // "$GPGGA,....*hh\r\n"
    size_t star_index = s.length() - 5;
    assert(s[star_index] == '*');

    uint8_t calc = 0;
    for (size_t i = 1; i < star_index; ++i) { // skipping $
        calc ^= static_cast<uint8_t>(s[i]);
    }
    uint8_t got = static_cast<uint8_t>(hex_Val(s[star_index + 1]) << 4) | hex_Val(s[star_index + 2]);
    return calc == got;
};

bool
NMEA_Parser::skip_UBX()
{
    if (ring.count() < 6) { // sync + class/id + len
        return false;
    }
    uint16_t len = static_cast<uint16_t>(static_cast<uint8_t>(ring[4]) | (static_cast<uint8_t>(ring[5]) << 8));
    std::size_t frameLen = 6 + len + 2; // + payload + CK_A/B

    if (frameLen > ring.capacity()) {
        ring.clear(2);
        return true;
    }
    if (ring.count() < frameLen) {
        return false;
    }
    ring.clear(frameLen);
    return true;
};
bool
NMEA_Parser::extract_NMEA(std::string &out)
{
    while (!ring.empty() && ring[0] != '$') {
        ring.clear();
    }
    if (ring.empty()) {
        return false;
    }

    std::size_t MAX_LEN = 82; // NMEA frame length max inc. CRLF

    for (std::size_t i = 1; i < ring.count() && i < MAX_LEN; ++i) {
        char c = ring[i];
        if ((c < 0x20 || c > 0x7E) && c != '\r' && c != '\n') {
            break;
        }

        if (ring[i] == '\n' && ring[i - 1] == '\r') {
            if (!(ring[i - 4] == '*') || !isxdigit(ring[i - 3]) || !isxdigit(ring[i - 2])) {
                break;
            }

            std::size_t len = i + 1; // include CR/LF
            out.resize(len);
            ring.read(std::span(out.data(), len));
            return verify_Checksum(out);
        }
    }
    ring.clear(); // bad data
    return true;
};

void
NMEA_Parser::push(std::queue<std::string> &msgs, std::span<const char> data)
{
    ring.write(data);
    while (true) {
        // UBX frame at buffer start?  0xB5 0x62
        if (ring.count() >= 2 && ring[0] == static_cast<char>(0xB5) && ring[1] == static_cast<char>(0x62)) {
            if (!skip_UBX()) {
                break;
            }
            continue;
        }

        std::string frame;
        if (!extract_NMEA(frame)) {
            break;
        }
        if (!frame.empty()) msgs.push(std::move(frame));
    }
}

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
        ::close(socket_fd);
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;

    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);

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

    while (true) {
        bytes_received = ::recv(socket_fd, recv_buf.data(), recv_buf.size(), 0);
        if (bytes_received == 0) {
            std::cerr << "socket closed" << std::endl;
            close(socket_fd);
            socket_fd = -1;
            return true; // if it returns false that would make it so that poll does not process the msgs.
        }
        if (bytes_received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            } else {
                std::cerr << "recv failed" << std::endl;
                close(socket_fd);
                socket_fd = -1;
                return false;
            }
        }
        parser.push(msgs, std::span(recv_buf.data(), bytes_received));
    }

    return true;
}
