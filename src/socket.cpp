#include "socket.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <vector>

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

inline bool NMEA_Parser::findStart(Ring_Buffer<char> &buf, size_t &i)
{
    for (; i < buf.count(); i++)
    {
        if (buf[i] == '$')
        {
            ++i;
            return true;
        }
    }
    return false;
}

inline bool NMEA_Parser::findEnd(Ring_Buffer<char> &buf, size_t &i)
{

    for (; i < buf.count(); i++)
    {
        if (buf[i-1] == '\r' && buf[i] == '\n')
        {
            return true;
        }
    }
    return false;
}

void NMEA_Parser::process_data(std::vector<std::string> &msgs,
                               std::array<char, TCP_BUFFER_LENGTH> &buf, size_t bytes_received)
{ 
    ring.write(buf.data(), bytes_received);

    size_t index = 0;
    while (index < ring.count()) {
        index = 0;
        if (findStart(ring, index))
        {
            if (findEnd(ring, index))
            {
                size_t sentence_length = index + 1;
                msgs.emplace_back(sentence_length, '\0');
                ring.read(&msgs.back()[0], sentence_length);
            }
            else
            {
                break;
            }
        }
        else
        {
            ring.clear();
        }
    }
};

bool TCP_Socket::recv(std::vector<std::string> &msgs)
{
    ssize_t bytes_received;
    while ((bytes_received = ::recv(socket_fd, recv_buf.data(), recv_buf.size(), 0)) > 0) 
    {
        // std::cout << "bytes_received: " << bytes_received << std::endl;
        // std::cout << "recv: " << recv_buf.data() << std::endl;
        parser.process_data(msgs, recv_buf, bytes_received);
    }

    if (bytes_received == 0)
    {
        std::cerr << "socket closed" << std::endl;
        close(socket_fd);
        return true;
    }

    if (bytes_received < 0)
    {
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
