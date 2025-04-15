#include "socket.hpp"
#include <exception>
#include <ios>
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

inline size_t findStart(char *buf, int index)
{
    for (int i = index; i < TCP_BUFFER_LENGTH - 1; i++)
    {
        if (buf[i] == '$')
        {
            return i + 1;
        }
    }
    return index;
}

inline size_t findEnd(char *buf, int index)
{

    for (int i = index; i < TCP_BUFFER_LENGTH - 2; i++)
    {
        if (buf[i] == '\r' && buf[i + 1] == '\n')
        {
            return i - 1;
        }
    }
    return index;
}

bool TCP_Socket::recv(std::vector<std::string> &msgs)
{
    ssize_t bytes_received = ::recv(socket_fd, stream_buffers.recv, sizeof(stream_buffers.recv), 0);
    if (bytes_received < 1)
    {
        std::cerr << "recv failed" << std::endl;
        close(socket_fd);
        return false;
    }

    for (int i = 0; i < TCP_BUFFER_LENGTH - 1;)
    {
        if (stream_buffers.half_message.exists == true)
        {
            std::string beginning(stream_buffers.half_message.buf, stream_buffers.half_message.end);
            size_t end = findEnd(stream_buffers.recv, 0);
            std::string ending(stream_buffers.recv, end);
            msgs.push_back((beginning + ending));
            stream_buffers.half_message.exists = false;
        }
        size_t start = findStart(stream_buffers.recv, i);
        if (start == i) // means we did not find a start delimiter, and can exit
            break;
        size_t end = findEnd(stream_buffers.recv, start);
        if (start == end) // means we did not find an end delimiter, and need to save the message
        {

            memcpy(stream_buffers.half_message.buf, &stream_buffers.recv[end],
                   TCP_BUFFER_LENGTH - start);
            stream_buffers.half_message.exists = true;
            stream_buffers.half_message.end = start;
            break;
        }
        std::string str(&stream_buffers.recv[start], (end - start));
        msgs.push_back(str);
        i = end + 1;
    }

    return true;
}

std::vector<std::string> TCP_Socket::recv_all()
{
    std::vector<std::string> msgs;
    if (recv(msgs))
    {
        while (stream_buffers.half_message.exists == true)
        {
            recv(msgs);
        }
    }
    return msgs;
}

std::vector<std::string> extract_messages() { std::vector<std::string> msgs; }

/*std::string TCP_Socket::ublox_Message(char *buffer, size_t buffer_size)*/
/*{*/
/*    return parseMessage(buffer, buffer_size);*/
/*};*/
