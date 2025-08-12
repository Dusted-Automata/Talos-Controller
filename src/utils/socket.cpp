#include "socket.hpp"
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

bool
TCP_Client::connect()
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

bool
tcp_server_setup(TCP_Server &server)
{
    // Create socket file descriptor
    if ((server.fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return false;
    }

    // Forcefully attaching socket to the port
    if (setsockopt(server.fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &server.opt, sizeof(server.opt))) {
        perror("setsockopt");
        return false;
    }

    server.address.sin_family = AF_INET;
    server.address.sin_addr.s_addr = INADDR_ANY;
    server.address.sin_port = htons(server.port);

    // Bind the socket to the network address and port
    if (bind(server.fd, (struct sockaddr *)&server.address, sizeof(server.address)) < 0) {
        perror("bind failed");
        return false;
    }

    std::cout << "Server initialized on port " << server.port << std::endl;

    if (listen(server.fd, 3) < 0) {
        perror("listen");
        return false;
    }

    std::cout << "Server listening for connections..." << std::endl;

    // Accept a connection
    if ((server.client_socket = accept(server.fd, (struct sockaddr *)&server.address, (socklen_t *)&server.addrlen))
        < 0) {
        perror("accept");
        return false;
    }

    std::cout << "Connection accepted from " << inet_ntoa(server.address.sin_addr) << std::endl;
    return true;
}

void
tcp_server_handle_client(TCP_Server &server)
{

    char buffer[1024] = { 0 };
    const char *response = "Hello from server!";

    // Read message from client
    int valread = read(server.client_socket, buffer, 1024);
    if (valread > 0) {
        std::cout << "Message from client: " << buffer << std::endl;

        // Echo the message back to client
        send(server.client_socket, buffer, strlen(buffer), 0);
        std::cout << "Echo message sent" << std::endl;
    }

    // Send a response
    send(server.client_socket, response, strlen(response), 0);
    std::cout << "Response sent to client" << std::endl;
}

void
tcp_server_start(TCP_Server &server)
{
    // Put the server socket in a passive mode, where it waits for client connections
    if (listen(server.fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening for connections..." << std::endl;

    while (true) {
        // Accept a connection
        if ((server.client_socket = accept(server.fd, (struct sockaddr *)&server.address, (socklen_t *)&server.addrlen))
            < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }

        std::cout << "Connection accepted from " << inet_ntoa(server.address.sin_addr) << std::endl;

        // Handle the client in a separate function
        tcp_server_handle_client(server);

        // Close the client socket
        close(server.client_socket);
    }
}

bool
socket_recv(int fd, std::array<char, TCP_BUFFER_LENGTH> recv_buf, Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring)

{

    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
    // bytes_received = read(fd, recv_buf.data(), recv_buf.size());
    ring.write(std::span(recv_buf.data(), bytes_received));

    if (bytes_received == 0) {
        std::cerr << "socket closed" << std::endl;
        ::close(fd);
        return false;
    }
    if (bytes_received < 0) {
        std::cerr << "recv failed" << std::endl;
        ::close(fd);
        return false;
    }

    return true;
}

int
TCP_Client::get_fd()
{
    return fd;
}

void
TCP_Client::disconnect()
{
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

bool
TCP_Client::recv(Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring)
{

    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
    ring.write(std::span(recv_buf.data(), bytes_received));

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

    return true;
}

bool
TCP_Client::send(const char *buf, size_t length)
{
    size_t total_sent = 0;
    while (total_sent < length) {
        ssize_t bytes_sent = ::send(fd, buf + total_sent, length - total_sent, 0);
        if (bytes_sent <= 0) {
            if (errno == EINTR) continue; // Retry if interrupted
            std::cerr << "Send error: " << strerror(errno) << std::endl;
            return false;
        }
        total_sent += bytes_sent;
    }
    return false;
}
