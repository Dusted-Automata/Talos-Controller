#pragma once
#include "socket.hpp"
#include <thread>
#include <vector>

class Robot;

class Server
{
  public:
    std::thread socket_thread;
    std::thread msg_parse_thread;
    TCP_Server socket;
    Robot *robot;
    std::vector<int> subscribers;

    bool init(Robot &robot);
    void loop();
};

void parse_msgs(Server &server); 
bool server_init(Server &server, Robot &r);
