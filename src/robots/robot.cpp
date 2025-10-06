#include "robot.hpp"
#include "linear_controller.hpp"

bool
Robot::pause()
{
    paused = true;
    return true;
}

bool
Robot::stop()
{
    running = false;
    return true;
}

bool
Robot::resume()
{
    paused = false;
    return true;
}


PVA 
Robot::get_PVA(){
    return pva;
}

bool
Reader::init(Robot &r)
{

    std::cerr << "Starting Reader" << std::endl;
    socket.port = 55555;
    socket.fd = -1;
    robot = &r;
    if (running) return true;
    if (!tcp_server_setup(socket)) {
        std::cerr << "Reader couldn't connect to socket" << std::endl;
        return false;
    };

    if (socket.fd < 0) {
        return false;
    }

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Reader::loop, this);
    running = true;
    return true;
}

void
Reader::loop()
{
    // tcp_server_start(socket);
    while (!(socket.client_socket < 0) && running) {
        if (!socket_recv(socket.client_socket, recv_buf, buf)) {
            running = false;
            // socket.disconnect();
        }
        for (size_t i = 0; i < buf.count(); i++) {
            if (buf[i] == '\n') {
                int len = i + 1; // i + 1 to include the newline
                std::string msg(len, '\0');
                buf.read(std::span(msg.data(), len));
                i = 0;
                json j;
                try {
                    j = json::parse(msg);
                } catch (nlohmann::json::parse_error &e) {
                    std::cerr << "Parse error at byte " << e.byte << ": " << e.what() << std::endl;
                    std::cout << msg.size() << " | " << msg.substr((e.byte - 10), 20) << std::endl;
                    std::cout << msg << std::endl;
                    break;
                } catch (nlohmann::json::exception &e) {
                    std::cerr << "Other JSON error: " << e.what() << std::endl;
                    break;
                }
                std::cout << j.dump() << std::endl;
                auto command = j["command"];
                if (command == "pause") {
                    robot->pause();
                    std::cout << "robot.pause : " << robot->paused << std::endl;
                }
                if (command == "stop") {
                    robot->stop();
                }

                if (command == "continue") {
                    robot->resume();
                }

                if (command == "pva") {
                    PVA pva = robot->get_PVA();
                    json pva_j = {
                        {"position", {"x", pva.pose.point.x(), "y", pva.pose.point.y(), "z", pva.pose.point.z()}},
                        {"velocity", 
                            {
                                "linear", 
                                {
                                    "x", pva.linear.velocity.x(),
                                    "y", pva.linear.velocity.y(),
                                    "z", pva.linear.velocity.z()
                                },
                                "angular", 
                                {
                                    "x", pva.angular.velocity.x(),
                                    "y", pva.angular.velocity.y(),
                                    "z", pva.angular.velocity.z()
                                },

                            },
                        },
                        {"acceleration", 
                            {
                                "linear", 
                                {
                                    "x", pva.linear.acceleration.x(),
                                    "y", pva.linear.acceleration.y(),
                                    "z", pva.linear.acceleration.z()
                                },
                                "angular", 
                                {
                                    "x", pva.angular.acceleration.x(),
                                    "y", pva.angular.acceleration.y(),
                                    "z", pva.angular.acceleration.z()
                                },

                            },
                        },
                    };

                    std::string send_pva = pva_j.dump(4);

                    tcp_send(robot->TCP_reader.socket.client_socket, send_pva.data(), send_pva.length());
                }
            }
        }
    }
}
