#include "include/unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

using namespace UNITREE_LEGGED_SDK;

class Robot {
public:
	Robot() : safe(LeggedType::Go1),
			  udp(HIGHLEVEL, 8090, "192.168.123.161", 8082)
	{
		HighCmd cmd = {0};
		udp.InitCmdData(cmd); 
	}

	void control_loop();

	Safety safe;
	UDP udp;
	HighState state = {0};
	int motiontime = 0;
	float dt = 0.002;

};

// void UDPRecv() {
// 	udp.Recv();
// }
//
// void UDPSend() {
// 	udp.Send();
// }
//
void Robot::control_loop() {}


int main(void) {
	std::cout << "Robot level set to: HIGH" << std::endl
			  << "WARNING: Make sure the robot is standing on the ground." << std::endl
			  << "Press Enter to continue..." << std::endl;
	std::cin.ignore();

	Robot robot;

	LoopFunc loop_control("control_loop", robot.dt, boost::bind(&Robot::control_loop, &robot));
	LoopFunc loop_udpSend("udp_send", robot.dt, 3, boost::bind(&UDP::Recv, &robot.udp));
	LoopFunc loop_udpRecv("udp_recv", robot.dt, 3, boost::bind(&UDP::Send, &robot.udp));

	loop_udpSend.start();
	loop_udpRecv.start();
	loop_control.start();

	while (1){
		sleep(10);
	}

	return 0;

} 
