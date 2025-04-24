#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <unistd.h>

void Sensor_Manager::loop()
{
	// while (true)
	// {
	// }
    ublox.poll();
	while (!ublox.msgs.empty())
	{
		GGA gga = ublox.msgs.front();
		latest_measurement.ublox_measurement = std::move(gga);
		ublox.msgs.pop();
	}
}
