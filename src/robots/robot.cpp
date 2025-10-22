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

