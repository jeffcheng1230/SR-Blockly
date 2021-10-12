// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "RobotClass.hpp"
#include <iostream>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
#define FOREVERMAIN bot->robot->step(bot->timeStep) != -1

// "controllerArgs" field of the Robot node

int main(int argc, char** argv) {
	theRobot* bot = new theRobot();
	while (FOREVERMAIN)
	{
		bot->lineTrace();
		bot->obstacle();
		bot->green();
	}
	return 0;
}
