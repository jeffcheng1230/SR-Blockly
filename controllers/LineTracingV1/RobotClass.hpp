#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>	
#include <math.h>
#include <iostream>
using namespace std;
using namespace webots;
#define WHEELBASE 39
#define WHEELRAD 9.4
#define MAX_VEL 12
#define FOREVER robot->step(timeStep) != -1
#define PI 3.14159
class theRobot {
public:

	Robot* robot;
	Motor* left;
	Motor* right;
	PositionSensor* LEnc;
	PositionSensor* REnc;
	Camera* leftSensor;
	Camera* rightSensor;
	Camera* midSensor;
	DistanceSensor* topDist;
	DistanceSensor* bottomDist;
	int timeStep;
	theRobot()
	{
		robot = new Robot();
		timeStep = (int)robot->getBasicTimeStep();
		left = robot->getMotor("left wheel");
		right = robot->getMotor("right wheel");
		LEnc = left->getPositionSensor();
		REnc = right->getPositionSensor();
		LEnc->enable(timeStep);
		REnc->enable(timeStep);
		leftSensor = robot->getCamera("leftSens");
		rightSensor = robot->getCamera("rightSens");
		midSensor = robot->getCamera("middleSens");
		leftSensor->enable(timeStep);
		rightSensor->enable(timeStep);
		midSensor->enable(timeStep);
		topDist = robot->getDistanceSensor("topDist");
		topDist->enable(timeStep);
		bottomDist = robot->getDistanceSensor("bottomDist");
		bottomDist->enable(timeStep);
		robot->step(timeStep);
	}
	void lineTrace()
	{
		int tp = 5;
		left->setPosition(INFINITY);
		right->setPosition(INFINITY);
		int vals = 0;
		vals += isBlack(leftSensor);
		vals = vals << 1;
		vals += isBlack(midSensor);
		vals = vals << 1;
		vals += isBlack(rightSensor);
		switch (vals)
		{
		case 0: //000
			//cout << "all white" << endl;
			left->setVelocity(tp);
			right->setVelocity(tp);
			//for (int x = 0; x < 10 && FOREVER; x++);
			break;
		case 1://001
			//cout << "right" << endl;
			left->setVelocity(tp);
			right->setVelocity(-tp * 1.3);
			break;
		case 4://100
			//cout << "left" << endl;
			left->setVelocity(-tp * 1.3);
			right->setVelocity(tp);
			break;
		case 6://110
			//cout << "light left" << endl;
			left->setVelocity(tp / 10);
			right->setVelocity(tp);
			break;
		case 3://011
			//cout << "light right" << endl;
			left->setVelocity(tp);
			right->setVelocity(tp / 10);
			break;
		default:
			//cout << "default" << endl;
			left->setVelocity(tp);
			right->setVelocity(tp);
			break;
		}
		return;
	}
	void moveForward(double cm)
	{
		bool flag = true;
		left->setPosition(INFINITY);
		right->setPosition(INFINITY);
		LEnc->enable(timeStep);
		//cout << "Starting move ; ";
		//cout << initEnc << "   " << initEnc + cm / WHEELRAD << endl;
		double goal = LEnc->getValue() + cm / WHEELRAD;
		double error;
		double kP = 1.5;
		double velocity = 0;
		while (robot->step(timeStep) != -1 && flag)
		{
			error = goal - LEnc->getValue();
			//cout << error << endl;
		   // cout << initEnc << "   " << initEnc + cm / WHEELRAD << " ";
			//cout << error << endl;
			velocity = kP * error;
			if (velocity < 1 && velocity > 0) velocity = 1;
			if (velocity > -1 && velocity < 0) velocity = -1;
			if (velocity < -1 * MAX_VEL) velocity = -1 * MAX_VEL;
			if (velocity > MAX_VEL) velocity = MAX_VEL;
			left->setVelocity(velocity);
			right->setVelocity(velocity);

			if (error > -0.018 && error < 0.018)
			{
				flag = false;
				// cout << "no error" << endl;
				left->setVelocity(0);
				right->setVelocity(0);
			}
		}
		//dealWithCamera();
	}
	void turnRight(double ang)
	{
		bool flag = true;
		left->setPosition(INFINITY);
		right->setPosition(INFINITY);
		LEnc->enable(timeStep);
		//cout << "Starting move ; ";
		//cout << initEnc << "   " << initEnc + cm / WHEELRAD << endl;
		double goal = LEnc->getValue() + ((ang / 360) * WHEELBASE * PI) / WHEELRAD;
		double error;
		double kP = 1.5;
		double velocity = 0;
		while (robot->step(timeStep) != -1 && flag)
		{
			error = goal - LEnc->getValue();
			//cout << error << endl;
		   // cout << initEnc << "   " << initEnc + cm / WHEELRAD << " ";
			//cout << error << endl;
			velocity = kP * error;
			if (velocity < 0.5 && velocity > 0) velocity = 0.5;
			if (velocity > -0.5 && velocity < 0) velocity = -0.5;
			if (velocity < -0.5 * MAX_VEL - 2) velocity = -0.5 * MAX_VEL - 2;
			if (velocity > MAX_VEL - 2) velocity = MAX_VEL - 2;
			left->setVelocity(velocity);
			right->setVelocity(-1 * velocity);

			if (error > -0.018 && error < 0.018)
			{
				flag = false;
				// cout << "no error" << endl;
				left->setVelocity(0);
				right->setVelocity(0);
			}
		}
	}
	void obstacle()
	{
		cout << topDist->getValue() << " " << bottomDist->getValue() << endl;
		if (bottomDist->getValue() > 15) return;
		if (abs(topDist->getValue() - bottomDist->getValue()) > 1)
		{
			cout << "RAMP!!!!" << endl;
			return;
		}
		else
		{
			cout << "OBSTACLE" << endl;
			left->setVelocity(0);
			right->setVelocity(0);
			turnRight(90);
			moveForward(WHEELBASE + 7);
			turnRight(-90);
			moveForward(WHEELBASE * 3.5);
			turnRight(-90);
			bool flag = true;
			while (FOREVER && flag)
			{
				left->setVelocity(1.3);
				right->setVelocity(1.3);
				if (isBlack(midSensor))
				{
					flag = false;
					left->setVelocity(0);
					right->setVelocity(0);
				}
			}
			moveForward(WHEELBASE / 2);
			turnRight(90);
			return;
		}
	}
	void green()
	{
		if (isGreen(leftSensor))
		{
			cout << "saw left" << endl;
			moveForward(-1.5);
			if (isBlack(leftSensor))
			{
				cout << "fake left" << endl;
				moveForward(3);
				return;
			}
			moveForward(3.5);
			while (FOREVER)
			{
				left->setVelocity(0);
				right->setVelocity(1);
				if (isGreen(rightSensor))
				{
					cout << "double green" << endl;
					doubleGreen();
					return;
				}
				if (isBlack(rightSensor))
				{
					cout << "left turn" << endl;
					leftTurn();
					return;
				}
			}
		}
		if (isGreen(rightSensor))
		{
			cout << "saw right" << endl;
			moveForward(-1.5);
			if (isBlack(rightSensor))
			{
				cout << "fake right" << endl;
				moveForward(3);
				return;
			}
			moveForward(3.5);
			while (FOREVER)
			{
				left->setVelocity(1);
				right->setVelocity(0);
				if (isGreen(leftSensor))
				{
					cout << "double green" << endl;
					doubleGreen();
					return;
				}
				if (isBlack(leftSensor))
				{
					cout << "right turn" << endl;
					rightTurn();
					return;
				}
			}
		}
	}
	
private:
	bool isBlack(Camera* cam)
	{
		const unsigned char* image = cam->getImage();

		int grayVal = cam->imageGetGray(image, cam->getWidth(), cam->getWidth() / 2, cam->getHeight() / 2);
		if (grayVal < 90)
		{
			return true;
		}
		return false;
	}
	bool isGreen(Camera* cam)
	{
		const unsigned char* image = cam->getImage();
		double greenVal = cam->imageGetGreen(image, cam->getWidth(), cam->getWidth() / 2, cam->getHeight() / 2);
		double redVal = cam->imageGetRed(image, cam->getWidth(), cam->getWidth() / 2, cam->getHeight() / 2);
		double comp = greenVal / (redVal + 0.001);
		//cout << (int)greenVal << " " << (int)redVal << " " << comp << endl;
		if (comp > 1.3)
		{
			return true;
		}
		return false;
	}
	void doubleGreen()
	{
		moveForward(WHEELBASE);
		turnRight(180);
	}
	void leftTurn()
	{
		moveForward(WHEELBASE * 0.2);
		turnRight(-35);
		while (FOREVER && !isBlack(midSensor))
		{
			left->setVelocity(-1);
			right->setVelocity(2);
		}
		moveForward(5);
	}
	void rightTurn()
	{
		moveForward(WHEELBASE * 0.2);
		turnRight(35);
		while (FOREVER && !isBlack(midSensor))
		{
			left->setVelocity(2);
			right->setVelocity(-1);
		}
		moveForward(5);
	}
};