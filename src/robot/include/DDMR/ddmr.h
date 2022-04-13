#ifndef __DDMR_H__
#define __DDMR_H__

// #include <Windows.h>
#include <array>
#include <iostream>
#include "ros/ros.h"
#include "PID/pid.h"
#include "geometry_msgs/Twist.h"

#define M_PI	3.14159265358979323846  /* pi */

using namespace std;

class DDMR
{
public:
	PID angle_pid = PID(5, 0, 0);
	array<float, 2> robotSpeed = { 0,0 };
	ros::Publisher outputPublisher;
	geometry_msgs::Twist output_msg;
	int* clientID;
	int robotHandler, leftMotorHandler, rightMotorHandler;
	int* robotHandle = &robotHandler;
	int* leftMotorHandle = &leftMotorHandler;
	int* rightMotorHandle = &rightMotorHandler;
	float robotOrientation[3];
	float robotPosition[3];
	float maxSpeed = 5;
	float robotMass = 1;

private:
	float radToDegree(float radian);

public:
	DDMR(ros::Publisher outputPublisher);
	~DDMR();
	array<float, 2> getSpeed();
	array<float, 2> getPosition();
	float getHeading();
	void motorRun(float angularSpeed, float linearSpeed);
	void robotRun();
	void robotStop();
	void setSpeed(array<float, 2> speed);
	void addImpulse(array<float, 2> impuls);
	bool isStuck();
};

#endif