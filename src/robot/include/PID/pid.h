#ifndef __PID_H__
#define __PID_H__

#include "ros/ros.h"

using namespace std;

class PID
{
public:
	chrono::high_resolution_clock::time_point lastTime, currentTime;
	chrono::duration<double, milli> sampleTime;
	float kp, ki, kd;
	float iError = 0, dError = 0, lastError = 0;
	bool isStarted = false;

public:
	PID(float kp, float ki, float kd);
	~PID();
	float calculateOutput(float Error);
	void stopPID();
};

#endif