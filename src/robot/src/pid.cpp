#include "PID/pid.h"

PID::PID(float kp, float ki, float kd)
{
	PID::kp = kp;
	PID::ki = ki;
	PID::kd = kd;
}


PID::~PID()
{
}


float PID::calculateOutput(float Error) {
	if (!PID::isStarted) {
		PID::isStarted = true;
		PID::lastTime = chrono::high_resolution_clock::now();
	}
	PID::currentTime = chrono::high_resolution_clock::now();
	PID::sampleTime = PID::currentTime - PID::lastTime;
	PID::iError += Error*PID::sampleTime.count();
	PID::dError = (Error - PID::lastError) / sampleTime.count();
	PID::lastTime = PID::currentTime;
	return kp*Error + ki*iError + kd*dError;
}

void PID::stopPID() {
	PID::isStarted = false;
	iError = 0;
	dError = 0;
	lastError = 0;
}