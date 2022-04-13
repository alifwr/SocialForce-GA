#ifndef __MySFM_H__
#define __MySFM_H__

#include <array>
#include <iostream>
#include <cmath>

#define M_PI 3.14159265358979323846 /* pi */

using namespace std;

class SFM
{
public:
	float robotMass = 3;
	float relaxationTime = 1;
	float robotRadius = 2;
	float maxSpeed = 10;
	float staticMagnitudeForce = 1;
	float dynamicMagnitudeForce = 1;
	float staticEffectiveRange = 1;
	float dynamicEffectiveRange = 1;

private:
	array<float, 2> goalVector = {0, 0};
	array<float, 2> goalForce = {0, 0};
	array<float, 2> staticForce = {0, 0};
	array<float, 2> dynamicForce = {0, 0};

public:
	SFM();
	void updateStaticParams(float staticMagnitudeForce, float staticEffectiveRange);
	void updateDynamicParams(float dynamicMagnitudeForce, float dynamicEffectiveRange);
	void setGoalForce(array<float, 2> currentSpeed, array<float, 2> goalVector, float relaxationTime);
	void setStaticForce(array<float, 2> vector);
	void setDynamicForce(array<float, 2> vector, float objectRadius);
	float calculateDistance(array<float, 2> vector);
	float getAngleFromVector(float xVector, float yVector);
	float radToDegree(float radian);
	float degToRadian(float degree);
	float normalizeAngle(float angle);
	array<float, 2> calculateImpulse();
};

#endif