#include "SFM/sfm.h"

SFM::SFM()
{
	//   x = 20;
	robotMass = 1;
}

void SFM::updateStaticParams(float staticMagnitudeForce, float staticEffectiveRange)
{
	SFM::staticMagnitudeForce = staticMagnitudeForce;
	SFM::staticEffectiveRange = staticEffectiveRange;
}

void SFM::updateDynamicParams(float dynamicMagnitudeForce, float dynamicEffectiveRange)
{
	SFM::dynamicMagnitudeForce = dynamicMagnitudeForce;
	SFM::dynamicEffectiveRange = dynamicEffectiveRange;
}

void SFM::setGoalForce(array<float, 2> currentSpeed, array<float, 2> goalVector, float relaxationTime)
{
	// float simi = sqrt(pow(goalVector[0], 2) + pow(goalVector[1], 2));
	float distance = SFM::calculateDistance(goalVector);
	float vectorX = goalVector[0] / distance;
	float vectorY = goalVector[1] / distance;
	goalForce[0] = vectorX * robotMass * (maxSpeed - currentSpeed[0]) / relaxationTime;
	goalForce[1] = vectorY * robotMass * (maxSpeed - currentSpeed[1]) / relaxationTime;
	// cout << "GOAL FORCE : " << goalVector[0] << " " << goalVector[1] << endl;
}

void SFM::setStaticForce(array<float, 2> vector)
{
	// float simi = sqrt(pow(vector[0], 2) + pow(vector[1], 2));
	float distance = SFM::calculateDistance(vector);
	float vectorX = vector[0] / distance;
	float vectorY = vector[1] / distance;
	float totalRadius = SFM::robotRadius;
	// cout << SFM::robotRadius * (vectorY / simi) << endl;
	if (distance > totalRadius)
	{
		SFM::staticForce = {0, 0};
	}
	else
	{
		float socialForce = pow(SFM::staticMagnitudeForce, (totalRadius - distance) / SFM::staticEffectiveRange);
		float physicalForce = SFM::staticMagnitudeForce * (totalRadius - distance);
		// cout << SFM::staticEffectiveRange << endl;
		// cout << "social static : " << socialForce << endl;
		// cout << "physical static : " << physicalForce << endl;
		if (physicalForce < 0)
			physicalForce = 0;
		SFM::staticForce[0] = (socialForce + physicalForce) * vectorX;
		SFM::staticForce[1] = (socialForce + physicalForce) * vectorY;
	}
}

void SFM::setDynamicForce(array<float, 2> vector, float objectRadius)
{
	// float simi = sqrt(pow(vector[0], 2) + pow(vector[1], 2));
	float distance = SFM::calculateDistance(vector);
	float vectorX = vector[0] / distance;
	float vectorY = vector[1] / distance;
	float totalRadius = SFM::robotRadius + objectRadius;
	// cout << SFM::robotRadius * (vectorY / simi) << endl;
	if (distance > totalRadius)
	{
		SFM::dynamicForce = {0, 0};
	}
	else
	{
		float socialForce = pow(SFM::dynamicMagnitudeForce, (totalRadius - distance) / SFM::dynamicEffectiveRange);
		float physicalForce = SFM::dynamicMagnitudeForce * (totalRadius - distance);
		if (physicalForce < 0)
			physicalForce = 0;
		SFM::dynamicForce[0] = (socialForce + physicalForce) * vectorX;
		SFM::dynamicForce[1] = (socialForce + physicalForce) * vectorY;
	}
}

float SFM::calculateDistance(array<float, 2> vector)
{
	return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

array<float, 2> SFM::calculateImpulse()
{
	float xResultant = SFM::goalForce[0] - SFM::staticForce[0] - SFM::dynamicForce[0];
	float yResultant = SFM::goalForce[1] - SFM::staticForce[1] - SFM::dynamicForce[1];
	
	// cout << "GOAL : " << goalForce[0] << " " << goalForce[1] << endl;
	// cout << "STATIC : " << staticForce[0] << " " << staticForce[1] << endl;
	// cout << "DYNAMIC : " << dynamicForce[0] << " " << dynamicForce[1] << endl;
	return {
		xResultant, yResultant};
}

float SFM::getAngleFromVector(float xVector, float yVector)
{
	float angle = 0;
	if (xVector < 0)
		angle = radToDegree(atan(yVector / xVector)) + 180;
	else if (xVector > 0)
	{
		if (yVector < 0)
			angle = radToDegree(atan(yVector / xVector)) + 360;
		else
			angle = radToDegree(atan(yVector / xVector));
	}
	return angle;
}

float SFM::radToDegree(float radian)
{
	return 180 * radian / M_PI;
}

float SFM::degToRadian(float degree)
{
	return degree * M_PI / 180;
}

float SFM::normalizeAngle(float angle)
{
	while (angle < 0)
		angle += 360;
	while (angle > 360)
		angle -= 360;
	return angle;
}
