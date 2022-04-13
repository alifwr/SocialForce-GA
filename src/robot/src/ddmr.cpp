#include "DDMR/ddmr.h"



DDMR::DDMR(ros::Publisher publisher)
{
	DDMR::outputPublisher = publisher;
	DDMR::motorRun(0, 0);
}

DDMR::~DDMR()
{
}

array<float,2> DDMR::getSpeed() {
	return DDMR::robotSpeed;
}

array<float, 2> DDMR::getPosition() {
	// simxGetObjectPosition(*DDMR::clientID, *DDMR::robotHandle, -1, DDMR::robotPosition, simx_opmode_streaming);
	return{
		DDMR::robotPosition[0],
		DDMR::robotPosition[1],
	};
}

float DDMR::radToDegree(float radian) {
	return 180 * radian / M_PI;
}

float DDMR::getHeading() {
	// simxGetObjectOrientation(*DDMR::clientID, *DDMR::robotHandle, -1, DDMR::robotOrientation, simx_opmode_buffer);
	if (robotOrientation[2] < 0)
		return 360 + DDMR::radToDegree(robotOrientation[2]);
	return DDMR::radToDegree(robotOrientation[2]);
}

void DDMR::motorRun(float angularSpeed, float linearSpeed) {
	float leftSpeed = linearSpeed - angularSpeed;
	float rightSpeed = linearSpeed + angularSpeed;
	DDMR::output_msg.angular.z = angularSpeed;
	DDMR::output_msg.linear.y = linearSpeed;
	outputPublisher.publish(output_msg);
	// simxSetJointTargetVelocity(*DDMR::clientID, *DDMR::leftMotorHandle, leftSpeed, simx_opmode_oneshot_wait);
	// simxSetJointTargetVelocity(*DDMR::clientID, *DDMR::rightMotorHandle, rightSpeed, simx_opmode_oneshot_wait);
}

void DDMR::robotRun() {
	float heading = DDMR::getHeading();
	float vectorX = DDMR::robotSpeed[0];
	float vectorY = DDMR::robotSpeed[1];
	float targetHeading;

	//Error Calculation
	if (vectorX > 0) {
		if (vectorY < 0)
			targetHeading = 360 + DDMR::radToDegree(atan(vectorY / vectorX));
		else
			targetHeading = DDMR::radToDegree(atan(vectorY / vectorX));
	}
	else if(vectorX < 0) {
		targetHeading = 180 + DDMR::radToDegree(atan(vectorY / vectorX));
	}
	else {
		if (vectorY > 0)
			targetHeading = DDMR::radToDegree(atan(vectorY / vectorX));
		else if (vectorY < 0)
			targetHeading = 360 + DDMR::radToDegree(atan(vectorY / vectorX));
		else
			targetHeading = heading;
	}
	float Error = targetHeading - heading;
	if (Error < -180)
		Error += 360;
	if (Error > 180)
		Error = Error -= 360;

	cout << "Error : " << Error << endl;
	
	float angularSpeed = angle_pid.calculateOutput(Error/180);
	float linearSpeed = sqrt(pow(vectorX, 2) + pow(vectorY, 2));

	DDMR::motorRun(angularSpeed, linearSpeed);
}

void DDMR::robotStop() {

}

void DDMR::setSpeed(array<float, 2> speed) {
	DDMR::robotSpeed = speed;
}

void DDMR::addImpulse(array<float, 2> impulse) {
	DDMR::robotSpeed[0] += impulse[0] / DDMR::robotMass;
	DDMR::robotSpeed[1] += impulse[1] / DDMR::robotMass;
	DDMR::robotSpeed[0] /= 2;
	DDMR::robotSpeed[1] /= 2;
	float delta_impulse = abs(impulse[0] - impulse[1]);
	float abs_ix = abs(impulse[0]);
	float abs_iy = abs(impulse[1]);
	float abs_vx = abs(DDMR::robotSpeed[0]);
	float abs_vy = abs(DDMR::robotSpeed[1]);

	if (abs_vx > DDMR::maxSpeed && abs_vy > DDMR::maxSpeed) {
		cout << "THIS IS " << "FULL KABEH" << endl;
		DDMR::robotSpeed[0] = DDMR::maxSpeed;
		DDMR::robotSpeed[1] = DDMR::maxSpeed;
		if (abs_ix < abs_iy) {
			DDMR::robotSpeed[0] -= delta_impulse*(abs_vx / DDMR::robotSpeed[0]);
		}
		else if (abs_ix > abs_ix) {
			DDMR::robotSpeed[1] -= delta_impulse*(abs_vy / DDMR::robotSpeed[1]);
		}
	}
	else {
		if (DDMR::robotSpeed[0] > DDMR::maxSpeed)
			DDMR::robotSpeed[0] = DDMR::maxSpeed;
		else if (DDMR::robotSpeed[0] < -DDMR::maxSpeed)
			DDMR::robotSpeed[0] = -DDMR::maxSpeed;
		if (DDMR::robotSpeed[1] > DDMR::maxSpeed)
			DDMR::robotSpeed[1] = DDMR::maxSpeed;
		else if (DDMR::robotSpeed[1] < -DDMR::maxSpeed)
			DDMR::robotSpeed[1] = -DDMR::maxSpeed;
	}
	DDMR::robotRun();
}

bool DDMR::isStuck() {
	return true;
}