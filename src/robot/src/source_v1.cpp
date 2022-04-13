#include <array>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "SFM/sfm.h"
#include "PID/pid.h"
#include "DDMR/ddmr.h"
#include "Fuzzy/fuzzy.h"

using namespace std;

geometry_msgs::Twist robot_setpoint_msg;
geometry_msgs::Point resultant_msg;
float robotHeading;
array<float, 2> robotPos, targetPos;
array<float, 2> staticObj, dynamicObj, goalObj;
array<float, 2> impulse;
array<float, 12> magnitudeRB = {
    30, 35, 40, 
    20, 25, 30, 
    15, 20, 25, 
    15, 15, 20
    };
array<float, 12> rangeRB = {
    20, 25, 30, 
    10, 15, 20, 
    5, 10, 10, 
    5, 5, 10};
array<float, 4> distMF = {0.4, 0.8, 1.5, 3.0};
array<float, 3> anglMF = {0, 45, 90};

array<float, 2> frameRotation(float robotHeading, array<float, 2> robotPos, array<float, 2> targetPos);
void robPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
void tarPosCallback(const geometry_msgs::Point::ConstPtr &msg);
void updateStaticObj(const geometry_msgs::Point::ConstPtr &msg);
void updateDynamicObj(const geometry_msgs::Point::ConstPtr &msg);
void updateGoalObj(const geometry_msgs::Point::ConstPtr &msg);
float calcDiagonal(array<float, 2> vector);
float vecToDeg(array<float, 2> vector);

int main(int argc, char **argv)
{
    SFM sfm = SFM();
    PID pid = PID(0.1, 0, 0);
    Fuzzy fuzzy_static = Fuzzy();
    Fuzzy fuzzy_dynamic = Fuzzy();

    fuzzy_static.setDistanceMF(distMF);
    fuzzy_static.setAngleMF(anglMF);
    fuzzy_dynamic.setDistanceMF(distMF);
    fuzzy_dynamic.setAngleMF(anglMF);

    fuzzy_static.setMagnitudeRB(magnitudeRB);
    fuzzy_static.setRangeRB(rangeRB);
    fuzzy_dynamic.setMagnitudeRB(magnitudeRB);
    fuzzy_dynamic.setRangeRB(rangeRB);

    sfm.updateStaticParams(10, 2);
    sfm.updateDynamicParams(10, 2);

    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber robotPoseSubber = n.subscribe("/robot_pose", 1000, robPoseCallback);
    ros::Subscriber targetPosSubber = n.subscribe("/target_pos", 1000, tarPosCallback);
    ros::Subscriber staticObjSubber = n.subscribe("/static_object", 1000, updateStaticObj);
    ros::Subscriber DynamicObjSubber = n.subscribe("/dynamic_object", 1000, updateDynamicObj);
    ros::Subscriber goalObjSubber = n.subscribe("/goal_object", 1000, updateGoalObj);

    ros::Publisher robotPublisher = n.advertise<geometry_msgs::Twist>("/robot_set_vel", 1000);
    ros::Publisher resultantPublisher = n.advertise<geometry_msgs::Point>("/sfm_result", 1000);
    // ros::spin();
    DDMR ddmr = DDMR(robotPublisher);

    while (ros::ok())
    {
        fuzzy_static.fuzzification(calcDiagonal(staticObj), vecToDeg(staticObj));
        fuzzy_dynamic.fuzzification(calcDiagonal(dynamicObj), vecToDeg(dynamicObj));
        sfm.updateStaticParams(fuzzy_static.getWeightedMagnitude(), fuzzy_static.getWeightedRange());
        sfm.updateDynamicParams(fuzzy_dynamic.getWeightedMagnitude(), fuzzy_dynamic.getWeightedRange());

        sfm.setGoalForce({0, 0}, goalObj, 0.1);
        sfm.setStaticForce(staticObj);
        sfm.setDynamicForce(dynamicObj, 1);

        impulse = sfm.calculateImpulse();
        resultant_msg.x = impulse[0];
        resultant_msg.y = impulse[1];
        resultantPublisher.publish(resultant_msg);

        if (impulse[1] > 0)
        {
            if (impulse[1] > 10)
                robot_setpoint_msg.linear.y = 10;
            else
                robot_setpoint_msg.linear.y = impulse[1];
        }
        else
            robot_setpoint_msg.linear.y = 0;

        float angularErr = vecToDeg({impulse[0], abs(impulse[1])});
        if (angularErr == angularErr)
            robot_setpoint_msg.angular.z = pid.calculateOutput(angularErr);
        else
            robot_setpoint_msg.angular.z = 0;
        robotPublisher.publish(robot_setpoint_msg);

        // cout << "IMPULSE1 : " << impulse[0] << " " << impulse[1] << endl;
        // impulse = frameRotation(robotHeading, robotPos, impulse);

        // array<float, 2> hasil = frameRotation(robotHeading, robotPos, impulse);

        // cout << "IMPULSE2 : " << impulse[0] << " " << impulse[1] << endl;

        ros::spinOnce();
        loop_rate.sleep();
    }
}

array<float, 2> frameRotation(float robotHeading, array<float, 2> robotPos, array<float, 2> targetPos)
{
    static const double pi = 3.14159265358979323846;
    float a = robotPos[0];
    float b = robotPos[1];
    float x = targetPos[0];
    float y = targetPos[1];
    float rotation;
    if (robotHeading > 0)
    {
        rotation = robotHeading - 90;
    }
    else
    {
        rotation = (robotHeading + 360) - 90;
    }
    double rotRad = rotation * pi / 180.0;
    // cout << robotHeading << " " << rotation << endl;
    float x_ = (((x - a) * cos(rotRad)) - ((y - b) * sin(rotRad)) + a);
    float y_ = (((x - a) * sin(rotRad)) + ((y - b) * cos(rotRad)) + b);
    return {x_, y_};
}

float vecToDeg(array<float, 2> vector)
{
    float diagonal = sqrt(pow(vector[0], 2) + pow(vector[1], 2));
    float radToDeg = (180.0 / 3.141592653589793238463);
    return asin(vector[0] / diagonal) * radToDeg;
}

float calcDiagonal(array<float, 2> vector)
{
    return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

void robPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    robotPos[0] = msg->x;
    robotPos[1] = msg->y;
    robotHeading = msg->theta;
}

void tarPosCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    targetPos[0] = msg->x;
    targetPos[1] = msg->y;
}

void updateStaticObj(const geometry_msgs::Point::ConstPtr &msg)
{
    staticObj[0] = msg->x;
    staticObj[1] = msg->y;
}

void updateDynamicObj(const geometry_msgs::Point::ConstPtr &msg)
{
    dynamicObj[0] = msg->x;
    dynamicObj[1] = msg->y;
}

void updateGoalObj(const geometry_msgs::Point::ConstPtr &msg)
{
    goalObj[0] = msg->x;
    goalObj[1] = msg->y;
}