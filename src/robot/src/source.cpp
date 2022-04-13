#include <array>
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "SFM/sfm.h"
#include "PID/pid.h"
#include "DDMR/ddmr.h"
#include "GA/ga.h"
#include "Fuzzy/fuzzy.h"
#include "Simulation/simulation.h"

using namespace std;

array<float, 2> frameRotation(float robotHeading, array<float, 2> robotPos, array<float, 2> targetPos);
void robPoseCallback(const geometry_msgs::Pose2D::ConstPtr &msg);
void tarPosCallback(const geometry_msgs::Point::ConstPtr &msg);
void updateStaticObj(const geometry_msgs::Point::ConstPtr &msg);
void updateDynamicObj(const geometry_msgs::Point::ConstPtr &msg);
void updateGoalObj(const geometry_msgs::Point::ConstPtr &msg);
float calcDiagonal(array<float, 2> vector);
float vecToDeg(array<float, 2> vector);
float simulateOnce(ros::Rate loop_rate, SFM sfm, Fuzzy fuzzy_static, Fuzzy fuzzy_dynamic, PID pid, ros::Publisher resultantPublisher, ros::Publisher robotPublisher);

geometry_msgs::Twist robot_setpoint_msg;
geometry_msgs::Point resultant_msg;
float robotHeading;
int iteration = 5;
array<float, 2> robotPos, targetPos;
array<float, 2> staticObj, dynamicObj, goalObj;
array<float, 2> impulse;
array<float, 12> staticMagnitudeRB = {
    30, 35, 40,
    20, 25, 30,
    15, 20, 25,
    15, 15, 20};
array<float, 12> staticRangeRB = {
    15, 20, 30,
    10, 15, 15,
    5, 10, 10,
    5, 5, 10};
array<float, 12> dynamicMagnitudeRB = {
    30, 35, 40,
    20, 25, 30,
    15, 20, 25,
    15, 15, 20};
array<float, 12> dynamicRangeRB = {
    15, 20, 30,
    10, 15, 15,
    5, 10, 10,
    5, 5, 10};
array<float, 4> distMF = {0.4, 0.8, 1.5, 3.0};
array<float, 3> anglMF = {0, 45, 90};

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

    fuzzy_static.setMagnitudeRB(staticMagnitudeRB);
    fuzzy_static.setRangeRB(staticRangeRB);
    fuzzy_dynamic.setMagnitudeRB(dynamicMagnitudeRB);
    fuzzy_dynamic.setRangeRB(dynamicRangeRB);

    sfm.updateStaticParams(10, 2);
    sfm.updateDynamicParams(10, 2);

    ros::init(argc, argv, "robot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    Simulation sim = Simulation();

    ros::Subscriber robotPoseSubber = n.subscribe("/robot_pose", 1000, robPoseCallback);
    ros::Subscriber targetPosSubber = n.subscribe("/target_pos", 1000, tarPosCallback);
    ros::Subscriber staticObjSubber = n.subscribe("/static_object", 1000, updateStaticObj);
    ros::Subscriber DynamicObjSubber = n.subscribe("/dynamic_object", 1000, updateDynamicObj);
    ros::Subscriber goalObjSubber = n.subscribe("/goal_object", 1000, updateGoalObj);

    ros::Publisher robotPublisher = n.advertise<geometry_msgs::Twist>("/robot_set_vel", 1000);
    ros::Publisher resultantPublisher = n.advertise<geometry_msgs::Point>("/sfm_result", 1000);

    GA ga = GA();

    // for(int i=0;i<10;i++){
    //     ga.run();
    //     // cout << ga.getBestFitness() << endl;
    // }

    for (int iter = 0; iter < iteration; iter++)
    {
        cout << "GA Iteration " << iter << " of " << iteration << endl;
        // ga.eliminate();
        ga.crossover();
        ga.mutate();

        for (int i = 0; i < n_population; i++)
        {
            cout << "***| Simulating n" << i << " |***" << endl;
            array<array<float, 48>, n_population> solutions = ga.getPopulation();
            copy(begin(solutions[i]), end(solutions[i]) - 12, begin(staticMagnitudeRB));
            copy(begin(solutions[i]) + 12, end(solutions[i]) + 24, begin(staticRangeRB));
            copy(begin(solutions[i]) + 24, end(solutions[i]) + 36, begin(dynamicMagnitudeRB));
            copy(begin(solutions[i]) + 36, end(solutions[i]) + 48, begin(dynamicRangeRB));

            fuzzy_static.setMagnitudeRB(staticMagnitudeRB);
            fuzzy_static.setRangeRB(staticRangeRB);
            fuzzy_dynamic.setMagnitudeRB(dynamicMagnitudeRB);
            fuzzy_dynamic.setRangeRB(dynamicRangeRB);

            sim.start();
            float score = simulateOnce(loop_rate, sfm, fuzzy_static, fuzzy_dynamic, pid, resultantPublisher, robotPublisher);
            cout << ">> Score = " << score << endl;
            sim.stop();

            ga.setFitness(i, score);
        }

        ga.sortPopulation();
        ga.printPopulation();
    }
}

float simulateOnce(ros::Rate loop_rate, SFM sfm, Fuzzy fuzzy_static, Fuzzy fuzzy_dynamic, PID pid, ros::Publisher resultantPublisher, ros::Publisher robotPublisher)
{
    int isGoal = 0;
    unsigned int counter = 0;
    unsigned int second = 0;
    unsigned int total = 0;
    unsigned int total_pow = 0;
    unsigned int n = 0;
    double meanErr = 0;
    double rmsErr = 0;
    double distToGoal = 0;
    goalObj[0] = 0;
    goalObj[1] = 0;

    while (ros::ok())
    {
        fuzzy_static.fuzzification(calcDiagonal(staticObj), vecToDeg(staticObj));
        fuzzy_dynamic.fuzzification(calcDiagonal(dynamicObj), vecToDeg(dynamicObj));
        sfm.updateStaticParams(fuzzy_static.getWeightedMagnitude(), fuzzy_static.getWeightedRange());
        sfm.updateDynamicParams(fuzzy_dynamic.getWeightedMagnitude(), fuzzy_dynamic.getWeightedRange());
        // cout << "Static : " << fuzzy_static.getWeightedMagnitude() << "  ";
        // cout << "Dynamic : " << fuzzy_dynamic.getWeightedMagnitude() << endl;

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

        total = total + abs(angularErr);
        total_pow = total_pow + pow(angularErr, 2);
        n++;
        counter += 1;
        second = (double)counter / 10;
        // cout << counter << endl;
        distToGoal = calcDiagonal(goalObj);
        // cout << "DIST TO GOAL : " << distToGoal << endl;
        // cout << total << " " << total_pow << " " << n << endl;
        if (distToGoal < 0.4 && distToGoal != 0)
        {
            isGoal = 1;
            break;
        }
        if (second > 60)
        {
            isGoal = 0;
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    robot_setpoint_msg.linear.y = 0;
    robot_setpoint_msg.angular.z = 0;
    robotPublisher.publish(robot_setpoint_msg);

    meanErr = (double)total / n;
    rmsErr = sqrt((double)total_pow / n);
    // cout << meanErr << " " << rmsErr << " " << n << endl;

    if (isGoal)
    {
        return 1000 - meanErr - rmsErr;
    }
    else
    {
        return 0 - meanErr - rmsErr;
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