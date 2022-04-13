#ifndef __SIMULAITON_H__
#define __SIMULATION_H__

#include "ros/ros.h"
extern "C"
{
#include "extApi.h"
#include "extApiPlatform.h"
#include "extApiInternal.h"
}

using namespace std;

class Simulation
{
private:
    simxInt *check = new simxInt[1];
    int portNb = 19997;
    int clientID = -1;

public:
    Simulation();
    ~Simulation();
    void start();
    void stop();
};

#endif