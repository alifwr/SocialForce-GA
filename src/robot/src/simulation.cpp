#include "Simulation/simulation.h"

Simulation::Simulation()
{
    ros::Rate loop_rate(1);
    Simulation::check[0] = 0;
    Simulation::check[1] = 0;

    simxFinish(-1);

    clientID = simxStart("127.0.0.1", portNb, true, true, 5000, 5);
    loop_rate.sleep();
}

Simulation::~Simulation()
{
    simxStopSimulation(clientID, simx_opmode_oneshot_wait);
}

void Simulation::start()
{
    ros::Rate loop_rate(1);
    simxStartSimulation(clientID, simx_opmode_oneshot_wait);
    cout << "\rSTARTING SIMULATION";
    for (int i = 0; i < 20; i++)
    {
        loop_rate.sleep();
        cout << "." << flush;
    }
    cout << "\rSTARTED                                 " << endl;
}

void Simulation::stop()
{
    ros::Rate loop_rate(1);
    simxStopSimulation(clientID, simx_opmode_oneshot_wait);
    cout << "\rSTOPING SIMULATION";
    for (int i = 0; i < 10; i++)
    {
        loop_rate.sleep();
        cout << "." << flush;
    }
    cout << "\rSTOPED                                  " << endl;
}