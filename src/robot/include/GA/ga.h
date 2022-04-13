#ifndef __GA_H__
#define __GA_H__

#include <array>
#include <iostream>
#include <math.h>

#define n_chromosome 48
#define n_population 10
#define percent_mutation 0.2

#define max_val 100

using namespace std;

class GA{
    private:
    array<array<float, n_chromosome>, n_population> population;
    array<float, n_population> fitnesses;

    public:
    GA();
    ~GA();
    array<array<float, n_chromosome>, n_population> getPopulation();
    array<float, n_population> getFitness();
    void setFitness(int index, float value);
    void printIndividual(array<float, n_chromosome> individual);
    void printPopulation();
    void calculateFitness();
    void sortPopulation();
    void eliminate();
    void crossover();
    void mutate();
    void run();
    float fitnessFunction(array<float, n_chromosome> individual);
    float getBestFitness();
    array<float, n_chromosome> getBestSolution();
};

#endif