#include "GA/ga.h"

GA::GA()
{
    cout << "Generate " << n_population << " first population" << endl;
    for (int i = 0; i < n_population; i++)
    {
        int gen = 0;
        cout << "n" << i << " ";
        for (int j = 0; j < n_chromosome; j++)
        {
            GA::population[i][j] = rand() % (max_val + 1);
            cout << "|" << GA::population[i][j];
            gen++;
            if (gen >= 12)
            {
                gen = 0;
                cout << "|\t";
            }
        }
        cout << endl;
    }
    cout << endl;
    GA::calculateFitness();
    GA::sortPopulation();
}

GA::~GA()
{
}

array<array<float, n_chromosome>, n_population> GA::getPopulation()
{
    return GA::population;
}

array<float, n_population> GA::getFitness()
{
    return GA::fitnesses;
}

void GA::setFitness(int index, float value)
{
    GA::fitnesses[index] = value;
}

void GA::printIndividual(array<float, n_chromosome> individual)
{
    for (int j = 0; j < n_chromosome; j++)
    {
        cout << "|" << individual[j];
    }
}

void GA::printPopulation()
{
    for (int i = 0; i < n_chromosome; i++)
        cout << "#";
    cout << endl;
    for (int i = 0; i < n_population; i++)
    {
        int gen = 0;
        cout << "n" << i << " ";
        for (int j = 0; j < n_chromosome; j++)
        {
            cout << "|" << GA::population[i][j];
            gen++;
            if (gen >= 12)
            {
                gen = 0;
                cout << "|\t";
            }
        }
        cout << "\t=\t" << GA::fitnesses[i] << endl;
    }
    for (int i = 0; i < n_chromosome; i++)
        cout << "#";
}

void GA::calculateFitness()
{
    // cout << "Calculate fitnesses" << endl;
    for (int i = 0; i < n_population; i++)
    {
        // cout << "n" << i << " ";
        // for (int j = 0; j < n_chromosome; j++)
        // {
        //     cout << "|" << GA::population[i][j];
        // }
        float fitness = fitnessFunction(GA::population[i]);
        GA::fitnesses[i] = fitness;
        // cout << "\t\t=\t" << fitness << endl;
    }
}

void GA::sortPopulation()
{
    // cout << "SORTING" << endl;
    array<float, n_chromosome> individual;
    float fitness;
    int isSorted = 0;

    while (1)
    {
        isSorted = 1;
        for (int i = 1; i < n_population; i++)
        {
            if (GA::fitnesses[i - 1] < GA::fitnesses[i])
            {
                isSorted = 0;
                fitness = GA::fitnesses[i];
                GA::fitnesses[i] = GA::fitnesses[i - 1];
                GA::fitnesses[i - 1] = fitness;
                individual = GA::population[i];
                GA::population[i] = GA::population[i - 1];
                GA::population[i - 1] = individual;
            }
        }
        if (isSorted)
            break;
    }
}

void GA::eliminate()
{
    // GA::printPopulation();
    // cout << "ELMINATING" << endl;
    for (int i = n_population - 2; i < n_population; i++)
    {
        GA::fitnesses[i] = 0;
        for (int j = 0; j < n_chromosome; j++)
        {
            GA::population[i][j] = 0;
        }
    }
}

void GA::crossover()
{
    int idx_parent1 = 0;
    int idx_parent2 = (rand() % (n_population - 3)) + 1;
    // cout << "CROSSOVERING" << endl;
    for (int i = 0; i < n_chromosome; i++)
    {
        int isSwitched = rand() % 2;
        if (isSwitched)
        {
            GA::population[n_population - 1][i] = GA::population[idx_parent2][i];
            GA::population[n_population - 2][i] = GA::population[idx_parent1][i];
        }
        else
        {
            GA::population[n_population - 1][i] = GA::population[idx_parent1][i];
            GA::population[n_population - 2][i] = GA::population[idx_parent2][i];
        }
    }
}

void GA::mutate()
{
    // cout << "MUTATING" << endl;
    for (int i = 1; i < n_population; i++)
    {
        int isMutated = rand() % 2;
        if (isMutated)
        {
            int idx_gen = (rand() % (n_chromosome));
            GA::population[i][idx_gen] = rand() % (max_val + 1);
        }
    }
}

void GA::run()
{
    // GA::printPopulation();
    // GA::eliminate();
    // GA::printPopulation();
    GA::crossover();
    // GA::printPopulation();
    GA::mutate();
    GA::calculateFitness();
    GA::sortPopulation();
    GA::printPopulation();
}

float GA::fitnessFunction(array<float, n_chromosome> individual)
{
    if (individual[2] != 0)
        return sin(individual[0]) * cos(individual[1]) / cos(individual[2]);
    else
        return 0;
}

float GA::getBestFitness()
{
    return GA::fitnesses[0];
}

array<float, n_chromosome> GA::getBestSolution()
{
    return GA::population[0];
}