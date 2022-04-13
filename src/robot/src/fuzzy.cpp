#include "Fuzzy/fuzzy.h"

Fuzzy::Fuzzy()
{
}

Fuzzy::~Fuzzy()
{
}

float Fuzzy::triangle(float a, float b, float c, float x)
{
    if (x <= a || x >= c)
        return 0;
    if (x < b)
    {
        return (x - a) / (b - a);
    }
    else if (x > b)
    {
        return (c - x) / (c - b);
    }
    else
        return 1;
}

float Fuzzy::trapesium(float a, float b, float x, bool isLeft)
{
    if (isLeft)
    {
        if (x < a)
            return 1;
        if (x >= b)
            return 0;
        return (b - x) / (b - a);
    }
    else
    {
        if (x <= a)
            return 0;
        if (x > b)
            return 1;
        return (x - a) / (b - a);
    }
}

void Fuzzy::setDistanceMF(array<float, n_distanceMF> mfValue)
{
    for (int i = 0; i < n_distanceMF; i++)
        distanceMF[i] = mfValue[i];
}

void Fuzzy::setAngleMF(array<float, n_angleMF> mfValue)
{
    for (int i = 0; i < n_angleMF; i++)
        angleMF[i] = mfValue[i];
}

void Fuzzy::fuzzification(float distance, float angle)
{
    for (int i = 0; i < n_distanceMF; i++)
    {
        if (i > 0 && i < n_distanceMF - 1)
        {
            Fuzzy::fuzzifiedDistance[i] = Fuzzy::triangle(Fuzzy::distanceMF[i - 1], Fuzzy::distanceMF[i], Fuzzy::distanceMF[i + 1], distance);
        }
        else if (i == 0)
        {
            Fuzzy::fuzzifiedDistance[i] = Fuzzy::trapesium(Fuzzy::distanceMF[i], Fuzzy::distanceMF[i + 1], distance, true);
        }
        else
        {
            Fuzzy::fuzzifiedDistance[i] = Fuzzy::trapesium(Fuzzy::distanceMF[i - 1], Fuzzy::distanceMF[i], distance, false);
        }
    }
    for (int i = 0; i < n_angleMF; i++)
    {
        if (i > 0 && i < n_angleMF - 1)
        {
            Fuzzy::fuzzifiedAngle[i] = Fuzzy::triangle(Fuzzy::angleMF[i - 1], Fuzzy::angleMF[i], Fuzzy::angleMF[i + 1], angle);
        }
        else if (i == 0)
        {
            Fuzzy::fuzzifiedAngle[i] = Fuzzy::trapesium(Fuzzy::angleMF[i], Fuzzy::angleMF[i + 1], angle, true);
        }
        else
        {
            Fuzzy::fuzzifiedAngle[i] = Fuzzy::trapesium(Fuzzy::angleMF[i - 1], Fuzzy::angleMF[i], angle, false);
        }
    }

    // for (int i = 0; i < n_distanceMF; i++)
    // {
    //     cout << Fuzzy::fuzzifiedDistance[i] << " ";
    // }
    // cout << endl;
    // for (int i = 0; i < n_angleMF; i++)
    // {
    //     cout << Fuzzy::fuzzifiedAngle[i] << " ";
    // }
    // cout << endl;
    Fuzzy::agregate();
}

void Fuzzy::agregate()
{
    float val, weighted_average;
    float magnitudeTotals = 0;
    float magnitudeWeights = 0;
    float rangeTotals = 0;
    float rangeWeights = 0;

    for (int i = 0; i < n_angleMF; i++)
    {
        if (Fuzzy::fuzzifiedAngle[i] != 0)
        {
            for (int j = 0; j < n_distanceMF; j++)
            {
                ////////////////////
                if (Fuzzy::fuzzifiedDistance[j] != 0)
                {
                    val = min(Fuzzy::angleMF[i], Fuzzy::distanceMF[j]);
                    if (val == 0)
                        val = max(Fuzzy::angleMF[i], Fuzzy::distanceMF[j]);
                    magnitudeTotals += Fuzzy::magnitudeRB[i + (n_angleMF * j)] * val;
                    magnitudeWeights += val;
                    rangeTotals += Fuzzy::rangeRB[i + (n_angleMF * j)] * val;
                    rangeWeights + val;
                    // cout << Fuzzy::magnitudeRB[i + (n_angleMF * j)] << ":" << val << " ";
                }
            }
            // cout << endl;
        }
    }
    Fuzzy::magnitudeWeighted = magnitudeTotals / magnitudeWeights;
    Fuzzy::rangeWeighted = rangeTotals / rangeWeights;
}

float Fuzzy::getWeightedMagnitude()
{
    return Fuzzy::magnitudeWeighted;
}

float Fuzzy::getWeightedRange()
{
    return Fuzzy::rangeWeighted;
}

void Fuzzy::setMagnitudeRB(array<float, n_ruleBaseF> ruleBase)
{
    Fuzzy::magnitudeRB = ruleBase;
}

void Fuzzy::setRangeRB(array<float, n_ruleBaseF> ruleBase)
{
    Fuzzy::rangeRB = ruleBase;
}