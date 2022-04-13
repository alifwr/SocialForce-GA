#ifndef __FUZZY_H__
#define __FUZZY_H__

#include <array>
#include <iostream>
#include <iterator>
#include <algorithm>

#define n_distanceMF 4
#define n_angleMF 3
#define n_outputMF 5
#define n_ruleBaseF n_distanceMF *n_angleMF

using namespace std;

class Fuzzy
{
public:
    enum operationMode
    {
        AND = 0,
        OR = 1,
    };
    float VH = 16, H = 12, M = 8, L = 4, VL = 2;
    float magnitudeWeighted, rangeWeighted;
    array<float, n_distanceMF> distanceMF;
    array<float, n_angleMF> angleMF;
    array<float, n_distanceMF> fuzzifiedDistance;
    array<float, n_angleMF> fuzzifiedAngle;
    array<float, n_ruleBaseF> agregationMember;
    array<float, n_ruleBaseF> magnitudeRB, rangeRB;

public:
    Fuzzy();
    ~Fuzzy();
    float triangle(float a, float b, float c, float x);
    float trapesium(float a, float b, float x, bool isLeft);
    float getWeightedMagnitude();
    float getWeightedRange();
    void fuzzification(float distance, float angle);
    void agregate();
    void setDistanceMF(array<float, n_distanceMF> mfValue);
    void setAngleMF(array<float, n_angleMF> mfValue);
    void setMagnitudeRB(array<float, n_ruleBaseF> ruleBase);
    void setRangeRB(array<float, n_ruleBaseF> ruleBase);
    void updateRuleBase(array<float, n_ruleBaseF> ruleBase);
};

#endif