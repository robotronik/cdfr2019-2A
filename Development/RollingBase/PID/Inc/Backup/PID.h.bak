#ifndef PID_H
#define PID_H

#include "Utils.h"

typedef struct
{
    float kp;
    float kd;
    float ki;
}CORRECTOR_PARAMETERS;

typedef struct
{
    CORRECTOR_PARAMETERS correctorValues;
    int errorGap[3];
    int currentValue;
    int finalValue;
}PID_VALUE;

#define InitializationPid(pid, goal, newKp, newKi, newKd)\
{\
    int i;\
    \
    for(i = 0; i<3; i++)\
    {\
    pid.errorGap[i] = 0;\
    }\
    \
    pid.currentValue = 0;\
    pid.finalValue = goal;\
    pid.correctorValues.kp = newKp;\
    pid.correctorValues.kd = newKd;\
    pid.correctorValues.ki = newKi;\
}\

#define PCalculation(pid)\
{\
    pid.currentValue += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]);\
}\

#define ICalculation(pid, T)\
{\
    pid.currentValue += pid.correctorValues.ki*pid.errorGap[0]*T;\
}\

#define DCalculation(pid, T)\
{\
    pid.currentValue += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]*2 + pid.errorGap[2]);\
}\

#define finalValueCalculation(pid, T, currentValue)\
{\
    pid.errorGap[2] = 0;\
    Swap(pid.errorGap[2],pid.errorGap[1])\
    Swap(pid.errorGap[1],pid.errorGap[0])\
    Swap(pid.currentValue, pid.finalValue)\
    pid.currentValue = currentValue;\
    pid.erro0rGap[0] = pid.finalValue - pid.currentValue;\
    PCalculation(pid)\
    ICalculation(pid, T)\
    DCalculation(pid, T)\
}\

#endif