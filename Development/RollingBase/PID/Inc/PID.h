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
    int previousValue;
    int newValue;
}PID_VALUE;

#define InitializationPid(pid)\
{\
    int i;\
    \
    for(i = 0; i<3; i++)\
    {\
    pid.errorGap[i] = 0;\
    }\
    \
    pid.previousValue = 0;\
    pid.newValue = 0;\
    pid.correctorValues.kp = 0;\
    pid.correctorValues.kd = 0;\
    pid.correctorValues.ki = 0;\
}\

#define PCalculation(pid)\
{\
    pid.newValue += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]);\
}\

#define ICalculation(pid, T)\
{\
    pid.newValue += pid.correctorValues.ki*pid.errorGap[0]*T;\
}\

#define DCalculation(pid, T)\
{\
    pid.newValue += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]*2 + pid.errorGap[2]);\
}\

#define newValueCalculation(pid, T, currentValue)\
{\
    pid.errorGap[2] = 0;\
    Swap(pid.errorGap[2],pid.errorGap[1])\
    Swap(pid.errorGap[1],pid.errorGap[0])\
    Swap(pid.previousValue, pid.newValue)\
    pid.newValue = currentValue;\
    pid.errorGap[0] = pid.newValue - previousValue;\
    PCalculation(pid)\
    ICalculation(pid, T)\
    DCalculation(pid, T)\
}\

#endif