#ifndef PID_H
#define PID_H

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
    int previousVoltage;
    int newVoltage;
}PID_VALUE;

#define InitializationPid(pid)\
{\
    int i;\
    \
    for(i = 0; i<3; i++)\
    {\
    pid.error = 0;\
    }\
    \
    pid.previousVoltage = 0;\
    pid.newVoltage = 0;\
    pid.correctorValues.kp = 0;\
    pid.correctorValues.kd = 0;\
    pid.correctorValues.ki = 0;\
}\

#define PCalculation(pid)\
{\
    pid.newVoltage += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]);\
}\

#define ICalculation(pid, T)\
{\
    pid.newVoltage += pid.correctorValues.ki*pid.errorGap[0]*T;\
}\

#define DCalculation(pid, T)\
{\
    pid.newVoltage += pid.correctorValues.kp*(pid.errorGap[0] - pid.errorGap[1]*2 + pid.errorGap[2]);\
}\

#define NewVoltageCalculation(pid, T)\
{\
    PCalculation(pid)\
    ICalculation(pid, T)\
    DCalculation(pid, T)\
}\

#endif