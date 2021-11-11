#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

typedef struct
{
    int32_t reference;
    int32_t feedback;
    int32_t error;
    float integrator;
    float kp;
    float ki;
    float kd;
    int32_t clamp_min;
    int32_t clamp_max;
    float command;
} PidReg_t;        // pid positional form

void PidReg_update(PidReg_t *reg);
void PidReg_reset(PidReg_t *reg);
void PidReg_set_gains(PidReg_t *reg, float kp, float ki, float kd);



#endif