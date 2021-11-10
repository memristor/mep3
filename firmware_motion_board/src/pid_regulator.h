#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#define PIDREG_DEFAULTS {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

typedef struct
{
    float ref;
    float fdb;
    float err;
    float integrator;
    float kp;
    float ki;
    float kd;
    float clamp_min;
    float clamp_max;
    float out;
} PidReg_t;        // pid positional form

void PidReg_update(PidReg_t *reg);
void PidReg_reset(PidReg_t *reg);
void PidReg_setGains(PidReg_t *reg, float kp, float ki, float kd);



#endif