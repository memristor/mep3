#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t reference;
    volatile int32_t feedback;
    volatile int32_t error;
    volatile float integrator;
    float kp;
    float ki;
    float kd;
    float integrator_min;
    float integrator_max;
    float d_term_filtered;
    float d_term_filter_coefficient;
    int32_t clamp_min;
    int32_t clamp_max;
    volatile float command;
} pid_regulator_t;        // pid positional form

void pid_regulator_update(pid_regulator_t *reg);
void pid_regulator_reset(pid_regulator_t *reg);
void pid_regulator_set_gains(pid_regulator_t *reg, float kp, float ki, float kd);

#endif
