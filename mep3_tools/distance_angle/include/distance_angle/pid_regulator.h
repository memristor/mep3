#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    double reference;
    double feedback;
    double feedback_old;
    double error;
    double integrator;
    double integrator_min;
    double integrator_max;
    double kp;
    double ki;
    double kd;
    double clamp_min;
    double clamp_max;
    double command;
} pid_regulator_t;        // pid positional form

void pid_regulator_update(pid_regulator_t *reg);
void pid_regulator_reset(pid_regulator_t *reg);
void pid_regulator_set_gains(pid_regulator_t *reg, double kp, double ki, double kd);



#endif