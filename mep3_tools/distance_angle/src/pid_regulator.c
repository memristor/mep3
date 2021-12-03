#include "distance_angle/pid_regulator.h"

void pid_regulator_update(pid_regulator_t* reg)
{   
    // P, I, D, terms discretized with Backward Euler (s = (z - 1) / zT)
    reg->error = reg->reference - reg->feedback;     // error = reference - feedback
    const double p_term = reg->kp * reg->error;  // proportional action
    double new_integrator = reg->integrator + reg->ki * reg->error;    // integrator
    const double d_term = -reg->kd * (reg->feedback - reg->feedback_old);    // differential action in local feedback

    bool integrator_ok = 1;      // flag

    if (new_integrator > reg->integrator_max)
        new_integrator = reg->integrator_max;
    else if (new_integrator < reg->integrator_min)
        new_integrator = reg->integrator_min;

    double u = p_term + new_integrator + d_term;
    
    /* ANTI WINDUP and output clamping*/
    if (u > reg->clamp_max)
    {
        u = reg->clamp_max;
        if (reg->error > 0)
        {
            integrator_ok = 0;  // integrator windup, stop it
        }
    }
    else if (u < reg->clamp_min)
    {
        u = reg->clamp_min;
        if (reg->error < 0)
        {
            integrator_ok = 0;  // integrator windup, stop it
        }
    }
    
    if (integrator_ok)
    {
        reg->integrator = new_integrator;   // no windup, continue
    }
    /********************************/
    
    reg->command = u;                           // set output

    reg->feedback_old = reg->feedback;
       
}

void pid_regulator_reset(pid_regulator_t *reg)
{
    reg->reference = reg->feedback;
    reg->feedback_old = reg->feedback;
    reg->error = 0;
    reg->command = 0;
    reg->kp = 0.0;
    reg->ki = 0.0;
    reg->kd = 0.0;
    reg->integrator = 0;
}

void pid_regulator_set_gains(pid_regulator_t* reg, double kp, double ki, double kd)
{
    reg->kp = kp;
    reg->ki = ki;
    reg->kd = kd;
}