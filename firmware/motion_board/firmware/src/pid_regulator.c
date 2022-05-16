#include "pid_regulator.h"

void pid_regulator_update(pid_regulator_t* reg)
{   
    // P, I, D, terms discretized with Backward Euler (s = (z - 1) / zT)
    const int32_t error_old = reg->error;
    reg->error = reg->reference - reg->feedback;     // error = reference - feedback
    const float p_term = reg->kp * reg->error;  // proportional action
    float new_integrator = reg->integrator + reg->ki * reg->error;    // integrator
    const float d_term_raw = reg->kd * (reg->error - error_old);
    reg->d_term_filtered =
        reg->d_term_filter_coefficient * reg->d_term_filtered +
        (1 - reg->d_term_filter_coefficient) * d_term_raw;  // apply first order low pass filter
    bool integrator_ok = 1;      // flag
    float u = p_term + new_integrator + reg->d_term_filtered;
    
    /* ANTI WINDUP and output clamping*/
    
    if (new_integrator > reg->integrator_max)
    {
        new_integrator = reg->integrator_max;
    } 
    else if (new_integrator < reg->integrator_min) 
    {
        new_integrator = reg->integrator_min;
    }
    
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
       
}

void pid_regulator_reset(pid_regulator_t *reg)
{
    reg->reference = reg->feedback;
    reg->error = 0;
    reg->command = 0;
    reg->kp = 0.0;
    reg->ki = 0.0;
    reg->kd = 0.0;
    reg->integrator = 0;
    reg->d_term_filtered = 0.0;
}

void pid_regulator_set_gains(pid_regulator_t* reg, float kp, float ki, float kd)
{
    reg->kp = kp;
    reg->ki = ki;
    reg->kd = kd;
}