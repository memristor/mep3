#include "pid_regulator.h"

void PidReg_update(PidReg_t* reg)
{   
    // P, I, D, terms discretized with Backward Euler (s = (z - 1) / zT)
    const float err_old = reg->err;
    reg->err = reg->ref - reg->fdb;     // error = reference - feedback
    float p_term = reg->kp * reg->err;  // proportional action
    float new_integrator = reg->integrator + reg->ki * reg->err;    // integrator
    float d_term = reg->kd * (reg->err - err_old);
    int integrator_ok = 1;      // flag
    float u = p_term + new_integrator + d_term;
    
    /* ANTI WINDUP and output clamping*/
    if (u > reg->clamp_max)
    {
        u = reg->clamp_max;
        if (reg->err > 0)
        {
            integrator_ok = 0;  // integrator windup, stop it
        }
    }
    else if (u < reg->clamp_min)
    {
        u = reg->clamp_min;
        if (reg->err < 0)
        {
            integrator_ok = 0;  // integrator windup, stop it
        }
    }
    
    if (integrator_ok)
    {
        reg->integrator = new_integrator;   // no windup, continue
    }
    /********************************/
    
    reg->out = u;                           // set output
       
}

void PidReg_reset(PidReg_t *reg)
{
    reg->out = 0;
    reg->kp = 0;
    reg->ki = 0;
    reg->kd = 0;
    reg->integrator = 0;
}

void PidReg_set_gains(PidReg_t* reg, float kp, float ki, float kd)
{
    reg->kp = kp;
    reg->ki = ki;
    reg->kd = kd;
}