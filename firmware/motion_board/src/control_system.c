#include "control_system.h"

volatile PidReg_t reg_left = PIDREG_DEFAULTS;
volatile PidReg_t reg_right = PIDREG_DEFAULTS;

void control_system_initRegulators()
{    
    reg_left.clamp_min = -(float)MOTOR_MAX_PWM;
    reg_left.clamp_max = +(float)MOTOR_MAX_PWM;
    
    reg_left.ref = 0;
    reg_left.out = 0;
    reg_left.integrator = 0;
    
    reg_left.kp = 120.0;
    reg_left.ki = 1.5;
    reg_left.kd = 0.0;
    
    reg_right.ref = 0;
    reg_right.out = 0;
    reg_right.integrator = 0;
    
    reg_right.kp = 120.0;
    reg_right.ki = 1.5;
    reg_right.kd = 0.0;
    
    reg_right.clamp_min = -(float)MOTOR_MAX_PWM;
    reg_right.clamp_max = +(float)MOTOR_MAX_PWM;
}

void control_system_resetRegulators()
{
    PidReg_reset(&reg_left);
    PidReg_reset(&reg_right);
}

/*Call this at fixed frequency!!!*/
void control_system_interrupt()
{
    int32_t v_left, v_right;
    v_left = -(int32_t)QEI3_VelocityGet();
    v_right = (int32_t)QEI1_VelocityGet();
    
    reg_left.fdb = (float)v_left;
    reg_right.fdb = (float)v_right;
    
    PidReg_update(&reg_left);
    PidReg_update(&reg_right);
    
    motor_leftSetPwm(reg_left.out);
    motor_rightSetPwm(reg_right.out);    
}

void control_system_setSetpointLeft(float setpoint)
{
    reg_left.ref = setpoint;
}
float control_system_getSetpointLeft()
{
    return reg_left.ref;
}

void control_system_setSetpointRight(float setpoint)
{
    reg_right.ref = setpoint;
}
float control_system_getSetpointRight()
{
    return reg_right.ref;
}

/* PID parameters left*/
void control_system_setKpLeft(float kp)
{
    reg_left.kp = kp;
}

float control_system_getKpLeft()
{
    return reg_left.kp;
}

void control_system_setKiLeft(float ki)
{
    reg_left.ki = ki;
}

float control_system_getKiLeft()
{
    return reg_left.ki;
}

void control_system_setKdLeft(float kd)
{
    reg_left.kd = kd;
}

float control_system_getKdLeft()
{
    return reg_left.kd;
}
/**********************/


/* PID parameters right*/
void control_system_setKpRight(float kp)
{
    reg_right.kp = kp;
}

float control_system_getKpRight()
{
    return reg_right.kp;
}

void control_system_setKiRight(float ki)
{
    reg_right.ki = ki;
}

float control_system_getKiRight()
{
    return reg_right.ki;
}

void control_system_setKdRight(float kd)
{
    reg_right.kd = kd;
}

float control_system_getKdRight()
{
    return reg_right.kd;
}
/**********************/