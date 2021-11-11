#include "control_system.h"

static PidReg_t reg_left;
static PidReg_t reg_right;

void control_system_init()
{    
    reg_left.clamp_min = -MOTOR_MAX_PWM;
    reg_left.clamp_max = MOTOR_MAX_PWM;
    
    reg_left.reference = 0;
    reg_left.feedback = 0;
    reg_left.error = 0;
    reg_left.command = 0.0;
    reg_left.integrator = 0.0;
    
    reg_left.kp = 120.0;
    reg_left.ki = 1.5;
    reg_left.kd = 0.0;
    
    reg_right.reference = 0;
    reg_right.feedback = 0;
    reg_right.error = 0;
    reg_right.command = 0.0;
    reg_right.integrator = 0.0;
    
    reg_right.kp = 120.0;
    reg_right.ki = 1.5;
    reg_right.kd = 0.0;
    
    reg_right.clamp_min = -MOTOR_MAX_PWM;
    reg_right.clamp_max = MOTOR_MAX_PWM;
}

void control_system_reset()
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
    
    reg_left.feedback = v_left;
    reg_right.feedback = v_right;
    
    PidReg_update(&reg_left);
    PidReg_update(&reg_right);
    
    motor_leftSetPwm(reg_left.command);
    motor_rightSetPwm(reg_right.command);    
}

void control_system_set_setpoint_left(float setpoint)
{
    reg_left.reference = setpoint;
}
float control_system_get_setpoint_left()
{
    return reg_left.reference;
}

void control_system_set_setpoint_right(float setpoint)
{
    reg_right.reference = setpoint;
}
float control_system_get_setpoint_right()
{
    return reg_right.reference;
}

/* PID parameters left*/
void control_system_set_kp_left(float kp)
{
    reg_left.kp = kp;
}

float control_system_get_kp_left()
{
    return reg_left.kp;
}

void control_system_set_ki_left(float ki)
{
    reg_left.ki = ki;
}

float control_system_get_ki_left()
{
    return reg_left.ki;
}

void control_system_set_kd_left(float kd)
{
    reg_left.kd = kd;
}

float control_system_get_kd_left()
{
    return reg_left.kd;
}
/**********************/


/* PID parameters right*/
void control_system_set_kp_right(float kp)
{
    reg_right.kp = kp;
}

float control_system_get_kp_right()
{
    return reg_right.kp;
}

void control_system_set_ki_right(float ki)
{
    reg_right.ki = ki;
}

float control_system_get_ki_right()
{
    return reg_right.ki;
}

void control_system_set_kd_right(float kd)
{
    reg_right.kd = kd;
}

float control_system_get_kd_right()
{
    return reg_right.kd;
}
/**********************/