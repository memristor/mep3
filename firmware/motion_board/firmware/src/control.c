#include "control.h"

static pid_regulator_t reg_linear;
static pid_regulator_t reg_angular;

void control_init()
{    
    reg_linear.clamp_min = -MOTOR_MAX_PWM;
    reg_linear.clamp_max = MOTOR_MAX_PWM;
    
    reg_linear.reference = 0;
    reg_linear.feedback = 0;
    reg_linear.error = 0;
    reg_linear.command = 0.0;
    reg_linear.integrator = 0.0;
    reg_linear.integrator_max = MOTOR_MAX_PWM / 2.0;
    reg_linear.integrator_min = -MOTOR_MAX_PWM / 2.0;
    reg_linear.d_term_filtered = 0.0;
    reg_linear.d_term_filter_coefficient = 0.1;
    
    reg_linear.kp = 100.0;
    reg_linear.ki = 1.5;
    reg_linear.kd = 0.0;
    
    reg_angular.reference = 0;
    reg_angular.feedback = 0;
    reg_angular.error = 0;
    reg_angular.command = 0.0;
    reg_angular.integrator = 0.0;
    reg_angular.integrator_max = MOTOR_MAX_PWM / 2.0;
    reg_angular.integrator_min = -MOTOR_MAX_PWM / 2.0;
    reg_angular.d_term_filtered = 0.0;
    reg_angular.d_term_filter_coefficient = 0.1;
    
    reg_angular.kp = 100.0;
    reg_angular.ki = 1.5;
    reg_angular.kd = 0.0;
    
    reg_angular.clamp_min = -MOTOR_MAX_PWM;
    reg_angular.clamp_max = MOTOR_MAX_PWM;
}

void control_reset()
{
    pid_regulator_reset(&reg_linear);
    pid_regulator_reset(&reg_angular);
}

/* Call this at fixed frequency!!! 500 Hz is default */
void control_interrupt()
{
    const int32_t v_left = -(int32_t)QEI3_VelocityGet();
    const int32_t v_right = (int32_t)QEI1_VelocityGet();

    const int32_t v_linear = (int32_t)(((int64_t)v_left + (int64_t)v_right) / 2);
    const int32_t v_angular = (int32_t)((int64_t)v_right - (int64_t)v_left);
    
    reg_linear.feedback = v_linear;
    reg_angular.feedback = v_angular;
    
    pid_regulator_update(&reg_linear);
    pid_regulator_update(&reg_angular);
    
    motor_left_set_pwm(reg_linear.command - reg_angular.command);
    motor_right_set_pwm(reg_linear.command + reg_angular.command);    
}

void control_set_setpoint_linear(int16_t setpoint)
{
    reg_linear.reference = setpoint;
}
int16_t control_get_setpoint_linear()
{
    return reg_linear.reference;
}

void control_set_setpoint_angular(int16_t setpoint)
{
    reg_angular.reference = setpoint;
}
int16_t control_get_setpoint_angular()
{
    return reg_angular.reference;
}

/* PID parameters linear*/
void control_set_kp_linear(float kp)
{
    reg_linear.kp = kp;
}

float control_get_kp_linear()
{
    return reg_linear.kp;
}

void control_set_ki_linear(float ki)
{
    reg_linear.ki = ki;
}

float control_get_ki_linear()
{
    return reg_linear.ki;
}

void control_set_kd_linear(float kd)
{
    reg_linear.kd = kd;
}

float control_get_kd_linear()
{
    return reg_linear.kd;
}
/**********************/


/* PID parameters angular*/
void control_set_kp_angular(float kp)
{
    reg_angular.kp = kp;
}

float control_get_kp_angular()
{
    return reg_angular.kp;
}

void control_set_ki_angular(float ki)
{
    reg_angular.ki = ki;
}

float control_get_ki_angular()
{
    return reg_angular.ki;
}

void control_set_kd_angular(float kd)
{
    reg_angular.kd = kd;
}

float control_get_kd_angular()
{
    return reg_angular.kd;
}
/**********************/