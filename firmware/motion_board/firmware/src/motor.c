#include "motor.h"
#include <stdio.h>
#include <stdbool.h>

static int flip_left = 1;
static int flip_right = 1;

static int motor_active = 0;

void motor_init()
{
    if (motor_active) return;
    motor_active = 1;
    
    motor_left_set_pwm(0);
    motor_right_set_pwm(0);
    
    LEFT_INH_Set();
    RIGHT_INH_Set();
    
    static int first_time = 1;
    if (first_time)
    {
        OCMP3_Enable();
        OCMP2_Enable();
        TMR2_Start();
        first_time = 0;
    }
}

void motor_turn_off()
{
    motor_active = 0;
    LEFT_INH_Clear();
    RIGHT_INH_Clear();
    
    motor_left_set_pwm(0);
    motor_right_set_pwm(0);
}

void motor_left_set_pwm(int pwm)
{
    pwm *= flip_left;
    if (pwm > MOTOR_MAX_PWM)
        pwm = MOTOR_MAX_PWM;
    else if (pwm < -MOTOR_MAX_PWM)
        pwm = -MOTOR_MAX_PWM;
    
    if (pwm >= 0)
    {
        LEFT_INB_Clear();
    }
    else
    {
        LEFT_INB_Set();
        pwm += MOTOR_MAX_PWM;
    }    
    
    OCMP3_CompareSecondaryValueSet((uint16_t)pwm);
}

void motor_right_set_pwm(int pwm)
{
    pwm *= flip_right;
    pwm = -pwm;
    
    if (pwm > MOTOR_MAX_PWM)
        pwm = MOTOR_MAX_PWM;
    else if (pwm < -MOTOR_MAX_PWM)
        pwm = -MOTOR_MAX_PWM;
    
    if (pwm >= 0)
    {
        RIGHT_INB_Clear();
    }
    else
    {
        RIGHT_INB_Set();
        pwm += MOTOR_MAX_PWM;
    }    
    
    OCMP2_CompareSecondaryValueSet((uint16_t)pwm);
}
