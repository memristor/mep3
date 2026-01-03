#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "motor.h"
#include "pid_regulator.h"

void control_init();
void control_reset();

void control_interrupt();

void control_set_setpoint_linear(int16_t setpoint);
int16_t control_get_setpoint_linear();

void control_set_setpoint_angular(int16_t setpoint);
int16_t control_get_setpoint_angular();

/* PID parameters linear*/
void control_set_kp_linear(float kp);
float control_get_kp_linear();

void control_set_ki_linear(float ki);
float control_get_ki_linear();

void control_set_kd_linear(float kd);
float control_get_kd_linear();
/**********************/

/* PID parameters angular*/
void control_set_kp_angular(float kp);
float control_get_kp_angular();

void control_set_ki_angular(float ki);
float control_get_ki_angular();

void control_set_kd_angular(float kd);
float control_get_kd_angular();
/**********************/

#endif