#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <stdint.h>
#include "motor.h"
#include "pid_regulator.h"

void control_system_init();
void control_system_reset();

void control_system_interrupt();

void control_system_set_setpoint_left(float setpoint);
float control_system_get_setpoint_left();

void control_system_set_setpoint_right(float setpoint);
float control_system_get_setpoint_right();

/* PID parameters left*/
void control_system_set_kp_left(float kp);
float control_system_get_kp_left();

void control_system_set_ki_left(float ki);
float control_system_get_ki_left();

void control_system_set_kd_left(float kd);
float control_system_get_kd_left();
/**********************/

/* PID parameters right*/
void control_system_set_kp_right(float kp);
float control_system_get_kp_right();

void control_system_set_ki_right(float ki);
float control_system_get_ki_right();

void control_system_set_kd_right(float kd);
float control_system_get_kd_right();
/**********************/

#endif