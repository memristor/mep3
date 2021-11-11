#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include "motor.h"
#include "pid_regulator.h"

void control_init();
void control_reset();

void control_interrupt();

void control_set_setpoint_left(float setpoint);
float control_get_setpoint_left();

void control_set_setpoint_right(float setpoint);
float control_get_setpoint_right();

/* PID parameters left*/
void control_set_kp_left(float kp);
float control_get_kp_left();

void control_set_ki_left(float ki);
float control_get_ki_left();

void control_set_kd_left(float kd);
float control_get_kd_left();
/**********************/

/* PID parameters right*/
void control_set_kp_right(float kp);
float control_get_kp_right();

void control_set_ki_right(float ki);
float control_get_ki_right();

void control_set_kd_right(float kd);
float control_get_kd_right();
/**********************/

#endif