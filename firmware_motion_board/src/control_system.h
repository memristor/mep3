#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <stdint.h>
#include "motor.h"
#include "pid_regulator.h"

void control_system_initRegulators();
void control_system_resetRegulators();

void control_system_interrupt();

void control_system_setSetpointLeft(float setpoint);
float control_system_getSetpointLeft();

void control_system_setSetpointRight(float setpoint);
float control_system_getSetpointRight();

/* PID parameters left*/
void control_system_setKpLeft(float kp);
float control_system_getKpLeft();

void control_system_setKiLeft(float ki);
float control_system_getKiLeft();

void control_system_setKdLeft(float kd);
float control_system_getKdLeft();
/**********************/

/* PID parameters right*/
void control_system_setKpRight(float kp);
float control_system_getKpRight();

void control_system_setKiRight(float ki);
float control_system_getKiRight();

void control_system_setKdRight(float kd);
float control_system_getKdRight();
/**********************/

#endif