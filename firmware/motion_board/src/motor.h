#ifndef MOTOR_H
#define MOTOR_H

#include "definitions.h"

#define MOTOR_MAX_PWM 5999

void motor_init();
void motor_turnOff();
void motor_leftSetPwm(int pwm);
void motor_rightSetPwm(int pwm);

#endif