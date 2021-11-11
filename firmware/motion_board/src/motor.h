#ifndef MOTOR_H
#define MOTOR_H

#include "definitions.h"

#define MOTOR_MAX_PWM 5999

void motor_init();
void motor_turn_off();
void motor_left_set_pwm(int pwm);
void motor_right_set_pwm(int pwm);

#endif