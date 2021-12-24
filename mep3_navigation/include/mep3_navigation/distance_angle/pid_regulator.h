// Copyright 2021 Memristor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MEP3_NAVIGATION__DISTANCE_ANGLE__PID_REGULATOR_H_
#define MEP3_NAVIGATION__DISTANCE_ANGLE__PID_REGULATOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  double reference;
  double feedback;
  double feedback_old;
  double error;
  double integrator;
  double integrator_min;
  double integrator_max;
  double kp;
  double ki;
  double kd;
  double clamp_min;
  double clamp_max;
  double command;

  bool angle_mode;
} pid_regulator_t;  // pid positional form

void pid_regulator_update(pid_regulator_t * reg);
void pid_regulator_reset(pid_regulator_t * reg);
void pid_regulator_set_gains(pid_regulator_t * reg, double kp, double ki, double kd);

#endif  // MEP3_NAVIGATION__DISTANCE_ANGLE__PID_REGULATOR_H_
