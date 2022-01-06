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

#include "mep3_navigation/distance_angle/pid_regulator.h"

#include <math.h>

void pid_regulator_update(pid_regulator_t * reg)
{
  if (reg->angle_mode) {
    while (reg->reference > M_PI) {
      reg->reference -= 2.0 * M_PI;
    }

    while (reg->reference < -M_PI) {
      reg->reference += 2.0 * M_PI;
    }
  }
  // P, I, D, terms discretized with Backward Euler (s = (z - 1) / zT)
  reg->error = reg->reference - reg->feedback;  // error = reference - feedback

  if (reg->angle_mode) {
    while (reg->error > M_PI) {
      reg->error -= 2.0 * M_PI;
    }

    while (reg->error < -M_PI) {
      reg->error += 2.0 * M_PI;
    }
  }
  const double p_term = reg->kp * reg->error;                      // proportional action
  double new_integrator = reg->integrator + reg->ki * reg->error;  // integrator
  double feedback_difference = reg->feedback - reg->feedback_old;

  if (reg->angle_mode) {
    while (feedback_difference > M_PI) {
      feedback_difference -= 2.0 * M_PI;
    }

    while (feedback_difference < -M_PI) {
      feedback_difference += 2.0 * M_PI;
    }
  }
  const double d_term_raw =
    -reg->kd * feedback_difference;  // differential action in local feedback
  reg->d_term_filtered =
    reg->d_term_filter_coefficient * reg->d_term_filtered +
    (1 - reg->d_term_filter_coefficient) * d_term_raw;  // apply first order low pass filter
  bool integrator_ok = 1;                               // flag

  if (new_integrator > reg->integrator_max)
    new_integrator = reg->integrator_max;
  else if (new_integrator < reg->integrator_min)
    new_integrator = reg->integrator_min;

  double u = p_term + new_integrator + reg->d_term_filtered;

  /* ANTI WINDUP and output clamping*/
  if (u > reg->clamp_max) {
    u = reg->clamp_max;
    if (reg->error > 0) {
      integrator_ok = 0;  // integrator windup, stop it
    }
  } else if (u < reg->clamp_min) {
    u = reg->clamp_min;
    if (reg->error < 0) {
      integrator_ok = 0;  // integrator windup, stop it
    }
  }

  if (integrator_ok) {
    reg->integrator = new_integrator;  // no windup, continue
  }
  /********************************/

  reg->command = u;  // set output

  reg->feedback_old = reg->feedback;
}

void pid_regulator_reset(pid_regulator_t * reg)
{
  reg->reference = reg->feedback;
  reg->feedback_old = reg->feedback;
  reg->error = 0;
  reg->command = 0;
  reg->kp = 0.0;
  reg->ki = 0.0;
  reg->kd = 0.0;
  reg->integrator = 0;
}

void pid_regulator_set_gains(pid_regulator_t * reg, double kp, double ki, double kd)
{
  reg->kp = kp;
  reg->ki = ki;
  reg->kd = kd;
}
