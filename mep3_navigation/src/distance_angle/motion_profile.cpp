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

#include "mep3_navigation/distance_angle/motion_profile.hpp"

#include <cmath>
#include <iostream>

#include "rclcpp/duration.hpp"

MotionProfile::MotionProfile()
{
  position_ = 0;
  position_initial_ = 0;
  setpoint_ = 0;
  velocity_max_ = 0.1;
  acceleration_max_ = 0.01;

  state_ = ProfileState::FINISHED;
}

MotionProfile::MotionProfile(double position_initial, double velocity_max, double acceleration_max)
{
  position_ = position_initial;
  position_initial_ = position_initial;
  setpoint_ = position_initial;
  velocity_max_ = velocity_max;
  acceleration_max_ = acceleration_max;

  state_ = ProfileState::FINISHED;
}

void MotionProfile::plan(
  double position_initial, double setpoint, double velocity_initial, double velocity_final,
  rclcpp::Time time)
{
  state_ = ProfileState::ACCELERATION;
  time_initial_ = time;
  position_initial_ = position_initial;
  setpoint_ = setpoint;
  velocity_initial_ = velocity_initial;
  velocity_final_ = velocity_final;

  const double x_stop =
    position_initial_ +
    (std::pow(velocity_final_, 2) - std::pow(velocity_initial_, 2)) / (2.0 * acceleration_max_);
  const double s = std::signbit(setpoint_ - x_stop) ? -1.0 : 1.0;

  acceleration_ = s * acceleration_max_;
  deceleration_ = -s * acceleration_max_;
  velocity_cruising_ = s * velocity_max_;

  double delta_t1 = std::abs((velocity_cruising_ - velocity_initial_) / acceleration_);
  double delta_t3 = -velocity_cruising_ / deceleration_;
  const double delta_x1 = velocity_initial_ * delta_t1 + acceleration_ * pow(delta_t1, 2) / 2.0;
  const double delta_x3 = velocity_cruising_ * delta_t3 + deceleration_ * pow(delta_t3, 2) / 2.0;

  double delta_t2 = (setpoint_ - (position_initial_ + delta_x1 + delta_x3)) / velocity_cruising_;

  if (std::signbit(delta_t2)) {
    // trapezoidal profile not possible
    velocity_cruising_ = s * sqrt(
                               s * acceleration_max_ * (setpoint_ - position_initial_) +
                               pow(velocity_initial_, 2) / 2.0);
    delta_t2 = 0;
    delta_t1 = std::abs((velocity_cruising_ - velocity_initial_) / acceleration_);
    delta_t3 = -velocity_cruising_ / deceleration_;
  }

  t0_ = 0;
  t1_ = t0_ + delta_t1;
  t2_ = t1_ + delta_t2;
  t3_ = t2_ + delta_t3;

  y1_ =
    position_initial_ + velocity_initial_ * delta_t1 + acceleration_ * delta_t1 * delta_t1 / 2.0;
  y2_ = y1_ + velocity_cruising_ * delta_t2;
}

double MotionProfile::update(rclcpp::Time time)
{
  rclcpp::Duration delta_t = time - time_initial_;
  double t = delta_t.nanoseconds() / 1000000000.0;

  if (t <= t1_) {
    position_ = position_initial_ + velocity_initial_ * (t - t0_) +
                acceleration_ * (t - t0_) * (t - t0_) / 2.0;
    velocity_current_ = velocity_initial_ + acceleration_ * (t - t0_);
    acceleration_current_ = acceleration_;
    state_ = ProfileState::ACCELERATION;
  } else if (t <= t2_) {
    position_ = y1_ + velocity_cruising_ * (t - t1_);
    velocity_current_ = velocity_cruising_;
    acceleration_current_ = 0;
    state_ = ProfileState::CRUISING;
  } else if (t < t3_) {
    position_ = y2_ + velocity_cruising_ * (t - t2_) + deceleration_ * (t - t2_) * (t - t2_) / 2.0;
    velocity_current_ = velocity_cruising_ + deceleration_ * (t - t2_);
    acceleration_current_ = deceleration_;
    state_ = ProfileState::DECELERATION;
  } else {
    position_ = setpoint_ + velocity_final_ *
                              (t - t3_);  // Continue integrating in case of non-zero final velocity
    velocity_current_ = velocity_final_;
    acceleration_current_ = 0;
    state_ = ProfileState::FINISHED;
  }

  return position_;
}

double MotionProfile::get_position() { return position_; }

double MotionProfile::get_velocity() { return velocity_current_; }

double MotionProfile::get_setpoint() { return setpoint_; }

double MotionProfile::get_velocity_max() { return velocity_max_; }

void MotionProfile::set_velocity_max(double velocity_max) { velocity_max_ = velocity_max; }

double MotionProfile::get_acceleration_max() { return acceleration_max_; }

void MotionProfile::set_acceleration_max(double acceleration_max)
{
  acceleration_max_ = acceleration_max;
}

bool MotionProfile::finished() { return state_ == ProfileState::FINISHED; }

MotionProfile::ProfileState MotionProfile::get_state() { return state_; }
