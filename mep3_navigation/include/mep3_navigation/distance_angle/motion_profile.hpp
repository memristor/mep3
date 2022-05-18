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

#ifndef MEP3_NAVIGATION__DISTANCE_ANGLE__MOTION_PROFILE_HPP_
#define MEP3_NAVIGATION__DISTANCE_ANGLE__MOTION_PROFILE_HPP_

#include "rclcpp/time.hpp"

class MotionProfile
{
public:
  MotionProfile();
  MotionProfile(double position_initial, double velocity_max, double acceleration_max);
  void plan(
    double position_initial, double setpoint, double velocity_initial, double velocity_final,
    rclcpp::Time time);
  double update(rclcpp::Time time);
  double get_position();
  double get_velocity();
  double get_setpoint();
  double get_velocity_max();
  void set_velocity_max(double velocity_max);
  double get_acceleration_max();
  void set_acceleration_max(double acceleration_max);
  bool finished();
  enum class ProfileState { ACCELERATION, CRUISING, DECELERATION, FINISHED };
  ProfileState get_state();

private:
  double position_;
  double position_initial_;
  double setpoint_;
  double velocity_max_;
  double acceleration_max_;
  double velocity_initial_;
  double velocity_cruising_;
  double velocity_final_;
  double velocity_current_;
  double acceleration_;
  double deceleration_;
  double acceleration_current_;

  double t0_, t1_, t2_, t3_;
  double y1_, y2_;

  rclcpp::Time time_initial_;

  ProfileState state_;
  bool finished_;
};

#endif  // MEP3_NAVIGATION__DISTANCE_ANGLE__MOTION_PROFILE_HPP_
