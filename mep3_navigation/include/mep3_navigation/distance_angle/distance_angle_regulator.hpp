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

#ifndef MEP3_NAVIGATION__DISTANCE_ANGLE__DISTANCE_ANGLE_REGULATOR_HPP_
#define MEP3_NAVIGATION__DISTANCE_ANGLE__DISTANCE_ANGLE_REGULATOR_HPP_

#define RUN_EACH_NTH_CYCLES(counter_type, nth, run) \
  {                                                 \
    static counter_type _cycle_ = 0;                \
    if (nth > 0 && ++_cycle_ >= nth) {              \
      _cycle_ = 0;                                  \
      run;                                          \
    }                                               \
  }

#include <memory>
#include <mutex>
#include <utility>
#include <vector>

extern "C" {
#include "mep3_navigation/distance_angle/pid_regulator.h"
}

#include "geometry_msgs/msg/twist.hpp"
#include "mep3_msgs/action/motion_command.hpp"
#include "mep3_navigation/distance_angle/motion_profile.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ruckig/ruckig.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class DistanceAngleRegulator : public rclcpp::Node
{
public:
  explicit DistanceAngleRegulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  using NavigatoToPoseT = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseServer = nav2_util::SimpleActionServer<NavigatoToPoseT>;
  using MotionCommandT = mep3_msgs::action::MotionCommand;
  using MotionCommandServer = nav2_util::SimpleActionServer<MotionCommandT>;

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  pid_regulator_t regulator_distance_;
  pid_regulator_t regulator_angle_;
  double robot_x_;
  double robot_y_;
  double robot_distance_;
  double robot_angle_;
  double robot_velocity_linear_;
  double robot_velocity_angular_;
  double robot_acceleration_linear_;
  double robot_acceleration_angular_;
  double prev_robot_x_;
  double prev_robot_y_;
  double distance_goal_tolerance_;
  double angle_goal_tolerance_;
  bool position_initialized_;
  bool debug_;
  uint64_t odometry_counter_;
  bool action_running_;
  bool output_enabled_;

  ruckig::Ruckig<2> * motion_profile_;
  ruckig::InputParameter<2> motion_profile_input_;
  ruckig::OutputParameter<2> motion_profile_output_;
  ruckig::Result motion_profile_result_;

  double goal_distance_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  std::unique_ptr<NavigateToPoseServer> navigate_to_pose_server_;
  std::unique_ptr<MotionCommandServer> motion_command_server_;

  rclcpp::Time odometry_time_;

  std::mutex data_mutex_;

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  double angle_normalize(double angle);
  void forward(double distance);
  void rotate_relative(double angle);
  void rotate_absolute(double angle);
  void softstop();

  bool distance_regulator_finished();
  bool angle_regulator_finished();
  bool motion_profile_finished();
  void wait_for_odometry();  // call this without mutex lock for now

  void navigate_to_pose();
  void motion_command();
};

#endif  // MEP3_NAVIGATION__DISTANCE_ANGLE__DISTANCE_ANGLE_REGULATOR_HPP_
