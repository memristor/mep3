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

extern "C" {
#include <mep3_navigation/distance_angle/pid_regulator.h>
}

#include <vector>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "mep3_msgs/msg/motion_command.hpp"
#include "mep3_navigation/distance_angle/motion_profile.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class DistanceAngleRegulator : public rclcpp::Node
{
public:
  explicit DistanceAngleRegulator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  using NavigatoToPoseT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<NavigatoToPoseT>;

private:
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void command_callback(const mep3_msgs::msg::MotionCommand::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<mep3_msgs::msg::MotionCommand>::SharedPtr command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  pid_regulator_t regulator_distance_;
  pid_regulator_t regulator_angle_;
  double robot_x_;
  double robot_y_;
  double robot_distance_;
  double robot_angle_;
  double robot_velocity_linear_;
  double robot_velocity_angular_;
  double prev_robot_x_;
  double prev_robot_y_;
  bool position_initialized_;
  bool debug_;

  MotionProfile distance_profile_;
  MotionProfile angle_profile_;

  double goal_distance_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  std::unique_ptr<ActionServer> action_server_;

  std::mutex data_lock_;

  double angle_normalize(double angle);
  void forward(double distance);
  void rotate_relative(double angle);
  void rotate_absolute(double angle);

  bool distance_regulator_finished();
  bool angle_regulator_finished();

  void navigate_to_goal();
};

#endif  // MEP3_NAVIGATION__DISTANCE_ANGLE__DISTANCE_ANGLE_REGULATOR_HPP_
