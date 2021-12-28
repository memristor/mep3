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

#include "mep3_navigation/distance_angle/distance_angle_regulator.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "tf2/utils.h"

DistanceAngleRegulator::DistanceAngleRegulator(const rclcpp::NodeOptions & options)
: Node("distance_angle_regulator", options)
{
  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 100, std::bind(&DistanceAngleRegulator::odometry_callback, this, _1));
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  command_subscription_ = this->create_subscription<mep3_msgs::msg::MotionCommand>(
    "/da_command", 100, std::bind(&DistanceAngleRegulator::command_callback, this, _1));

  this->declare_parameter("kp_distance", 10.0);
  this->declare_parameter("ki_distance", 0.0);
  this->declare_parameter("kd_distance", 0.0);  // na pravom robotu 5.0

  this->declare_parameter("kp_angle", 8.0);
  this->declare_parameter("ki_angle", 0.0);
  this->declare_parameter("kd_angle", 0.0);  // na pravom robotu 3.5

  debug_ = this->declare_parameter("debug", false);
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DistanceAngleRegulator::parameters_callback, this, std::placeholders::_1));

  this->get_parameter("kp_distance", regulator_distance_.kp);
  this->get_parameter("ki_distance", regulator_distance_.ki);
  this->get_parameter("kd_distance", regulator_distance_.kd);
  regulator_distance_.clamp_min = -0.6;
  regulator_distance_.clamp_max = 0.6;
  regulator_distance_.integrator_min = -0.08;
  regulator_distance_.integrator_max = 0.08;
  regulator_distance_.angle_mode = false;

  regulator_distance_.clamp_min = -0.25;
  regulator_distance_.clamp_max = 0.25;
  regulator_distance_.integrator_min = -0.04;
  regulator_distance_.integrator_max = 0.04;

  this->get_parameter("kp_angle", regulator_angle_.kp);
  this->get_parameter("ki_angle", regulator_angle_.ki);
  this->get_parameter("kd_angle", regulator_angle_.kd);
  regulator_angle_.clamp_min = -5.0;
  regulator_angle_.clamp_max = 5.0;
  regulator_angle_.integrator_min = -1.0;
  regulator_angle_.integrator_max = 1.0;
  regulator_angle_.angle_mode = true;

  regulator_angle_.clamp_min = -6.28;
  regulator_angle_.clamp_max = 6.28;

  robot_distance_ = 0;
  position_initialized_ = false;

  distance_profile_ = MotionProfile(0, 0.55, 0.5);
  angle_profile_ = MotionProfile(0, 2.5, 2.0);

  distance_profile_ = MotionProfile(0, 0.1, 0.02);
  angle_profile_ = MotionProfile(0, 0.5, 0.1);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "distance_angle_goal",
    std::bind(&DistanceAngleRegulator::navigate_to_goal, this));

  action_server_->activate();
}

void DistanceAngleRegulator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time time = this->get_clock()->now();

  std::unique_lock<std::mutex> lock(data_lock_);

  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_velocity_linear_ = msg->twist.twist.linear.x;
  robot_velocity_angular_ = msg->twist.twist.angular.z;

  if (!position_initialized_) {
    prev_robot_x_ = robot_x_;
    prev_robot_y_ = robot_y_;
    position_initialized_ = true;

    robot_angle_ = tf2::getYaw(msg->pose.pose.orientation);

    distance_profile_.plan(0, 0, 0, 0, time);
    angle_profile_.plan(robot_angle_, robot_angle_, 0, 0, time);
  }

  robot_angle_ = tf2::getYaw(msg->pose.pose.orientation);

  const double delta_x = robot_x_ - prev_robot_x_;
  const double delta_y = robot_y_ - prev_robot_y_;
  const double distance_increment = std::hypot(delta_x, delta_y);
  const double distance_increment_angle = std::atan2(delta_y, delta_x);

  int sign = 1;
  if (std::abs(robot_angle_ - distance_increment_angle) > 0.1) sign = -1;

  robot_distance_ += sign * distance_increment;

  regulator_distance_.feedback = robot_distance_;
  regulator_angle_.feedback = robot_angle_;

  regulator_distance_.reference = distance_profile_.update(time);
  // We need to wrap the angle from -PI to PI
  const double angle_ref = angle_normalize(angle_profile_.update(time));
  regulator_angle_.reference = angle_ref;

  pid_regulator_update(&regulator_distance_);
  pid_regulator_update(&regulator_angle_);

  if (debug_) {
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Robot distance: %lf", robot_distance_);
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Regulator distance reference: %lf",
      regulator_distance_.reference);
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Regulator distance error: %lf\n",
      regulator_distance_.error);

    RCLCPP_INFO(rclcpp::get_logger("distance_angle_regulator"), "Robot angle: %lf", robot_angle_);
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Robot angle deg: %lf",
      robot_angle_ * 180.0 / M_PI);
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Robot angle reference deg: %lf",
      regulator_angle_.reference * 180.0 / M_PI);
    RCLCPP_INFO(
      rclcpp::get_logger("distance_angle_regulator"), "Regulator angle error deg: %lf\n",
      regulator_angle_.error * 180.0 / M_PI);
  }

  geometry_msgs::msg::Twist motor_command;
  motor_command.linear.x = regulator_distance_.command;
  motor_command.angular.z = regulator_angle_.command;
  twist_publisher_->publish(motor_command);

  prev_robot_x_ = robot_x_;
  prev_robot_y_ = robot_y_;

  lock.unlock();
}

rcl_interfaces::msg::SetParametersResult DistanceAngleRegulator::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "kp_distance") {
      regulator_distance_.kp = param.as_double();
    } else if (param.get_name() == "ki_distance") {
      regulator_distance_.ki = param.as_double();
    } else if (param.get_name() == "kd_distance") {
      regulator_distance_.kd = param.as_double();
    } else if (param.get_name() == "kp_angle") {
      regulator_angle_.kp = param.as_double();
    } else if (param.get_name() == "ki_angle") {
      regulator_angle_.ki = param.as_double();
    } else if (param.get_name() == "kd_angle") {
      regulator_angle_.kd = param.as_double();
    } else if (param.get_name() == "debug") {
      debug_ = param.as_bool();
    }
  }

  return result;
}

void DistanceAngleRegulator::command_callback(const mep3_msgs::msg::MotionCommand::SharedPtr msg)
{
  if (msg->command == "forward") {
    forward(msg->value);
  } else if (msg->command == "rotate_absolute") {
    rotate_absolute(msg->value);
  } else if (msg->command == "rotate_relative") {
    rotate_relative(msg->value);
  }
}

double DistanceAngleRegulator::angle_normalize(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}
void DistanceAngleRegulator::forward(double distance)
{
  rclcpp::Time time = this->get_clock()->now();
  distance_profile_.plan(
    robot_distance_, robot_distance_ + distance, robot_velocity_linear_, 0, time);
}

void DistanceAngleRegulator::rotate_absolute(double angle)
{
  rotate_relative(angle - robot_angle_);
}

void DistanceAngleRegulator::rotate_relative(double angle)
{
  rclcpp::Time time = this->get_clock()->now();
  angle_profile_.plan(
    angle_profile_.get_position(), angle_profile_.get_position() + angle,
    angle_profile_.get_velocity(), 0, time);

  angle_profile_.plan(robot_angle_, robot_angle_ + angle, robot_velocity_angular_, 0, time);
}

bool DistanceAngleRegulator::distance_regulator_finished()
{
  if (!distance_profile_.finished()) return false;
  return regulator_distance_.error < 1.2;
}

bool DistanceAngleRegulator::angle_regulator_finished()
{
  if (!angle_profile_.finished()) return false;
  return regulator_angle_.error < 0.015;
}

void DistanceAngleRegulator::navigate_to_goal()
{
  auto result = std::make_shared<NavigatoToPoseT::Result>();
  auto goal = action_server_->get_current_goal();

  const double goal_x = goal->pose.pose.position.x;
  const double goal_y = goal->pose.pose.position.y;

  enum class MotionState { START, ROTATING_TO_GOAL, MOVING_TO_GOAL, ROTATING_IN_GOAL, FINISHED };
  MotionState state = MotionState::START;

  std::unique_lock<std::mutex> lock(data_lock_);

  double delta_x = goal_x - robot_x_;
  double delta_y = goal_y - robot_y_;
  lock.unlock();
  double distance_to_goal = std::hypot(delta_x, delta_y);
  double angle_to_goal = std::atan2(delta_y, delta_x);

  rclcpp::WallRate r(50.0);

  while (rclcpp::ok()) {
    if (action_server_->is_cancel_requested()) {
      action_server_->terminate_all();
      return;
    }
    lock.lock();
    switch (state) {
      case MotionState::START:
        rotate_absolute(angle_to_goal);
        state = MotionState::ROTATING_TO_GOAL;
        break;

      case MotionState::ROTATING_TO_GOAL:
        if (angle_regulator_finished()) {
          delta_x = goal_x - robot_x_;
          delta_y = goal_y - robot_y_;
          distance_to_goal = std::hypot(delta_x, delta_y);
          forward(distance_to_goal);
          state = MotionState::MOVING_TO_GOAL;
        }
        break;

      case MotionState::MOVING_TO_GOAL:
        RUN_EACH_NTH_CYCLES(uint8_t, 5, {
          delta_x = goal_x - robot_x_;
          delta_y = goal_y - robot_y_;
          distance_to_goal = std::hypot(delta_x, delta_y);
          angle_to_goal = std::atan2(delta_y, delta_x);
          rotate_absolute(angle_to_goal);  // refresh angle reference
        })

        if (distance_regulator_finished()) {
          const double yaw = tf2::getYaw(goal->pose.pose.orientation);
          rotate_absolute(yaw);
          state = MotionState::ROTATING_IN_GOAL;
        }
        break;

      case MotionState::ROTATING_IN_GOAL:
        if (angle_regulator_finished()) {
          state = MotionState::FINISHED;
        }
        break;

      case MotionState::FINISHED:
        action_server_->succeeded_current(result);
        return;
        break;

      default:
        break;
    }
    lock.unlock();
    r.sleep();
  }
  action_server_->terminate_current(result);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceAngleRegulator>());
  rclcpp::shutdown();

  return 0;
}
