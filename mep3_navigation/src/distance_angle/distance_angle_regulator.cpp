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
#include <utility>
#include <vector>

#include "tf2/utils.h"

using std::placeholders::_1;
using std::placeholders::_2;

DistanceAngleRegulator::DistanceAngleRegulator(const rclcpp::NodeOptions & options)
: Node("distance_angle_regulator", options)
{
  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&DistanceAngleRegulator::odometry_callback, this, _1));
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  /*            PARAMETER DECLARATION           */
  this->declare_parameter("kp_distance", 10.0);
  this->declare_parameter("ki_distance", 0.0);
  this->declare_parameter("kd_distance", 0.0);  // na pravom robotu 5.0
  this->declare_parameter("d_term_filter_distance", 0.1);

  this->declare_parameter("kp_angle", 8.0);
  this->declare_parameter("ki_angle", 0.0);
  this->declare_parameter("kd_angle", 0.0);  // na pravom robotu 3.5
  this->declare_parameter("d_term_filter_angle", 0.1);

  this->declare_parameter("distance_goal_tolerance", 0.0015);
  this->declare_parameter("angle_goal_tolerance", 0.0035);

  this->declare_parameter("control_frequency", 60.0);

  debug_ = this->declare_parameter("debug", false);
  parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DistanceAngleRegulator::parameters_callback, this, std::placeholders::_1));
  /**********************************************/

  /*           DISTANCE REGULATOR PARAMETERS           */
  this->get_parameter("kp_distance", regulator_distance_.kp);
  this->get_parameter("ki_distance", regulator_distance_.ki);
  this->get_parameter("kd_distance", regulator_distance_.kd);
  regulator_distance_.clamp_min = -2.0;
  regulator_distance_.clamp_max = 2.0;
  regulator_distance_.integrator_min = -0.08;
  regulator_distance_.integrator_max = 0.08;
  this->get_parameter("d_term_filter_distance", regulator_distance_.d_term_filter_coefficient);
  regulator_distance_.angle_mode = false;

  this->get_parameter("distance_goal_tolerance", distance_goal_tolerance_);
  robot_velocity_linear_ = 0.0;
  /*****************************************************/

  /*           ANGLE REGULATOR PARAMETERS              */
  this->get_parameter("kp_angle", regulator_angle_.kp);
  this->get_parameter("ki_angle", regulator_angle_.ki);
  this->get_parameter("kd_angle", regulator_angle_.kd);
  regulator_angle_.clamp_min = -6.0;
  regulator_angle_.clamp_max = 6.0;
  regulator_angle_.integrator_min = -1.0;
  regulator_angle_.integrator_max = 1.0;
  this->get_parameter("d_term_filter_angle", regulator_angle_.d_term_filter_coefficient);
  regulator_angle_.angle_mode = true;

  this->get_parameter("angle_goal_tolerance", angle_goal_tolerance_);
  robot_velocity_angular_ = 0.0;
  /*****************************************************/

  robot_distance_ = 0;
  position_initialized_ = false;

  odometry_counter_ = 0;
  action_running_ = false;
  output_enabled_ = false;

  double control_frequency;
  this->get_parameter("control_frequency", control_frequency);
  const double control_period = 1.0 / control_frequency;
  motion_profile_ = new ruckig::Ruckig<2>{control_period};

  // Disabling Synchronization is important. We want independent control for distance and angle.
  motion_profile_input_.synchronization = ruckig::Synchronization::None;
  motion_profile_input_.max_velocity = {0.2, 2.0};
  motion_profile_input_.max_acceleration = {0.8, 1.5};
  motion_profile_input_.max_jerk = {
    999999999999.0, 999999999999.0};  // force trapezoidal velocity profile
  motion_profile_result_ = ruckig::Result::Finished;

  navigate_to_pose_server_ = std::make_unique<NavigateToPoseServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "~/navigate_to_pose",
    std::bind(&DistanceAngleRegulator::navigate_to_pose, this));

  motion_command_server_ = std::make_unique<MotionCommandServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "~/motion_command",
    std::bind(&DistanceAngleRegulator::motion_command, this));

  navigate_to_pose_server_->activate();
  motion_command_server_->activate();
}

void DistanceAngleRegulator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time time = this->get_clock()->now();

  std::unique_lock<std::mutex> lock(data_mutex_);

  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_velocity_linear_ = msg->twist.twist.linear.x;
  robot_velocity_angular_ = msg->twist.twist.angular.z;

  if (!position_initialized_) {
    prev_robot_x_ = robot_x_;
    prev_robot_y_ = robot_y_;
    position_initialized_ = true;

    robot_angle_ = tf2::getYaw(msg->pose.pose.orientation);

    motion_profile_input_.current_position = {0.0, robot_angle_};
    motion_profile_input_.target_position = {0.0, robot_angle_};
  }

  robot_angle_ = tf2::getYaw(msg->pose.pose.orientation);

  const double delta_x = robot_x_ - prev_robot_x_;
  const double delta_y = robot_y_ - prev_robot_y_;
  const double distance_increment = std::hypot(delta_x, delta_y);
  const double distance_increment_angle = std::atan2(delta_y, delta_x);

  int sign = 1;
  if (std::abs(angle_normalize(robot_angle_ - distance_increment_angle)) > 0.1) sign = -1;

  robot_distance_ += sign * distance_increment;

  regulator_distance_.feedback = robot_distance_;
  regulator_angle_.feedback = robot_angle_;

  /* RUCKIG */
  motion_profile_result_ = motion_profile_->update(motion_profile_input_, motion_profile_output_);

  if (motion_profile_result_ == ruckig::Result::Working) {
    regulator_distance_.reference = motion_profile_output_.new_position[0];
    regulator_angle_.reference = angle_normalize(motion_profile_output_.new_position[1]);
    motion_profile_output_.pass_to_input(motion_profile_input_);
  }
  /**********/
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

  const bool distance_finished = distance_regulator_finished();
  const bool angle_finished = angle_regulator_finished();

  if (action_running_) output_enabled_ = true;
  if (output_enabled_) {
    if (distance_finished && angle_finished) {
      output_enabled_ = false;
      motor_command.linear.x = 0.0;
      motor_command.angular.z = 0.0;
    }
    twist_publisher_->publish(motor_command);
  } else {
    // Make targets track current position so regulators don't go crazy while we aren't using them
    // motion_profile_input_.target_position[0] = robot_distance_;
    // motion_profile_input_.target_position[1] +=
    //   angle_normalize(robot_angle_ - motion_profile_input_.target_position[1]);
  }

  prev_robot_x_ = robot_x_;
  prev_robot_y_ = robot_y_;

  odometry_counter_++;
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
    } else if (param.get_name() == "d_term_filter_distance") {
      regulator_distance_.d_term_filter_coefficient = param.as_double();
    } else if (param.get_name() == "kp_angle") {
      regulator_angle_.kp = param.as_double();
    } else if (param.get_name() == "ki_angle") {
      regulator_angle_.ki = param.as_double();
    } else if (param.get_name() == "kd_angle") {
      regulator_angle_.kd = param.as_double();
    } else if (param.get_name() == "d_term_filter_angle") {
      regulator_angle_.d_term_filter_coefficient = param.as_double();
    } else if (param.get_name() == "distance_goal_tolerance") {
      distance_goal_tolerance_ = param.as_double();
    } else if (param.get_name() == "angle_goal_tolerance") {
      angle_goal_tolerance_ = param.as_double();
    } else if (param.get_name() == "debug") {
      debug_ = param.as_bool();
    }
  }

  return result;
}

void DistanceAngleRegulator::motion_command()
{
  auto result = std::make_shared<MotionCommandT::Result>();
  auto goal = motion_command_server_->get_current_goal();

  std::unique_lock<std::mutex> lock(data_mutex_);

  if (action_running_) {
    RCLCPP_WARN(
      rclcpp::get_logger(""), "Tried calling motion command action while other action is running!");
    motion_command_server_->terminate_current();
    return;
  } else {
    action_running_ = true;

    motion_profile_input_.current_position[0] = robot_distance_;
    motion_profile_input_.current_position[1] = robot_angle_;

    motion_profile_input_.target_position[0] = robot_distance_;
    motion_profile_input_.target_position[1] = robot_angle_;

    pid_regulator_reset(&regulator_distance_);
    pid_regulator_reset(&regulator_angle_);
  }

  if (goal->velocity_linear != 0) {
    motion_profile_input_.max_velocity[0] = goal->velocity_linear;
  }

  if (goal->acceleration_linear != 0) {
    motion_profile_input_.max_acceleration[0] = goal->acceleration_linear;
  }

  if (goal->velocity_angular != 0) {
    motion_profile_input_.max_velocity[1] = goal->velocity_angular;
  }

  if (goal->acceleration_angular != 0) {
    motion_profile_input_.max_acceleration[1] = goal->acceleration_angular;
  }
  lock.unlock();

  enum class MotionState { START, RUNNING_COMMAND, FINISHED };
  MotionState state = MotionState::START;

  rclcpp::WallRate r(200);
  const int timeout = 160;
  int timeout_counter;

  while (rclcpp::ok()) {
    lock.lock();

    if (motion_command_server_->is_cancel_requested()) {
      softstop();
      action_running_ = false;
      result->set__result("drift");
      motion_command_server_->terminate_current(result);
    }

    switch (state) {
      case MotionState::START:
        if (goal->command == "forward") {
          forward(goal->value);
        } else if (goal->command == "rotate_absolute") {
          rotate_absolute(goal->value);
        } else if (goal->command == "rotate_relative") {
          rotate_relative(goal->value);
        }
        timeout_counter = timeout;

        state = MotionState::RUNNING_COMMAND;
        break;

      case MotionState::RUNNING_COMMAND:
        if (motion_profile_finished()) timeout_counter--;
        if (timeout_counter <= 0) {
          state = MotionState::FINISHED;
        }
        if (distance_regulator_finished() && angle_regulator_finished()) {
          state = MotionState::FINISHED;
        }
        break;

      case MotionState::FINISHED:
        if (distance_regulator_finished() && angle_regulator_finished()) {
          result->set__result("success");
        } else {
          result->set__result("drift");
        }

        action_running_ = false;

        motion_command_server_->succeeded_current(result);
        return;
        break;

      default:
        break;
    }
    lock.unlock();
    r.sleep();
  }

  return;
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
  motion_profile_result_ = ruckig::Working;
  // motion_profile_input_.current_position[0] = robot_distance_;
  // motion_profile_input_.current_velocity[0] = robot_velocity_linear_;
  // motion_profile_input_.target_position[0] = robot_distance_ + distance;
  motion_profile_input_.target_position[0] = motion_profile_input_.current_position[0] + distance;
}

void DistanceAngleRegulator::rotate_absolute(double angle)
{
  motion_profile_result_ = ruckig::Working;
  rotate_relative(angle_normalize(angle - robot_angle_));
}

void DistanceAngleRegulator::rotate_relative(double angle)
{
  motion_profile_result_ = ruckig::Working;
  // motion_profile_input_.current_position[1] = robot_angle_;
  // motion_profile_input_.current_velocity[1] = robot_velocity_angular_;
  // motion_profile_input_.target_position[1] = robot_angle_ + angle;
  motion_profile_input_.target_position[1] = motion_profile_input_.current_position[1] + angle;
}

void DistanceAngleRegulator::softstop()
{
  double linear_stop_distance =
    std::pow(robot_velocity_linear_, 2) / (2.0 * motion_profile_input_.max_acceleration[0]);
  double angular_stop_distance =
    std::pow(robot_velocity_angular_, 2) / (2.0 * motion_profile_input_.max_acceleration[1]);

  linear_stop_distance = std::copysign(linear_stop_distance, robot_velocity_linear_);
  angular_stop_distance = std::copysign(angular_stop_distance, robot_velocity_angular_);
  forward(linear_stop_distance);
  rotate_relative(angular_stop_distance);
}

bool DistanceAngleRegulator::distance_regulator_finished()
{
  return motion_profile_finished() &&
         (std::abs(regulator_distance_.error) < distance_goal_tolerance_);
}

bool DistanceAngleRegulator::angle_regulator_finished()
{
  return motion_profile_finished() && (std::abs(regulator_angle_.error) < angle_goal_tolerance_);
}

bool DistanceAngleRegulator::motion_profile_finished()
{
  return motion_profile_result_ != ruckig::Result::Working;
}

void DistanceAngleRegulator::wait_for_odometry()
{
  uint64_t c = odometry_counter_;
  rclcpp::WallRate r(200);
  while (c == odometry_counter_) r.sleep();
}

void DistanceAngleRegulator::navigate_to_pose()
{
  auto result = std::make_shared<NavigatoToPoseT::Result>();
  auto goal = navigate_to_pose_server_->get_current_goal();

  const double goal_x = goal->pose.pose.position.x;
  const double goal_y = goal->pose.pose.position.y;

  enum class MotionState { START, ROTATING_TO_GOAL, MOVING_TO_GOAL, ROTATING_IN_GOAL, FINISHED };
  MotionState state = MotionState::START;

  std::unique_lock<std::mutex> lock(data_mutex_);

  if (action_running_) {
    RCLCPP_WARN(
      rclcpp::get_logger(""),
      "Tried calling navigate to goal action while other action is running!");
    navigate_to_pose_server_->terminate_current();
    return;
  } else {
    action_running_ = true;
    
    motion_profile_input_.current_position[0] = robot_distance_;
    motion_profile_input_.current_position[1] = robot_angle_;

    motion_profile_input_.target_position[0] = robot_distance_;
    motion_profile_input_.target_position[1] = robot_angle_;

    pid_regulator_reset(&regulator_distance_);
    pid_regulator_reset(&regulator_angle_);
  }

  double delta_x = goal_x - robot_x_;
  double delta_y = goal_y - robot_y_;
  lock.unlock();
  double distance_to_goal = std::hypot(delta_x, delta_y);
  double angle_to_goal = std::atan2(delta_y, delta_x);

  rclcpp::WallRate r(200);

  const int timeout = 160;
  int timeout_counter;

  while (rclcpp::ok()) {
    lock.lock();

    if (navigate_to_pose_server_->is_cancel_requested()) {
      softstop();
      action_running_ = false;
      navigate_to_pose_server_->terminate_current();
      return;
    }

    switch (state) {
      case MotionState::START:
        rotate_absolute(angle_to_goal);
        timeout_counter = timeout;
        state = MotionState::ROTATING_TO_GOAL;
        break;

      case MotionState::ROTATING_TO_GOAL:
        if (motion_profile_finished()) timeout_counter--;
        if (timeout_counter <= 0) {
          softstop();
          navigate_to_pose_server_->terminate_current();
          return;
        }
        if (angle_regulator_finished()) {
          delta_x = goal_x - robot_x_;
          delta_y = goal_y - robot_y_;
          distance_to_goal = std::hypot(delta_x, delta_y);
          forward(distance_to_goal);
          timeout_counter = timeout;
          state = MotionState::MOVING_TO_GOAL;
        }

        break;

      case MotionState::MOVING_TO_GOAL:
        RUN_EACH_NTH_CYCLES(uint8_t, 10, {
          delta_x = goal_x - robot_x_;
          delta_y = goal_y - robot_y_;
          distance_to_goal = std::hypot(delta_x, delta_y);
          angle_to_goal = std::atan2(delta_y, delta_x);
          // refresh only for longer moves
          if (distance_to_goal > 0.1) {
            // refresh both distance and angle
            motion_profile_input_.target_position[0] = robot_distance_ + distance_to_goal;
            rotate_absolute(angle_to_goal);
          }
        })  // refresh angle reference

        if (motion_profile_finished()) timeout_counter--;
        if (timeout_counter <= 0) {
          softstop();
          navigate_to_pose_server_->terminate_current();
          return;
        }
        if (distance_regulator_finished()) {
          const double yaw = tf2::getYaw(goal->pose.pose.orientation);
          rotate_absolute(yaw);
          timeout_counter = timeout;
          state = MotionState::ROTATING_IN_GOAL;
        }
        break;

      case MotionState::ROTATING_IN_GOAL:
        if (motion_profile_finished()) timeout_counter--;
        if (timeout_counter <= 0) {
          softstop();
          navigate_to_pose_server_->terminate_current();
          return;
        }
        if (angle_regulator_finished()) {
          state = MotionState::FINISHED;
        }
        break;

      case MotionState::FINISHED:
        action_running_ = false;
        navigate_to_pose_server_->succeeded_current(result);
        return;
        break;

      default:
        break;
    }
    lock.unlock();
    r.sleep();
  }
  navigate_to_pose_server_->terminate_current(result);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceAngleRegulator>());
  rclcpp::shutdown();

  return 0;
}
