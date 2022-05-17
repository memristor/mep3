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

#include "mep3_driver/dynamixel_driver.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace mep3_driver
{
DynamixelDriver::DynamixelDriver(const rclcpp::NodeOptions & options)
: Node("dynamixel_driver", options)
{
  this->declare_parameter<std::string>("usb_port");
  this->declare_parameter<int>("baud_rate");
  this->declare_parameter<std::vector<std::string>>("joint_names");
  this->declare_parameter<double>("control_frequency");

  this->get_parameter("usb_port", usb_port_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("joint_names", joint_names_);

  RCLCPP_INFO(
    this->get_logger(), "USB Port: %s, baud rate: %d", usb_port_.c_str(), (int)baud_rate_);

  for (auto j : joint_names_) {
    this->declare_parameter<uint8_t>((j + ".id").c_str());
    uint8_t id;
    this->get_parameter((j + ".id").c_str(), id);
    joint_ids_.push_back(id);
    joint_moving_speeds_.push_back(kDefaultMovingSpeed);
    RCLCPP_INFO(this->get_logger(), "Joint: %s, id: %d", j.c_str(), (int)id);
  }

  const char * log = nullptr;

  if (!dynamixel_workbench_.init(usb_port_.c_str(), baud_rate_, &log)) {
    RCLCPP_FATAL(this->get_logger(), "%s", log);
    rclcpp::shutdown();
    return;
  }

  for (uint i = 0; i < joint_names_.size(); i++) {
    // ping every dynamixel, we will need this for sync write
    int try_count = 50;
    while (try_count >= 0) {
      if (dynamixel_workbench_.ping(joint_ids_[i])) {
        try_count = 50;
        break;
      } else {
        try_count--;
      }
    }

    if (try_count < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to ping dynamixel ID: %d", (int)joint_ids_[i]);
      rclcpp::shutdown();
      return;
    }

    // Position mode
    while (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log))
      ;

    // Torque On
    while (!dynamixel_workbench_.torqueOn(joint_ids_[i]))
      ;

    // Get position offset so zero rad means zero increments, not middlepoint
    const float offset = dynamixel_workbench_.convertValue2Radian(joint_ids_[i], 0);
    joint_position_offsets_.push_back(offset);

    // get current position
    int32_t position = 0;
    while (!dynamixel_workbench_.itemRead(joint_ids_[i], "Present_Position", &position, &log)) {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
    }
    joint_present_positions_.push_back(
      dynamixel_workbench_.convertValue2Radian(joint_ids_[i], position) - offset);

    joint_goal_positions_.push_back(joint_present_positions_[i]);

    action_servers_.push_back(std::make_shared<DynamixelCommandServer>(
      this, (std::string("dynamixel_command/") + joint_names_[i]).c_str(),
      std::bind(&DynamixelDriver::action_execute, this, i), nullptr,
      std::chrono::milliseconds(1500), true, rcl_action_server_get_default_options()));
    action_servers_[i]->activate();
  }

  const ControlItem * moving_speed =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], "Moving_Speed", &log);
  if (moving_speed == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "%s", log);
  }

  const ControlItem * goal_position =
    dynamixel_workbench_.getItemInfo(joint_ids_[0], "Goal_Position", &log);
  if (goal_position == nullptr) {
    RCLCPP_FATAL(this->get_logger(), "%s", log);
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        moving_speed->address, moving_speed->data_length, &log)) {
    RCLCPP_FATAL(this->get_logger(), "%s", log);
  }

  if (!dynamixel_workbench_.addSyncWriteHandler(
        goal_position->address, goal_position->data_length, &log)) {
    RCLCPP_FATAL(this->get_logger(), "%s", log);
  }

  RCLCPP_INFO(this->get_logger(), "Ready!");

  double control_frequency;
  this->get_parameter("control_frequency", control_frequency);
  const double control_period = 1.0 / control_frequency;
  std::chrono::duration<double> chrono_control_period(control_period);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), chrono_control_period,
    std::bind(&DynamixelDriver::control_loop, this));
}

void DynamixelDriver::action_execute(uint index)
{
  RCLCPP_INFO(this->get_logger(), "Action execute!");
  auto result = std::make_shared<DynamixelCommandT::Result>();
  auto goal = action_servers_[index]->get_current_goal();

  std::unique_lock<std::mutex> lock(data_mutex_);

  RCLCPP_INFO(
    this->get_logger(), "Dynamixel ID: %d, Goal Position Deg: %lf", (int)joint_ids_[index],
    goal->position);
  joint_goal_positions_[index] = static_cast<float>(DEG2RAD(goal->position));
  joint_moving_speeds_[index] = static_cast<float>(DEG2RAD(goal->velocity));
  lock.unlock();

  float tolerance = kDefaultTolerance;
  if (goal->tolerance != 0.0) {
    tolerance = goal->tolerance;
  }

  float timeout = kDefaultTimeout;
  if (goal->timeout != 0.0) {
    timeout = goal->timeout;
  }

  if (goal->position > 512.0) {
    torque_enabled = false;
    RCLCPP_WARN(this->get_logger(), "Torque Disable Request");
    result->set__result(0);
    action_servers_[index]->succeeded_current(result);
    return;
  }

  int timeout_counter = static_cast<int>(timeout / 0.1);  // we will poll at 10 Hz
  bool success = false;

  while (rclcpp::ok() && timeout_counter > 0) {
    lock.lock();

    if (action_servers_[index]->is_cancel_requested()) {
      RCLCPP_WARN(this->get_logger(), "Dynamixel ID: %d goal CANCELLED!", (int)joint_ids_[index]);
      result->set__result(2);
      action_servers_[index]->terminate_current(result);
      return;
    }

    if (action_servers_[index]->is_preempt_requested()) {
      RCLCPP_WARN(this->get_logger(), "Dynamixel ID: %d goal PREEMPTED!", (int)joint_ids_[index]);
      result->set__result(2);
      action_servers_[index]->terminate_current(result);
      return;
    }

    const float position_error =
      std::abs(joint_goal_positions_[index] - joint_present_positions_[index]);
    lock.unlock();
    if (position_error <= DEG2RAD(tolerance)) {
      success = true;
      break;
    }
    timeout_counter--;
    std::this_thread::sleep_for(100ms);
  }

  if (success) {
    result->set__result(0);
    action_servers_[index]->succeeded_current(result);
    return;
  } else {
    result->set__result(1);
    action_servers_[index]->terminate_current(result);
  }
}

int32_t DynamixelDriver::radian_to_value(uint8_t id, float radian)
{
  int32_t position = 0;

  auto model_info = dynamixel_workbench_.getModelInfo(id);
  if (model_info == NULL) return false;

  const int64_t value_max = model_info->value_of_max_radian_position - 1;

  if (radian >= 0) {
    position = radian / (2 * model_info->max_radian) * value_max;
    if (position > value_max) position = value_max;
  } else {
    position = model_info->value_of_min_radian_position;
  }

  return position;
}

int32_t DynamixelDriver::velocity_to_value(uint8_t id, float velocity)
{
  int32_t value = 0;
  const float RPM2RADPERSEC = 0.104719755f;

  auto model_info = dynamixel_workbench_.getModelInfo(id);
  if (model_info == NULL) return false;

  value = velocity / (model_info->rpm * RPM2RADPERSEC);

  if (value > 1023) value = 1023;

  return value;
}

void DynamixelDriver::control_loop()
{
  // RCLCPP_INFO(this->get_logger(), "Control loop!");
  std::unique_lock<std::mutex> lock(data_mutex_, std::defer_lock);
  static bool prev_torque_enabled = true;

  if (prev_torque_enabled == true && torque_enabled == false) {
    lock.lock();
    for (uint i = 0; i < joint_ids_.size(); i++) {
      dynamixel_workbench_.torqueOff(joint_ids_[i]);
    }
    RCLCPP_WARN(this->get_logger(), "TORQUE DISABLED");
    lock.unlock();
  }

  if (torque_enabled) {
    const char * log = nullptr;

    lock.lock();

    std::vector<int32_t> data(joint_ids_.size(), 0);

    // write commanded velocities (Moving Speed)
    for (uint i = 0; i < joint_ids_.size(); i++) {
      data[i] = velocity_to_value(joint_ids_[i], joint_moving_speeds_[i]);
    }
    if (!dynamixel_workbench_.syncWrite(0, data.data(), &log)) {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    // write goal positions
    for (uint i = 0; i < joint_ids_.size(); i++) {
      data[i] = radian_to_value(joint_ids_[i], joint_goal_positions_[i]);
    }
    if (!dynamixel_workbench_.syncWrite(1, data.data(), &log)) {
      RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    // read current positions
    int32_t position;
    for (uint i = 0; i < joint_ids_.size(); i++) {
      if (!dynamixel_workbench_.itemRead(joint_ids_[i], "Present_Position", &position, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
      } else {
        joint_present_positions_[i] =
          dynamixel_workbench_.convertValue2Radian(joint_ids_[i], position) -
          joint_position_offsets_[i];
      }
    }

    lock.unlock();
  }

  prev_torque_enabled = torque_enabled;
}

}  // namespace mep3_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::ExecutorOptions options;
  rclcpp::executors::MultiThreadedExecutor executor(
    options, (size_t)12, false, std::chrono::nanoseconds(-1));
  auto node = std::make_shared<mep3_driver::DynamixelDriver>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
