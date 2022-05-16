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

#ifndef MEP3_DRIVER__DYNAMIXEL_DRIVER_HPP_
#define MEP3_DRIVER__DYNAMIXEL_DRIVER_HPP_

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define DEG2RAD(x) (x * M_PI / 180.0)

#include "mep3_driver/simple_action_server.hpp"
#include "mep3_msgs/action/dynamixel_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mep3_driver
{
class DynamixelDriver : public rclcpp::Node
{
public:
  explicit DynamixelDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  using DynamixelCommandT = mep3_msgs::action::DynamixelCommand;
  using DynamixelCommandServer = nav2_util::SimpleActionServer<DynamixelCommandT>;

private:
  DynamixelWorkbench dynamixel_workbench_;
  std::string usb_port_;
  int baud_rate_;
  std::atomic_bool torque_enabled = true;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> joint_names_;
  std::vector<uint8_t> joint_ids_;
  std::vector<float> joint_present_positions_;  // rad
  std::vector<float> joint_position_offsets_;   // rad
  std::vector<float> joint_goal_positions_;     // rad
  std::vector<float> joint_moving_speeds_;      // rad/s
  std::mutex data_mutex_;
  std::vector<std::shared_ptr<DynamixelCommandServer>> action_servers_;

  const float kDefaultMovingSpeed = 11.9;  // rad/s
  const float kDefaultTolerance = 5.0;     // deg
  const float kDefaultTimeout = 5.0;       // s

  void control_loop();
  void action_execute(uint index);

  // helpers since dynamixel workbench does not clamp
  int32_t radian_to_value(uint8_t id, float radian);
  int32_t velocity_to_value(uint8_t id, float velocity);
};

}  // namespace mep3_driver

#endif  // MEP3_DRIVER__DYNAMIXEL_DRIVER_HPP_
