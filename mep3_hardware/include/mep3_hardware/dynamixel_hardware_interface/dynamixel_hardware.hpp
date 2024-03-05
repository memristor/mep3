// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <vector>
#include <deque>
#include <optional>
#include <chrono>

#include "mep3_hardware/dynamixel_hardware_interface/visibility_control.hpp"
#include "rclcpp/macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace dynamixel_hardware
{

struct JointState
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double voltage{0.0};
  double temperature{0.0};
  bool overloaded;
  std::deque<double> previous_efforts_{};
  std::optional<std::chrono::time_point<std::chrono::system_clock>> high_torque_start{};
};

struct JointCommand
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double recovery_position{0.0};
};

struct Joint
{
  JointState state{};
  JointCommand command{};
};

enum class ControlMode {
  Position,
  Velocity,
  Torque,
  Currrent,
  ExtendedPosition,
  MultiTurn,
  CurrentBasedPosition,
  PWM,
};

class DynamixelHardware
: public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DYNAMIXEL_HARDWARE_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  return_type enable_torque(const bool enabled);

  return_type set_control_mode(const ControlMode & mode, const bool force_set = false);

  return_type reset_command();

  void read_from_hardware();
  void write_to_hardware();

  void read1(const rclcpp::Time & time, const rclcpp::Duration & period);
  void read2(const rclcpp::Time & time, const rclcpp::Duration & period);

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool torque_enabled_{false};
  ControlMode control_mode_{ControlMode::Position};
  bool use_dummy_{false};
  double offset_{0};
  bool keep_read_write_thread_{true};
  unsigned int effort_average_ {0};
  double torque_threshold_ {0.9};
  std::chrono::milliseconds recovery_timeout_{250};
};
}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_