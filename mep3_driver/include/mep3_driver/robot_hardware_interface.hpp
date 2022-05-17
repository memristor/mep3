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

#ifndef MEP3_DRIVER__ROBOT_HARDWARE_INTERFACE_HPP_
#define MEP3_DRIVER__ROBOT_HARDWARE_INTERFACE_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motion_board_driver.hpp"

#define POW2(N) (1UL << (N))

namespace mep3_driver
{
class RobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  double left_wheel_velocity_command_;
  double left_wheel_position_state_;
  double left_wheel_velocity_state_;
  double right_wheel_velocity_command_;
  double right_wheel_position_state_;
  double right_wheel_velocity_state_;

  int32_t prev_left_wheel_raw_;
  int32_t prev_right_wheel_raw_;
  int odom_left_overflow_;
  int odom_right_overflow_;

  float kp_linear_, ki_linear_, kd_linear_;
  float kp_angular_, ki_angular_, kd_angular_;
  double update_rate_;

  // Encoder Resolution = How many ticks for one full rotation
  static constexpr double ENCODER_RESOLUTION = 8192.0;  // 2048 * 4

  MotionBoardDriver motion_board_;
};
}  // namespace mep3_driver

#endif  // MEP3_DRIVER__ROBOT_HARDWARE_INTERFACE_HPP_
