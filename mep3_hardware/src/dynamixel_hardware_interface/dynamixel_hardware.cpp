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

#include "mep3_hardware/dynamixel_hardware_interface/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware
{
  constexpr const char *kDynamixelHardware = "DynamixelHardware";
  constexpr uint8_t kGoalPositionIndex = 0;
  constexpr uint8_t kGoalVelocityIndex = 1;
  constexpr uint8_t kPresentPositionVelocityCurrentIndex = 0;
  constexpr const char *kGoalPositionItem = "Goal_Position";
  constexpr const char *kGoalVelocityItem = "Goal_Velocity";
  constexpr const char *kMovingSpeedItem = "Moving_Speed";
  constexpr const char *kPresentPositionItem = "Present_Position";
  constexpr const char *kPresentVelocityItem = "Present_Velocity";
  constexpr const char *kPresentSpeedItem = "Present_Speed";
  constexpr const char *kPresentCurrentItem = "Present_Current";
  constexpr const char *kPresentLoadItem = "Present_Load";
  constexpr const char *const kExtraJointParameters[] = {
      "Profile_Velocity",
      "Profile_Acceleration",
      "Position_P_Gain",
      "Position_I_Gain",
      "Position_D_Gain"
      "Velocity_P_Gain",
      "Velocity_I_Gain",
  };

  CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Configure");
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    joints_.resize(info_.joints.size(), Joint());
    joint_ids_.resize(info_.joints.size(), 0);

    try {
      effort_average_ = std::stoi(info_.hardware_parameters.at("effort_average"));
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "effort_average: %d samples", effort_average_);
    } catch (const std::out_of_range& e) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "effort_average: %d samples [default]", effort_average_);
    }

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
      // Command
      joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.timeout = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.recovery_position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.recovery_mode_ = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.recovery_mode = mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_STAY;
      // State
      joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.voltage = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.temperature = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.overloaded = false;
      joints_[i].state.high_torque = false;
      joints_[i].state.recovery_state_ = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.recovery_state = OFF;
      // Bookkeeping
      joints_[i].state.previous_efforts_ = std::deque<double>();
      joints_[i].state.previous_efforts_.resize(effort_average_);
      joints_[i].state.previous_safe_position_ = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.recovery_pending_start_.reset();
      joints_[i].state.recovery_off_start_.reset();
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
    }

    if (
        info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
        info_.hardware_parameters.at("use_dummy") == "true")
    {
      use_dummy_ = true;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
      return CallbackReturn::SUCCESS;
    }

    auto usb_port = info_.hardware_parameters.at("usb_port");
    auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    if (info_.hardware_parameters.find("offset") != info_.hardware_parameters.end())
      offset_ = std::stof(info_.hardware_parameters.at("offset"));
    const char *log = nullptr;

    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port.c_str());
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate);

    try {
      recovery_timeout_ = std::chrono::milliseconds(stoi(info_.hardware_parameters.at("recovery_timeout")));
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "recovery_timeout: %ld ms", recovery_timeout_.count());
    } catch (const std::out_of_range& e) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "recovery_timeout: %ld ms [default]", recovery_timeout_.count());
    }

    try {
      torque_threshold_ = stof(info_.hardware_parameters.at("torque_threshold"));
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "torque_threshold: %.2f", torque_threshold_);
    } catch (const std::out_of_range& e) {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "torque_threshold: %.2f [default]", torque_threshold_);
    }

    if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < info_.joints.size(); ++i)
    {
      uint16_t model_number = 0;
      bool sucess = false;
      uint8_t retry_count = 0;
      while (!sucess && retry_count < 10) {
        retry_count++;
        sucess = dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log);
        if (!sucess)
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
      }
      if (!sucess)
        return CallbackReturn::ERROR;
    }

    enable_torque(false);
    set_control_mode(ControlMode::Position, true);
    for (uint i = 0; i < info_.joints.size(); ++i)
    {
      for (auto paramName : kExtraJointParameters)
      {
        if (info_.joints[i].parameters.find(paramName) != info_.joints[i].parameters.end())
        {
          auto value = std::stoi(info_.joints[i].parameters.at(paramName));
          if (!dynamixel_workbench_.itemWrite(joint_ids_[i], paramName, value, &log))
          {
            RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
            return CallbackReturn::ERROR;
          }
          RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "%s set to %d for joint %d", paramName, value, i);
        }
      }
    }
    enable_torque(true);

    const ControlItem *goal_position =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
    if (goal_position == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *goal_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
    if (goal_velocity == nullptr)
    {
      goal_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
    }
    if (goal_velocity == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_position =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
    if (present_position == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
    if (present_velocity == nullptr)
    {
      present_velocity = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
    }
    if (present_velocity == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_current =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
    if (present_current == nullptr)
    {
      present_current = dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
    }
    if (present_current == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    control_items_[kGoalPositionItem] = goal_position;
    control_items_[kGoalVelocityItem] = goal_velocity;
    control_items_[kPresentPositionItem] = present_position;
    control_items_[kPresentVelocityItem] = present_velocity;
    control_items_[kPresentCurrentItem] = present_current;

    if (!dynamixel_workbench_.addSyncWriteHandler(
            control_items_[kGoalPositionItem]->address, control_items_[kGoalPositionItem]->data_length,
            &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    if (!dynamixel_workbench_.addSyncWriteHandler(
            control_items_[kGoalVelocityItem]->address, control_items_[kGoalVelocityItem]->data_length,
            &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    uint16_t start_address = std::min(
        control_items_[kPresentPositionItem]->address, control_items_[kPresentCurrentItem]->address);
    uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                           control_items_[kPresentVelocityItem]->data_length +
                           control_items_[kPresentCurrentItem]->data_length + 2;
    if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, "recovery_state", &joints_[i].state.recovery_state_));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].command.effort));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, "timeout", &joints_[i].command.timeout));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, "recovery_position", &joints_[i].command.recovery_position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, "recovery_mode", &joints_[i].command.recovery_mode_));
    }

    return command_interfaces;
  }

  CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Start");
    for (uint i = 0; i < joints_.size(); i++)
    {
      if (use_dummy_ && std::isnan(joints_[i].state.position))
      {
        joints_[i].state.position = 0.0;
        joints_[i].state.velocity = 0.0;
        joints_[i].state.effort = 0.0;
      }
    }

    this->read_from_hardware();
    this->reset_command();
    this->write_to_hardware();

    // Start read/write thread
    keep_read_write_thread_ = true;
    std::thread read_write_thread(
        [=]()
        {
          while (rclcpp::ok() && keep_read_write_thread_)
          {
            this->read_from_hardware();
            this->update_command();
            this->write_to_hardware();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        });
    read_write_thread.detach();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    keep_read_write_thread_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "Stop");
    return CallbackReturn::SUCCESS;
  }

  return_type DynamixelHardware::read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    return return_type::OK;
  }

  void DynamixelHardware::read_from_hardware()
  {
    if (use_dummy_)
    {
      for (uint i = 0; i < joints_.size(); i++)
      {
        joints_[i].state.position = joints_[i].command.position;
        joints_[i].state.velocity = joints_[i].command.velocity;
        joints_[i].state.effort = joints_[i].command.effort;
      }
      return;
    }

    dynamixel_workbench_.getProtocolVersion() > 1.5
        ? read2(rclcpp::Time{}, rclcpp::Duration(0, 0))
        : read1(rclcpp::Time{}, rclcpp::Duration(0, 0));
  }

  void DynamixelHardware::write_to_hardware()
  {
    if (use_dummy_)
    {
      return;
    }

    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> commands(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char *log = nullptr;

    if (std::any_of(
            joints_.cbegin(), joints_.cend(), [](auto j)
            { return j.command.velocity != 0.0; }))
    {
      // Velocity control
      // TODO: Add VelocityPosition mode
      // set_control_mode(ControlMode::Velocity);
      for (uint i = 0; i < ids.size(); i++)
      {
        commands[i] = dynamixel_workbench_.convertVelocity2Value(
            ids[i], static_cast<float>(joints_[i].command.velocity));
      }
      if (!dynamixel_workbench_.syncWrite(
              kGoalVelocityIndex, ids.data(), ids.size(), commands.data(), 1, &log))
      {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
      }
      // TODO: Add VelocityPosition mode
      // return return_type::OK;
    }
    else if (std::any_of(
                 joints_.cbegin(), joints_.cend(), [](auto j)
                 { return j.command.effort != 0.0; }))
    {
      // Effort control
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Effort control is not implemented");
      return;
    }

    // Position control
    set_control_mode(ControlMode::Position);
    for (uint i = 0; i < ids.size(); i++)
    {
      commands[i] = dynamixel_workbench_.convertRadian2Value(
          ids[i], static_cast<float>(joints_[i].command.position) - offset_);
    }
    if (!dynamixel_workbench_.syncWrite(
            kGoalPositionIndex, ids.data(), ids.size(), commands.data(), 1, &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }
  }

  void DynamixelHardware::read2(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  /* Read function for protocol 2.0 */
  {
    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> positions(info_.joints.size(), 0);
    std::vector<int32_t> velocities(info_.joints.size(), 0);
    std::vector<int32_t> currents(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char *log = nullptr;

    if (!dynamixel_workbench_.syncRead(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(), &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentCurrentItem]->address,
            control_items_[kPresentCurrentItem]->data_length, currents.data(), &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentVelocityItem]->address,
            control_items_[kPresentVelocityItem]->data_length, velocities.data(), &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentPositionItem]->address,
            control_items_[kPresentPositionItem]->data_length, positions.data(), &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "%s", log);
    }

    for (uint i = 0; i < ids.size(); i++)
    {
      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]) + offset_;
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
      joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
    }
  }

  bool DynamixelHardware::timeout_passed(std::chrono::time_point<std::chrono::system_clock> & start_time, double joint_timeout) {
    const auto time_delta = std::chrono::system_clock::now() - start_time;
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_delta);
    const auto joint_duration = std::chrono::milliseconds((int) (joint_timeout * 1000.0));
    return duration > recovery_timeout_ || duration > joint_duration;
  }

  std::string DynamixelHardware::recovery_mode(const int8_t mode) {
    switch (mode)
    {
    case mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_STAY:
      return "STAY";
    case mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_RETURN:
      return "RETURN";
    default:
      return "";
    }
  }
  
  std::string DynamixelHardware::recovery_state(const enum RecoveryState state) {
    switch (state)
    {
    case OFF:
      return "OFF";
    case PENDING:
      return "PENDING";
    case ACTIVE:
      return "ACTIVE";
    default:
      return "";
    }
  }

  int8_t DynamixelHardware::to_recovery_mode(const double mode) {
    if (mode > -0.5 && mode < 0.5) {
      return mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_STAY;
    }
    if (mode > 0.5 && mode < 1.5) {
      return mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_RETURN;
    }
    // Default
    return mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_STAY;
  }
  
  double DynamixelHardware::from_recovery_state(const enum RecoveryState state) {
    switch (state)
    {
    case OFF:
      return 0.0;
    case PENDING:
      return 1.0;
    case ACTIVE:
      return 2.0;
    default:
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  void DynamixelHardware::update_command() {

    for (auto&& joint = joints_.begin(); joint != joints_.end(); ++joint)
    {
      // Cast double recieved from controller into enum
      joint->command.recovery_mode = to_recovery_mode(joint->command.recovery_mode_);
      joint->state.recovery_state_ = from_recovery_state(joint->state.recovery_state);

      // Set recovery position to safe position if NaN
      if (std::isnan(joint->command.recovery_position)) {
        joint->command.recovery_position = joint->state.previous_safe_position_;
      }

      const bool max_effort_reached = joint->command.effort > 0 && joint->state.effort > joint->command.effort;

      // Joint is under high torque, overloaaded or has reached maximum torque from command
      if (joint->state.high_torque || joint->state.overloaded || max_effort_reached) {
        // Reset recovery off counter
        joint->state.recovery_off_start_.reset();
        // Encountered high torque, set recovery state to pending
        if (joint->state.recovery_state == OFF) {
          // RCLCPP_WARN(
          //   rclcpp::get_logger(kDynamixelHardware),
          //   "Joint enter PENDING recovery state"
          // );
          joint->state.recovery_state = PENDING;
          joint->state.recovery_pending_start_ = std::chrono::system_clock::now();
        }
        // Enter active recovery state if pending reaches timeout
        else if (joint->state.recovery_state == PENDING) {
          if (!joint->state.recovery_pending_start_.has_value()) {
            RCLCPP_FATAL(
              rclcpp::get_logger(kDynamixelHardware),
              "Recovery timeout start time for joint is missing"
            );
            continue;
          }
          if (this->timeout_passed(joint->state.recovery_pending_start_.value(), joint->command.timeout)) {
            // RCLCPP_WARN(
            //   rclcpp::get_logger(kDynamixelHardware),
            //   "Joint enter ACTIVE recovery state"
            // );
            joint->state.recovery_state = ACTIVE;
          }
        }
      } else {
        // Set prevoious safe position to current one (effort is tolerable)
        joint->state.previous_safe_position_ = joint->state.position;
        // Return from recovery if necessary
        if (joint->state.recovery_state != OFF) {
          if (joint->state.recovery_off_start_.has_value()) {
            if (this->timeout_passed(joint->state.recovery_off_start_.value(), joint->command.timeout)) {
              // RCLCPP_WARN(
              //   rclcpp::get_logger(kDynamixelHardware),
              //   "Joint enter OFF recovery state"
              // );
              joint->state.recovery_state = OFF;
              joint->state.recovery_pending_start_.reset();
              joint->state.recovery_off_start_.reset();
            }
          } else {
            joint->state.recovery_off_start_ = std::chrono::system_clock::now();
          }
        }
      }

      // Recover to position set by controller
      if (joint->state.recovery_state == ACTIVE) {
        switch (joint->command.recovery_mode) {
        case mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_STAY:
          joint->command.position = joint->state.previous_safe_position_;
          break;
        case mep3_msgs::action::JointPositionCommand::Goal::RECOVERY_RETURN:
          joint->command.position = joint->command.recovery_position;
          break;
        }
        const double RECOVERY_TOLERANCE = 0.1;
        if (abs(joint->state.position - joint->command.position) < RECOVERY_TOLERANCE) {
          joint->state.recovery_state = OFF;
          // RCLCPP_WARN(
          //   rclcpp::get_logger(kDynamixelHardware),
          //   "Joint enter OFF recovery state"
          // );
        }
      }
    }
  }

  void DynamixelHardware::read1(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  /* Read function for protocol 1.0 */
  {
    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> positions(info_.joints.size(), 0);
    std::vector<int32_t> velocities(info_.joints.size(), 0);
    std::vector<int32_t> currents(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char *log = nullptr;

    for (uint i = 0; i < ids.size(); i++)
    {
      // https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
      // ax12 present position address: 36 [2B]
      // ax12 present speed address: 38 [2B]
      // ax12 present load address: 40 [2B]
      // ax12 present voltage address: 42 [1B]
      // ax12 present temperature address: 43 [1B]

      const unsigned PRESENT_DATA_ADDRESS = 36;
      const unsigned PRESENT_DATA_BYTES = 2+2+2+1+1;
      const unsigned TORQUE_LOAD_MAX = 1023;

      unsigned int present_data[PRESENT_DATA_BYTES];
      if (!dynamixel_workbench_.readRegister(ids[i], PRESENT_DATA_ADDRESS, PRESENT_DATA_BYTES, present_data, &log)) {
        RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "read0: %s", log);
        dynamixel_workbench_.itemWrite(ids[i], "Torque_Limit", (int32_t)TORQUE_LOAD_MAX, &log);
        dynamixel_workbench_.itemWrite(ids[i], "Torque_Enable", (int32_t)1, &log);
      }

      int16_t position = present_data[0] | (present_data[1] << 8);
      
      int16_t speed =    (present_data[2] | ((0x3 & present_data[3]) << 8));
      // data[3] third bit determines speed sign
      if (present_data[3] & 0x4)
        speed = -speed;

      int16_t load =     (present_data[4] | ((0x3 & present_data[5]) << 8));
      bool overload = (unsigned) load > TORQUE_LOAD_MAX;
      bool high_torque = (double) load >= TORQUE_LOAD_MAX * torque_threshold_;
      // data[5] third bit determines effort sign
      if (present_data[5] & 0x4)
        load = -load;

      uint8_t voltage = present_data[6];
      uint8_t temperature = present_data[7];

      joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], position) + offset_;
      joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], speed);
      joints_[i].state.voltage = voltage / 10.0;
      joints_[i].state.temperature = temperature;
      joints_[i].state.overloaded = overload;
      joints_[i].state.high_torque = high_torque;

      // Read average effort from joint
      if (overload) {
        RCLCPP_WARN(
          rclcpp::get_logger(kDynamixelHardware),
          "Joint %s is overloaded",
          info_.joints[i].name.c_str()
        );
        joints_[i].state.effort = std::numeric_limits<double>::infinity();
      } else {
        double effort = dynamixel_workbench_.convertValue2Current(load);
        if (joints_[i].state.previous_efforts_.size() >= effort_average_) {
          joints_[i].state.previous_efforts_.pop_front();
        }
        joints_[i].state.previous_efforts_.push_back(abs(effort));
        double new_effort_average = 0;
        for (unsigned int j = 0; j < joints_[i].state.previous_efforts_.size(); ++j) {
          new_effort_average += joints_[i].state.previous_efforts_[j];
        }
        new_effort_average /= joints_[i].state.previous_efforts_.size();
        joints_[i].state.effort = new_effort_average;
      }

      // RCLCPP_WARN(
      //   rclcpp::get_logger(kDynamixelHardware),
      //   "DYNAMIXEL [ position: %6.2f rad | speed: %6.2f rad/s | load: %6.2f mA | voltage: %6.2f V | temperature: %6.2f C ]",
      //   joints_[i].state.position,
      //   joints_[i].state.velocity,
      //   joints_[i].state.effort,
      //   joints_[i].state.voltage,
      //   joints_[i].state.temperature
      // );
    }
  }

  return_type DynamixelHardware::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
  {
    return return_type::OK;
  }

  return_type DynamixelHardware::enable_torque(const bool enabled)
  {
    const char *log = nullptr;

    if (enabled && !torque_enabled_)
    {
      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      reset_command();
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque enabled");
    }
    else if (!enabled && torque_enabled_)
    {
      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Torque disabled");
    }

    torque_enabled_ = enabled;
    return return_type::OK;
  }

  return_type DynamixelHardware::set_control_mode(const ControlMode &mode, const bool force_set)
  {
    const char *log = nullptr;

    if (mode == ControlMode::Velocity && (force_set || control_mode_ != ControlMode::Velocity))
    {
      bool torque_enabled = torque_enabled_;
      if (torque_enabled)
      {
        enable_torque(false);
      }

      for (uint i = 0; i < joint_ids_.size(); ++i)
      {
        if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Velocity control");
      control_mode_ = ControlMode::Velocity;

      if (torque_enabled)
      {
        enable_torque(true);
      }
    }
    else if (
        mode == ControlMode::Position && (force_set || control_mode_ != ControlMode::Position))
    {
      bool torque_enabled = torque_enabled_;
      if (torque_enabled)
      {
        enable_torque(false);
      }

      for (uint i = 0; i < joint_ids_.size(); ++i)
      {
        if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Position control");
      control_mode_ = ControlMode::Position;

      if (torque_enabled)
      {
        enable_torque(true);
      }
    }
    else if (control_mode_ != ControlMode::Velocity && control_mode_ != ControlMode::Position)
    {
      RCLCPP_FATAL(
          rclcpp::get_logger(kDynamixelHardware), "Only position/velocity control are implemented");
      return return_type::ERROR;
    }

    return return_type::OK;
  }

  return_type DynamixelHardware::reset_command()
  {
    for (uint i = 0; i < joints_.size(); i++)
    {
      joints_[i].command.position = joints_[i].state.position;
      joints_[i].command.velocity = 0.0;
      joints_[i].command.effort = 0.0;
    }

    return return_type::OK;
  }

} // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)
