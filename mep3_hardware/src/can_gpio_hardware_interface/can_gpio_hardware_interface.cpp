// Copyright 2023 Memristor Robotics
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

#include "mep3_hardware/can_gpio_hardware_interface/can_gpio_hardware_interface.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define MESSAGE_SIZE 8

namespace mep3_hardware
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGpioHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    
    interface_name_ = info_.hardware_parameters.at("interface_name");
    filter_can_id_ = std::stoi(info_.hardware_parameters.at("can_id"));
    filter_can_mask_ = std::stoi(info_.hardware_parameters.at("can_mask"));

    for (hardware_interface::ComponentInfo component : info.gpios)
    {
      if (component.command_interfaces.size() > 0 && component.state_interfaces.size() > 0)
      {
        RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "%s cannot be state and command pin at the same time", component.name.c_str());
        return CallbackReturn::FAILURE;
      }

      if (component.command_interfaces.size() + component.state_interfaces.size() > 1)
      {
        RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "%s cannot be more than one interface type at the same time", component.name.c_str());
        return CallbackReturn::FAILURE;
      }

      if (component.command_interfaces.size() > 0 && component.command_interfaces.at(0).name != "output")
      {
        RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "%s can only be `output` type", component.name.c_str());
        return CallbackReturn::FAILURE;
      }

      if (component.state_interfaces.size() > 0 && component.state_interfaces.at(0).name != "input")
      {
        RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "%s can only be `input` type", component.name.c_str());
        return CallbackReturn::FAILURE;
      }

      Pin pin;
      pin.name = component.name;
      pin.index = std::stoi(component.parameters.at("index"));
      pin.direction = (component.command_interfaces.size() > 0) ? PinType::DIGITAL_OUTPUT : PinType::DIGITAL_INPUT;
      pins_.emplace_back(pin);
    }

    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGpioHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    if ((canbus_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW_FILTER)) < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "Socket error");
      return CallbackReturn::FAILURE;
    }

    filter_.can_id = (uint32_t)filter_can_id_;
    filter_.can_id |= CAN_INV_FILTER;
    filter_.can_mask = (uint32_t)filter_can_mask_;
    if (setsockopt(canbus_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_, sizeof(filter_)) < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "Can filter setsockopt failed!");
      return CallbackReturn::FAILURE;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    if (setsockopt(canbus_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0)
    {
      std::cerr << "Can filter setsockopt failed!\n";
      return CallbackReturn::FAILURE;
    }

    memset(&ifr_, 0, sizeof(ifr_));
    snprintf(ifr_.ifr_name, sizeof(ifr_.ifr_name), "%s", interface_name_.c_str());
    if (ioctl(canbus_socket_, SIOCGIFINDEX, &ifr_) < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "ioctl fail! Does can interface 'can0' exist?");
      return CallbackReturn::FAILURE;
    }

    address_.can_family = AF_CAN;
    address_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(canbus_socket_, (struct sockaddr *)&address_, sizeof(address_)) < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "Bind error!");
      return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CanGpioHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> CanGpioHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (Pin &pin : pins_)
    {
      if (pin.direction == PinType::DIGITAL_INPUT)
      {
        interfaces.emplace_back(hardware_interface::StateInterface(pin.name, "input", &(pin.value)));
      }
    }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  CanGpioHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (Pin &pin : pins_)
    {
      if (pin.direction == PinType::DIGITAL_OUTPUT)
      {
        interfaces.emplace_back(hardware_interface::CommandInterface(pin.name, "output", &(pin.value)));
      }
    }
    return interfaces;
  }

  hardware_interface::return_type CanGpioHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type CanGpioHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    struct can_frame frame;
    frame.can_id = filter_can_id_;
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = MESSAGE_SIZE;

    memset(frame.data, 0xFF, MESSAGE_SIZE);
    for (Pin &pin : pins_)
    {
      if (pin.direction == PinType::DIGITAL_OUTPUT)
      {
        const uint8_t byte_index = pin.index / 4;
        const uint8_t bit_index = 8 - 2 * (pin.index % 4);
        if (pin.value > 0.5)
          frame.data[byte_index] &= ~(0b01 << (bit_index - 1));
        else
          frame.data[byte_index] &= ~(0b11 << bit_index);
      }
    }

    if (!::write(canbus_socket_, &frame, sizeof(struct can_frame)))
    {
      RCLCPP_FATAL(rclcpp::get_logger("mep3_hardware"), "Cannot send a CAN message");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }
} // namespace mep3_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mep3_hardware::CanGpioHardwareInterface, hardware_interface::SystemInterface)
