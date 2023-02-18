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

#ifndef MEP3_HARDWARE__CAN_GPIO_HARDWARE_INTERFACE_HPP_
#define MEP3_HARDWARE__CAN_GPIO_HARDWARE_INTERFACE_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

extern "C" {
#include <linux/can.h>      // CAN network layer definitions
#include <linux/can/raw.h>  // CAN_RAW_FILTER
#include <net/if.h>         // ifreq
#include <pthread.h>        // POSIX threads, can be switched to realtime
#include <sys/ioctl.h>      // IO control
#include <sys/socket.h>     // socket library
#include <unistd.h>         // read, write to sockets, close
}

namespace mep3_hardware
{
  enum PinDirection {
    INPUT = 0,
    OUTPUT
  };

  struct Pin
  {
    double value;
    uint8_t index;
    PinDirection direction;
    std::string name;
  };

  class CanGpioHardwareInterface : public hardware_interface::SystemInterface
  {
  public:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    std::vector<Pin> pins_;

    std::string interface_name_;
    int64_t filter_can_id_;
    int64_t filter_can_mask_;

    int canbus_socket_;
    struct sockaddr_can address_;
    struct ifreq ifr_;
    struct can_filter filter_;
  };
} // namespace mep3_hardware

#endif // MEP3_HARDWARE__CAN_GPIO_HARDWARE_INTERFACE_HPP_
