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

#ifndef MEP3_DRIVER__SOCKETCAN_BRIDGE_HPP_
#define MEP3_DRIVER__SOCKETCAN_BRIDGE_HPP_

extern "C" {
#include <linux/can.h>      // CAN network layer definitions
#include <linux/can/raw.h>  // CAN_RAW_FILTER
#include <net/if.h>         // ifreq
#include <pthread.h>        // POSIX threads, can be switched to realtime
#include <sys/ioctl.h>      // IO control
#include <sys/socket.h>     // socket library
#include <unistd.h>         // read, write to sockets, close
}

#include <cstdint>
#include <string>
#include <thread>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mep3_driver
{
class Bridge : public rclcpp::Node
{
public:
  explicit Bridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Bridge();
  int init(std::string interface_name);
  void halt();

private:
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr write_to_can_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  void canbus_receive_function();
  void write_to_can_callback(const can_msgs::msg::Frame::SharedPtr msg);

  int canbus_socket_;
  struct sockaddr_can address_;
  struct ifreq ifr_;

  struct can_filter filter_;

  std::atomic_bool run_thread_;
  std::thread canbus_receive_thread_;
};

}  // namespace mep3_driver

#endif  // MEP3_DRIVER__SOCKETCAN_BRIDGE_HPP_
