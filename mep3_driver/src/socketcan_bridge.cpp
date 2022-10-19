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

#include "mep3_driver/socketcan_bridge.hpp"

#include <memory>
#include <string>

using std::placeholders::_1;

namespace mep3_driver
{
Bridge::Bridge(const rclcpp::NodeOptions & options) : Node("socketcan_bridge", options)
{
  write_to_can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_send", 100, std::bind(&Bridge::write_to_can_callback, this, _1));
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_receive", 100);

  this->declare_parameter("interface_name", "can0");
  this->declare_parameter("filter_can_id_invert", (int64_t)0x80000200);
  this->declare_parameter("filter_can_mask_invert", (int64_t)0x1FFFFFF8);

  std::string interface_name;
  this->get_parameter("interface_name", interface_name);

  int result = init(interface_name);

  if (result != 0) {
    rclcpp::shutdown();
    return;
  }

  run_thread_ = true;
  canbus_receive_thread_ = std::thread(&Bridge::canbus_receive_function, this);
}

Bridge::~Bridge()
{
  close(canbus_socket_);
  run_thread_ = false;
  canbus_receive_thread_.join();
}

void Bridge::canbus_receive_function()
{
  while (run_thread_) {
    struct can_frame frame;
    const int nbytes = read(canbus_socket_, &frame, sizeof(struct can_frame));

    if (nbytes > 0) {
      if ((frame.can_id & CAN_ERR_FLAG) == 0) {
        can_msgs::msg::Frame msg;
        msg.id = frame.can_id & CAN_EFF_MASK;
        msg.dlc = frame.can_dlc;
        for (int i = 0; i < frame.can_dlc; i++) {
          msg.data[i] = frame.data[i];
        }
        msg.is_error = false;
        msg.is_extended = frame.can_id & CAN_EFF_FLAG;
        msg.is_rtr = frame.can_id & CAN_RTR_FLAG;

        msg.header.stamp = this->get_clock()->now();

        can_publisher_->publish(msg);
      }
    }
  }
  return;
}

void Bridge::write_to_can_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  struct can_frame frame;
  frame.can_id = msg->id;
  frame.can_id |= CAN_EFF_FLAG;
  frame.can_dlc = msg->dlc;
  for (int i = 0; i < msg->dlc; i++) {
    frame.data[i] = msg->data[i];
  }

  write(canbus_socket_, &frame, sizeof(struct can_frame));
}

int Bridge::init(std::string interface_name)
{
  if ((canbus_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW_FILTER)) < 0) {
    std::cerr << "Socket error!\n";
    return 1;
  }

  int64_t filter_can_id;
  int64_t filter_can_mask;
  this->get_parameter("filter_can_id_invert", filter_can_id);
  this->get_parameter("filter_can_mask_invert", filter_can_mask);

  filter_.can_id = (uint32_t)filter_can_id;
  filter_.can_id |= CAN_INV_FILTER;
  filter_.can_mask = (uint32_t)filter_can_mask;

  if (setsockopt(canbus_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter_, sizeof(filter_)) < 0) {
    std::cerr << "Can filter setsockopt failed!\n";
    return 1;
  }

  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 50000;

  if (setsockopt(canbus_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    std::cerr << "Can filter setsockopt failed!\n";
    return 1;
  }

  memset(&ifr_, 0, sizeof(ifr_));
  snprintf(ifr_.ifr_name, sizeof(ifr_.ifr_name), "%s", interface_name.c_str());
  if (ioctl(canbus_socket_, SIOCGIFINDEX, &ifr_) < 0) {
    std::cerr << "ioctl fail! Does can interface 'can0' exist?\n";
    return 1;
  }

  address_.can_family = AF_CAN;
  address_.can_ifindex = ifr_.ifr_ifindex;

  if (bind(canbus_socket_, (struct sockaddr *)&address_, sizeof(address_)) < 0) {
    std::cerr << "Bind error!\n";
    return 1;
  }

  return 0;
}

}  // namespace mep3_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mep3_driver::Bridge>());
  rclcpp::shutdown();

  return 0;
}
