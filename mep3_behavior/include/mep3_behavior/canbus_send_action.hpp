// Copyright 2022 Memristor Robotics
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

#ifndef MEP3_BEHAVIOR_TREE__CANBUS_SEND_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__CANBUS_SEND_ACTION_HPP_

#include <iostream>
#include <string>

#include "can_msgs/msg/frame.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior
{
class CanbusSendAction : public BT::AsyncActionNode
{
public:
  CanbusSendAction(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::AsyncActionNode(name, config_)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    canbus_pub_ = node_->create_publisher<can_msgs::msg::Frame>(
      "can_send", rclcpp::SystemDefaultsQoS()
    );
  }

  CanbusSendAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message"),
      BT::InputPort<uint32_t>("can_id"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr canbus_pub_;
};

BT::NodeStatus CanbusSendAction::tick()
{
  std::string message;
  uint32_t id;
  getInput("message", message);
  getInput<uint32_t>("can_id", id);

  auto parts = BT::splitString(message.c_str(), ';');
  auto msg = can_msgs::msg::Frame();
  
  if (parts.size() > 0) {
    for (size_t i = 0; i < parts.size(); i++) {
      msg.data[i] = BT::convertFromString<int>(parts[i]);
    }
  }
  
  msg.id = id;
  msg.dlc = parts.size();
  msg.is_extended = true;

  canbus_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mep3_behavior

#endif  // MEP3_BEHAVIOR_TREE__CANBUS_SEND_ACTION_HPP_
