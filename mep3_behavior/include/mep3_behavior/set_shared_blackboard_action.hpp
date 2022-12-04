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

#ifndef MEP3_BEHAVIOR_TREE__SET_SHARED_BLACKBOARD_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__SET_SHARED_BLACKBOARD_ACTION_HPP_

#include <iostream>
#include <string>

#include "diagnostic_msgs/msg/key_value.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior
{

  using KeyValueT = diagnostic_msgs::msg::KeyValue;

  class SetSharedBlackboardAction : public BT::AsyncActionNode
  {
  public:
    SetSharedBlackboardAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::AsyncActionNode(name, config_)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      blackboard_publisher_ = node_->create_publisher<KeyValueT>(
          "/shared_blackboard", rclcpp::SystemDefaultsQoS().reliable().transient_local());
    }

    SetSharedBlackboardAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("output_key"),
          BT::InputPort<std::string>("value")};
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<KeyValueT>::SharedPtr blackboard_publisher_;
  };

  BT::NodeStatus SetSharedBlackboardAction::tick()
  {
    std::string output_key;
    std::string value;
    getInput("output_key", output_key);
    getInput("value", value);

    auto msg = KeyValueT();
    msg.key = output_key;
    msg.value = value;
    blackboard_publisher_->publish(msg);
    config().blackboard->set(output_key, value);

    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__SET_SHARED_BLACKBOARD_ACTION_HPP_
