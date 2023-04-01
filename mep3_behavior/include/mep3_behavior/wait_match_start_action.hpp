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

#ifndef MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_ACTION_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

namespace mep3_behavior
{
  class WaitMatchStartAction : public BT::ActionNodeBase
  {
  public:
    WaitMatchStartAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::ActionNodeBase(name, config_)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

      match_start_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
          "/match_start_status", rclcpp::SystemDefaultsQoS().reliable().transient_local(),
          std::bind(&WaitMatchStartAction::matchStartCallback, this, std::placeholders::_1));
    }

    WaitMatchStartAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<int>("state"),
      };
    }

    void halt() override
    {
      halted_ = true;
    }

  private:
    void matchStartCallback(std_msgs::msg::Int8::SharedPtr msg) { match_start_state_ = msg->data; }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr match_start_sub_;
    int match_start_state_;
    bool halted_{false};
  };

  BT::NodeStatus WaitMatchStartAction::tick()
  {
    int desired_state;
    getInput("state", desired_state);

    if (halted_)
      return BT::NodeStatus::FAILURE;

    if (match_start_state_ == desired_state)
      return BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node_);

    return BT::NodeStatus::RUNNING;
  }

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_ACTION_HPP_
