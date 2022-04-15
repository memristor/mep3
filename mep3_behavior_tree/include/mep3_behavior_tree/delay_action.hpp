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

#ifndef MEP3_BEHAVIOR_TREE__DELAY_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__DELAY_ACTION_HPP_

#include <iostream>
#include <string>

#include "mep3_msgs/msg/scoreboard.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior_tree
{
  class DelayAction : public BT::AsyncActionNode
  {
  public:
    DelayAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::AsyncActionNode(name, config_), halted_(false)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    DelayAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("duration"),
      };
    }

    void halt() override
    {
      halted_ = true;
    }

  private:
    rclcpp::Node::SharedPtr node_;
    bool halted_;
  };

  BT::NodeStatus DelayAction::tick()
  {
    double duration;
    getInput("duration", duration);
  
    rclcpp::Time start = node_->get_clock()->now();
    while (!halted_ && (node_->get_clock()->now() - start).seconds() < duration)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    halted_ = false;
    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__DELAY_ACTION_HPP_
