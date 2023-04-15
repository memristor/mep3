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

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior
{
  class DelayAction : public BT::ActionNodeBase
  {
  public:
    DelayAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::ActionNodeBase(name, config_), started_(false), halted_(false)
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
    double duration_;
    bool started_;
    bool halted_;
    rclcpp::Time start_time_;
  };

  BT::NodeStatus DelayAction::tick()
  {
    getInput("duration", duration_);
    if (!started_)
    {
      started_ = true;
      start_time_ = node_->get_clock()->now();
    }

    if (halted_)
    {
      started_ = false;
      halted_ = false;
      return BT::NodeStatus::FAILURE;
    }

    if ((node_->get_clock()->now() - start_time_).seconds() >= duration_)
    {
      started_ = false;
      halted_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    rclcpp::spin_some(node_);

    return BT::NodeStatus::RUNNING;
  }

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__DELAY_ACTION_HPP_
