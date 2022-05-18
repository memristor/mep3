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

#ifndef MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_
#define MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior_tree
{

class CompareBlackboard : public BT::ControlNode
{
public:
  CompareBlackboard(
    const std::string& name,
    const BT::NodeConfiguration& config_
  ) : BT::ControlNode(name, config_)
  {
    this->running_child_ = -1;
  }

  CompareBlackboard() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("operator"),
      BT::InputPort<std::string>("output_key"),
      BT::InputPort<std::string>("value")
    };
  }

  void halt() override
  {
    if(this->running_child_ != -1)
    {
      haltChild(running_child_);
      this->running_child_ = -1;
    }
    ControlNode::halt();
  }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
private:
    ssize_t running_child_;
};

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_
