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

#ifndef MEP3_BEHAVIOR_TREE__REMOVE_OBSTACLE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__REMOVE_OBSTACLE_ACTION_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mep3_behavior
{
  class RemoveObstacleAction : public BT::AsyncActionNode
  {
  public:
    RemoveObstacleAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::AsyncActionNode(name, config_)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      publisher_ = node_->create_publisher<std_msgs::msg::String>(
          "remove_obstacle", rclcpp::QoS(1).reliable());
    }

    RemoveObstacleAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("label"),
      };
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  };

  BT::NodeStatus RemoveObstacleAction::tick()
  {
    std::string label;
    getInput("label", label);

    std_msgs::msg::String msg;
    msg.data = label;

    publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__REMOVE_OBSTACLE_ACTION_HPP_
