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

#ifndef MEP3_BEHAVIOR_TREE__ADD_OBSTACLE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__ADD_OBSTACLE_ACTION_HPP_

#include <iostream>
#include <string>

#include "mep3_behavior/pose_2d.hpp"
#include "mep3_msgs/msg/temporal_obstacle.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace mep3_behavior
{
  class AddObstacleAction : public BT::AsyncActionNode
  {
  public:
    AddObstacleAction(const std::string &name, const BT::NodeConfiguration &config_)
        : BT::AsyncActionNode(name, config_)
    {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      publisher_ = node_->create_publisher<mep3_msgs::msg::TemporalObstacle>(
          "add_obstacle", rclcpp::QoS(1).reliable());
    }

    AddObstacleAction() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("label"),
          BT::InputPort<std::vector<geometry_msgs::msg::Point>>("polygon")
      };
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<mep3_msgs::msg::TemporalObstacle>::SharedPtr publisher_;
  };

  BT::NodeStatus AddObstacleAction::tick()
  {
    std::string label;
    std::vector<geometry_msgs::msg::Point> polygon;
    getInput("label", label);
    getInput<std::vector<geometry_msgs::msg::Point>>("polygon", polygon);

    mep3_msgs::msg::TemporalObstacle msg;
    msg.label = label;
    msg.polygon = polygon;

    publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__ADD_OBSTACLE_ACTION_HPP_
