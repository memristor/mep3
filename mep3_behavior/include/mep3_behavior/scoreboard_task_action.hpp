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

#ifndef MEP3_BEHAVIOR_TREE__SCOREBOARD_TASK_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__SCOREBOARD_TASK_ACTION_HPP_

#include <iostream>
#include <string>

#include "mep3_msgs/msg/scoreboard.hpp"

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior
{
class ScoreboardTaskAction : public BT::AsyncActionNode
{
public:
  ScoreboardTaskAction(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::AsyncActionNode(name, config_)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    scoreboard_task_pub_ = node_->create_publisher<mep3_msgs::msg::Scoreboard>(
      "/scoreboard", rclcpp::SystemDefaultsQoS()
    );
  }

  ScoreboardTaskAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("task"),
      BT::InputPort<int>("points"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<mep3_msgs::msg::Scoreboard>::SharedPtr scoreboard_task_pub_;
};

BT::NodeStatus ScoreboardTaskAction::tick()
{
  std::string task;
  int points;
  getInput("task", task);
  getInput("points", points);

  auto msg = mep3_msgs::msg::Scoreboard();
  msg.task = task;
  msg.points = points;

  std::cout << "Scoreboard points: " << task \
      << " POINTS = "<< points <<std::endl;
  
  scoreboard_task_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mep3_behavior

#endif  // MEP3_BEHAVIOR_TREE__SCOREBOARD_TASK_ACTION_HPP_
