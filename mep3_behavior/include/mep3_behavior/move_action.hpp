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

#ifndef MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/pose_2d.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/move_command.hpp"

using namespace BT;

namespace mep3_behavior
{
  class TranslateAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    TranslateAction(const std::string &name,
                    const NodeConfig &conf,
                    const ActionNodeParams &params,
                    typename std::shared_ptr<ActionClient> action_client)
        : RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<double>("x"),
              BT::InputPort<std::string>("frame_id"),
              BT::InputPort<bool>("ignore_obstacles"),
              BT::InputPort<int>("reversing"),
              BT::OutputPort<int>("error")};
    }

    bool setGoal(Goal &goal) override
    {
      getInput<double>("x", goal.target.x);
      getInput<std::string>("frame_id", goal.header.frame_id);
      getInput<bool>("ignore_obstacles", goal.ignore_obstacles);

      std::cout << "TranslateAction: setGoal" << std::endl;
      std::cout << "  x: " << goal.target.x << std::endl;
      std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
      std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;

      goal.mode = mep3_msgs::msg::MoveCommand::MODE_TRANSLATE;

      return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->error);

      setOutput<int>("error", wr.result->error);

      return wr.result->error ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
      RCLCPP_ERROR(node_->get_logger(), "%s: onFailure %d", name().c_str(), error);
      return NodeStatus::FAILURE;
    }
  };
} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
