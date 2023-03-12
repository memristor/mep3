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

namespace mep3_behavior
{
  class MoveAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    MoveAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
      if (!getInput("goal", target_pose_))
        throw BT::RuntimeError(
          "Move action requires 'goal' argument"
        );

      std::string table = this->config().blackboard->get<std::string>("table");
      BT::Pose2D goal_offset;
      if (table.length() > 0 && getInput("goal_" + table, goal_offset)) {
        target_pose_ += goal_offset;
      }

      BT::TeamColor color = this->config().blackboard->get<BT::TeamColor>("color");
      if (color == BT::TeamColor::GREEN) {
        target_pose_ = BT::mirrorPose(target_pose_);
      }
    }

    bool setGoal(Goal &goal) override
    {
      goal.header.frame_id = "map";
      goal.target.x = target_pose_.x;
      goal.target.y = target_pose_.y;
      goal.target.theta = target_pose_.theta / 180.0 * M_PI;
      goal.ignore_obstacles = true;
      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<std::string>("goal")};

      // Dynamic parameters
      for (std::string table : BT::SharedBlackboard::access()->get<std::vector<std::string>>("predefined_tables"))
      {
        port_list.insert(
            BT::InputPort<double>("goal_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Pose2D target_pose_;
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
