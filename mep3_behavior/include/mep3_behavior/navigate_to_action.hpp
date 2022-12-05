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

#ifndef MEP3_BEHAVIOR_TREE__NAVIGATE_TO_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__NAVIGATE_TO_ACTION_HPP_

#include <string>
#include <cmath>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/pose_2d.hpp"
#include "mep3_behavior/table_specific_ports.hpp"
#include "mep3_behavior/team_color_strategy_mirror.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace mep3_behavior
{
  class NavigateToAction
    : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
  {
  public:
    NavigateToAction(const std::string &name,
                     const BT::NodeConfiguration &conf,
                     const BT::ActionNodeParams &params,
                     typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params, action_client)
    {
      if (!getInput("goal", target_pose_))
        throw BT::RuntimeError(
          "Navigate action requires 'goal' argument"
        );
      getInput("behavior_tree", behavior_tree_);

      std::string table = this->config().blackboard->get<std::string>("table");
      BT::Pose2D goal_offset;
      if (table.length() > 0 && getInput("goal_" + table, goal_offset)) {
        target_pose_ += goal_offset;
      }

      if (g_StrategyMirror.requires_mirroring()) {
        g_StrategyMirror.mirror_pose(target_pose_);
      }
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
        BT::InputPort<BT::Pose2D>("goal"),
        BT::InputPort<std::string>("behavior_tree")
      };

      // Dynamic parameters
      for (std::string table : g_InputPortNameFactory.get_names()) {
        port_list.insert(
          BT::InputPort<BT::Pose2D>("goal_" + table)
        );
      }

      return port_list;
    }

    bool setGoal(Goal &goal) override
    {
      std::cout << "Navigating to x=" << target_pose_.x \
              << " y=" << target_pose_.y \
              << " θ=" << target_pose_.theta << "°" << std::endl;

      goal.pose.header.frame_id = "map";
      goal.pose.header.stamp = node_->get_clock()->now();

      // Position
      goal.pose.pose.position.x = target_pose_.x;
      goal.pose.pose.position.y = target_pose_.y;
      goal.behavior_tree = behavior_tree_;

      // Orientation (yaw)
      // Convert deg to rad
      double theta = target_pose_.theta * M_PI / 180.0;
      // https://math.stackexchange.com/a/1972382
      goal.pose.pose.orientation.w = std::cos(theta / 2.0);
      goal.pose.pose.orientation.z = std::sin(theta / 2.0);
      return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &/*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Pose2D target_pose_;
    std::string behavior_tree_;
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__NAVIGATE_TO_ACTION_HPP_
