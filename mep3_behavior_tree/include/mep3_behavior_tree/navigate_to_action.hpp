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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_behavior_tree/pose_2d.hpp"
#include "mep3_behavior_tree/table_specific_ports.hpp"
#include "mep3_behavior_tree/team_color_strategy_mirror.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace mep3_behavior_tree
{
  class NavigateToAction
    : public mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
  {
  public:
    explicit NavigateToAction(const std::string &xml_tag_name, const BT::NodeConfiguration &config)
        : mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(
              xml_tag_name, "navigate_to_pose", config)
    {
      if (!getInput("goal", this->goal))
        throw BT::RuntimeError(
          "Navigate action requires 'goal' argument"
        );
      getInput("behavior_tree", this->behavior_tree);

      std::string table = this->config().blackboard->get<std::string>("table");
      BT::Pose2D goal_offset;
      if (table.length() > 0 && getInput("goal_" + table, goal_offset)) {
        goal += goal_offset;
      }

      if (g_StrategyMirror.requires_mirroring(mirror_)) {
        g_StrategyMirror.mirror_pose(goal);
      }
    }

    void on_tick() override;
    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = providedBasicPorts({
        BT::InputPort<BT::Pose2D>("goal"),
        BT::InputPort<std::string>("behavior_tree")
      });

      // Dynamic parameters
      for (std::string table : g_InputPortNameFactory.get_names()) {
        port_list.insert(
          BT::InputPort<BT::Pose2D>("goal_" + table)
        );
      }

      return port_list;
    }

  private:
    BT::Pose2D goal;
    std::string behavior_tree;
  };

  void NavigateToAction::on_tick()
  {
    std::cout << "Navigating to x=" << goal.x \
              << " y=" << goal.y \
              << " θ=" << goal.theta << "°" << std::endl;

    goal_.pose.header.frame_id = "map";
    goal_.pose.header.stamp = node_->get_clock()->now();

    // Position
    goal_.pose.pose.position.x = goal.x;
    goal_.pose.pose.position.y = goal.y;
    goal_.behavior_tree = behavior_tree;

    // Orientation (yaw)
    // Convert deg to rad
    double theta = goal.theta * M_PI / 180.0;
    // https://math.stackexchange.com/a/1972382
    goal_.pose.pose.orientation.w = std::cos(theta / 2.0);
    goal_.pose.pose.orientation.z = std::sin(theta / 2.0);
  }

  BT::NodeStatus NavigateToAction::on_success()
  {
    std::cout << "Navigation succesful" << std::endl;

    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__NAVIGATE_TO_ACTION_HPP_
