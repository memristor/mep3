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

#ifndef MEP3_BEHAVIOR_TREE__NAVIGATE_THROUGH_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__NAVIGATE_THROUGH_ACTION_HPP_

#include <string>
#include <cmath>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/pose_2d.hpp"
#include "mep3_behavior/table_specific_ports.hpp"
#include "mep3_behavior/team_color_strategy_mirror.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace mep3_behavior
{
  class NavigateThroughAction
      : public BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>
  {
  public:
    NavigateThroughAction(const std::string &name,
                          const BT::NodeConfiguration &conf,
                          const BT::ActionNodeParams &params,
                          typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<nav2_msgs::action::NavigateThroughPoses>(name, conf, params, action_client)
    {
    }

    bool setGoal(Goal &goal) override
    {
      std::vector<BT::Pose2D> poses;
      std::string behavior_tree;
      getInput("goal", poses);
      getInput("behavior_tree", behavior_tree);

      std::string table = config().blackboard->get<std::string>("table");
      std::vector<BT::Pose2D> poses_offset;
      if (table.length() > 0 && getInput("goal_" + table, poses_offset))
      {
        poses += poses_offset;
        std::cout << "Navigation goal offsets for table '"
                  << table << "' detected" << std::endl;
      }

      bool requires_mirroring = g_StrategyMirror.requires_mirroring();

      goal.behavior_tree = behavior_tree;
      for (auto &pose : poses)
      {
        if (requires_mirroring)
        {
          g_StrategyMirror.mirror_pose(pose);
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = node_->get_clock()->now();

        // Position
        pose_stamped.pose.position.x = pose.x;
        pose_stamped.pose.position.y = pose.y;

        // Orientation (yaw)
        // Convert deg to rad
        const double theta = pose.theta * M_PI / 180.0;
        // https://math.stackexchange.com/a/1972382
        pose_stamped.pose.orientation.w = std::cos(theta / 2.0);
        pose_stamped.pose.orientation.z = std::sin(theta / 2.0);

        goal.poses.push_back(pose_stamped);
      }
      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<BT::Pose2D>("goal"),
          BT::InputPort<std::string>("behavior_tree")};

      // Dynamic parameters
      for (std::string table : g_InputPortNameFactory.get_names())
      {
        port_list.insert(
            BT::InputPort<BT::Pose2D>("goal_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__NAVIGATE_THROUGH_ACTION_HPP_
