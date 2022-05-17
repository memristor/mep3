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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_behavior_tree/pose_2d.hpp"
#include "mep3_behavior_tree/team_color_strategy_mirror.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"

namespace mep3_behavior_tree
{
  class NavigateThroughAction
      : public mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateThroughPoses>
  {
  public:
    explicit NavigateThroughAction(const std::string &xml_tag_name, const BT::NodeConfiguration &config)
        : mep3_behavior_tree::BtActionNode<nav2_msgs::action::NavigateThroughPoses>(
              xml_tag_name, "navigate_through_poses", config)
    {
    }

    void on_tick() override;
    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<BT::Pose2D>("goal"),
          BT::InputPort<std::string>("behavior_tree")};
    }
  };

  void NavigateThroughAction::on_tick()
  {
    std::vector<BT::Pose2D> poses;
    std::string behavior_tree;
    getInput("goal", poses);
    getInput("behavior_tree", behavior_tree);

    goal_.behavior_tree = behavior_tree;
    for (auto &pose : poses)
    {
      g_StrategyMirror.mirror_pose(pose);

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

      goal_.poses.push_back(pose_stamped);
    }
  }

  BT::NodeStatus NavigateThroughAction::on_success()
  {
    std::cout << "Navigation succesful " << std::endl;

    return BT::NodeStatus::SUCCESS;
  }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__NAVIGATE_THROUGH_ACTION_HPP_
