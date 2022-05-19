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

#ifndef MEP3_BEHAVIOR_TREE__POSE_TO_FULL_POSE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__POSE_TO_FULL_POSE_ACTION_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mep3_behavior_tree/pose_2d.hpp"

namespace mep3_behavior_tree
{
class PoseToFullPoseAction : public BT::SyncActionNode
{
public:
  PoseToFullPoseAction(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::SyncActionNode(name, config_)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  PoseToFullPoseAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<BT::Pose2D>("pose"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("full_pose"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

BT::NodeStatus PoseToFullPoseAction::tick()
{
  BT::Pose2D pose;
  getInput("pose", pose);

  geometry_msgs::msg::PoseStamped full_pose;
  full_pose.header.frame_id = "map";
  full_pose.header.stamp = node_->get_clock()->now();

  // Position
  full_pose.pose.position.x = pose.x;
  full_pose.pose.position.y = pose.y;

  // Orientation (yaw)
  // Convert deg to rad
  const double theta = pose.theta * M_PI / 180.0;
  // https://math.stackexchange.com/a/1972382
  full_pose.pose.orientation.w = std::cos(theta / 2.0);
  full_pose.pose.orientation.z = std::sin(theta / 2.0);
  setOutput("full_pose", full_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mep3_behavior_tree

#endif  // MEP3_BEHAVIOR_TREE__POSE_TO_FULL_POSE_ACTION_HPP_
