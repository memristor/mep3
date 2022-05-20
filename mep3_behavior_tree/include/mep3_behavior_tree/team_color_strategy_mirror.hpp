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

#ifndef MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_
#define MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_

#include <regex>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "mep3_behavior_tree/pose_2d.hpp"

namespace mep3_behavior_tree
{
enum TeamColor {
  Purple,
  Yellow
};

class StrategyMirror {
public:
  StrategyMirror() {
    // Set to default color
    this->color = TeamColor::Purple;
  }

  void set_color(const std::string& color) {
    this->color = StrategyMirror::string_to_color_enum(color);
  }

  void set_default_color(const std::string& color) {
    this->default_color = StrategyMirror::string_to_color_enum(color);
  }

  bool is_color(const std::string color) {
    return this->color == StrategyMirror::string_to_color_enum(color);
  }

  void mirror_pose(BT::Pose2D& pose) {
    if (this->color == this->default_color)
      return;
    pose.x *= -1;
    if (pose.theta >= 0) {
      pose.theta = 180.0 - pose.theta;
    } else {
      pose.theta = -180.0 - pose.theta;
    }
  }

  void remap_server_name(std::string& server_name) {
    if (this->color == this->default_color)
      return;
    const auto re_left = std::regex("left");
    const auto re_right = std::regex("right");
    const auto re_placeholder = std::regex("PLACEHOLDER");
    server_name = std::regex_replace(server_name, re_right, "PLACEHOLDER");
    server_name = std::regex_replace(server_name, re_left, "right");
    server_name = std::regex_replace(server_name, re_placeholder, "left");
  }

  bool server_name_requires_mirroring(std::string server_name) {
    if (this->color == this->default_color)
      return false;
    //const auto re_arm_base = std::regex("arm_[a-z]+_motor_base");
    const auto re_hand = std::regex("hand_[a-z]+_(Dz|G)");
    const auto re_vacuum = std::regex("[a-z]+_(left|right)_connector");
    /* return std::regex_search(server_name, re_arm_base) || \*/
    return       std::regex_search(server_name, re_hand) || \
           std::regex_search(server_name, re_vacuum);
  }

  bool server_name_requires_mirroring1(std::string server_name) {
    if (this->color == this->default_color)
      return false;
    const auto re_arm_base = std::regex("arm_[a-z]+_motor_base");
    return std::regex_search(server_name, re_arm_base);
  }
  
  template<typename Number>
  void mirror_angle(Number& angle, const bool invert) {
    if (this->color == this->default_color)
      return;
    if (invert) {
      angle *= -1;
    } else {
      angle = 180.0 - angle;
    }
  }

  template<typename Number>
  void mirror_resistance(Number& resistance) {
    if (this->color == this->default_color)
      return;
    switch (resistance) {
    case 420:
      resistance = 1750;
      return;
    case 1750:
      resistance = 420;
      return;
    }
  }

private:
  static TeamColor string_to_color_enum(const std::string& color) {
    if (color == "purple") {
      return TeamColor::Purple;
    } else if (color == "yellow") {
      return TeamColor::Yellow;
    } else {
      throw std::invalid_argument("received invalid color");
    }
  }

  TeamColor color;
  TeamColor default_color;
};

// Globally shared singleton
StrategyMirror g_StrategyMirror;

class DefaultTeamColorCondition : public BT::ConditionNode
{
public:
  DefaultTeamColorCondition(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::ConditionNode(name, config_)
  {
  }

  DefaultTeamColorCondition() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("default_color"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string color;
    getInput("default_color", color);

    g_StrategyMirror.set_default_color(color);

    return BT::NodeStatus::SUCCESS;
  }
};

class IfTeamColorThenElseControl : public BT::ControlNode
{
public:
  IfTeamColorThenElseControl(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::ControlNode(name, config_)
  {
    this->running_child_ = -1;
  }

  IfTeamColorThenElseControl() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("color"),
    };
  }

  void halt() override
  {
    if(this->running_child_ != -1)
    {
      haltChild(running_child_);
      this->running_child_ = -1;
    }
    ControlNode::halt();
  }

  BT::NodeStatus tick() override
  {
    std::string color;
    getInput("color", color);
    size_t branch = (g_StrategyMirror.is_color(color)) ? 0 : 1;

    // first child runs if color matches
    // second optional child runs if color doesn't match
    assert(branch < this->childrenCount());

    auto& child_branch = this->children_nodes_[branch];
    BT::NodeStatus child_status = child_branch->executeTick();
    if(child_status == BT::NodeStatus::RUNNING)
    {
      running_child_ = branch;
    }
    else
    {
      haltChildren();
      running_child_ = -1;
    }
    return child_status;
  }
private:
    ssize_t running_child_;
};

}  // namespace mep3_behavior_tree

#endif  // MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_
