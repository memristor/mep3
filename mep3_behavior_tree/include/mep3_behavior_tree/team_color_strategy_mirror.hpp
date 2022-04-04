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

  void mirror_pose(BT::Pose2D& pose) {
    pose.x *= -1;
    pose.theta *= -1;
  }

  void remap_server_name(std::string& server_name) {
    const auto re_left = std::regex("left");
    const auto re_right = std::regex("right");
    const auto re_placeholder = std::regex("PLACEHOLDER");
    server_name = std::regex_replace(server_name, re_right, "PLACEHOLDER");
    server_name = std::regex_replace(server_name, re_left, "right");
    server_name = std::regex_replace(server_name, re_placeholder, "left");
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

// Globally shared sinleton
StrategyMirror g_StrategyMirror;

class DefaultTeamColorAction : public BT::SyncActionNode
{
public:
  DefaultTeamColorAction(const std::string & name, const BT::NodeConfiguration & config_)
  : BT::SyncActionNode(name, config_)
  {
  }

  DefaultTeamColorAction() = delete;

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

}  // namespace mep3_behavior_tree

#endif  // MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_
