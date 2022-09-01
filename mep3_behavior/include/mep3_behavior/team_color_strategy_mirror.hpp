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

#define RESISTANCE_VALUE_YELLOW 1000
#define RESISTANCE_VALUE_PURPLE 470

#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "mep3_behavior/pose_2d.hpp"

namespace mep3_behavior
{
enum TeamColor {
  Purple,
  Yellow
};

class StrategyMirror {
public:
  StrategyMirror() {
    // Set to default color
    this->default_color = TeamColor::Purple;
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

  bool requires_mirroring() {
      if (this->color == this->default_color)
        return false;
      else
        return true;
  }
  
  template<typename Number>
  void invert_angle(Number& angle) {
    // Constraint by physical servo orientation
    angle = 300.0 - angle;
  }

  template<typename Number>
  void mirror_angle(Number& angle) {
    if (angle >= 0) {
      angle = 180.0 - angle;
    } else {
      angle = -180.0 - angle;
    }
  }

  template<typename Number>
  void mirror_resistance(Number& resistance) {
    switch (resistance) {
    case RESISTANCE_VALUE_YELLOW:
      resistance = RESISTANCE_VALUE_PURPLE;
      return;
    case RESISTANCE_VALUE_PURPLE:
      resistance = RESISTANCE_VALUE_YELLOW;
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


  static std::string strip_server_name(const std::string& full_name) {
    std::string stripped_name = full_name;
    auto separator = full_name.find_last_of("/");
    if (separator != std::string::npos && separator < stripped_name.length() - 1) {
      stripped_name = stripped_name.substr(separator + 1, stripped_name.length() - (separator + 1));
    }
    return stripped_name;
  }

  TeamColor color, default_color;
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
      BT::InputPort<std::string>("color"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string color;
    getInput("color", color);

    g_StrategyMirror.set_default_color(color);

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace mep3_behavior

#endif  // MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_
