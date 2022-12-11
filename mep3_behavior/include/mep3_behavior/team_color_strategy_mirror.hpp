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
#include <vector>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "mep3_behavior/pose_2d.hpp"

namespace mep3_behavior
{
  enum TeamColor
  {
    Blue,
    Green
  };

  class StrategyMirror
  {
  private:
    StrategyMirror();
    inline static std::atomic<TeamColor> default_color_ = TeamColor::Blue, self_color_ = TeamColor::Blue;

    static bool is_default_color()
    {
      return  StrategyMirror::self_color_ == StrategyMirror::default_color_;
    }

    static TeamColor string_to_color(const std::string &color)
    {
      if (color == "blue")
      {
        return TeamColor::Blue;
      }
      else if (color == "green")
      {
        return TeamColor::Green;
      }
      else
      {
        throw std::invalid_argument("received invalid color");
      }
    }

  public:
    static void set_color(const std::string& color)
    {
      StrategyMirror::self_color_ = StrategyMirror::string_to_color(color);
    }

    static void set_default_color(const std::string& color)
    {
      StrategyMirror::default_color_ = StrategyMirror::string_to_color(color);
    }

    static bool is_color(const std::string& color) {
      return  StrategyMirror::self_color_ == StrategyMirror::string_to_color(color);
    }

    template <typename F, typename T>
    static void conditional_map(F mapping, T &value) {
      // Mirror value using mapping if self color is not default color
      if (!StrategyMirror::is_default_color())
      {
        mapping(value);
      }
    }
    
    static void mirror_pose(BT::Pose2D &pose) {
      StrategyMirror::conditional_map([](auto pose) {
        pose.x *= -1;
        pose.theta = ((pose.theta >= 0) ? 180.0 : -180.0) - pose.theta;
      }, pose);
    }
  };

  class DefaultTeamColorCondition : public BT::ConditionNode
  {
  public:
    DefaultTeamColorCondition(const std::string &name, const BT::NodeConfiguration &config_)
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

      StrategyMirror::set_default_color(color);

      return BT::NodeStatus::SUCCESS;
    }
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__TEAM_COLOR_STRATEGY_MIRROR_HPP_
