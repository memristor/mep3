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

#ifndef MEP3_BEHAVIOR_TREE__MOTION_COMMAND_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__MOTION_COMMAND_ACTION_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_behavior_tree/table_specific_ports.hpp"
#include "mep3_behavior_tree/team_color_strategy_mirror.hpp"
#include "mep3_msgs/action/motion_command.hpp"

namespace mep3_behavior_tree
{
class MotionCommandAction
  : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>
{
public:
  explicit MotionCommandAction(
    const std::string & xml_tag_name, const BT::NodeConfiguration & config)
  : mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>(
      xml_tag_name, "motion_command", config)
  {
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    // Static parameters
    BT::PortsList port_list = providedBasicPorts({
      BT::InputPort<std::string>("command"),
      BT::InputPort<_Float64>("value"),
      BT::InputPort<_Float64>("velocity_linear"),
      BT::InputPort<_Float64>("acceleration_linear"),
      BT::InputPort<_Float64>("velocity_angular"),
      BT::InputPort<_Float64>("acceleration_angular"),
      BT::OutputPort<std::string>("result")
    });

    // Dynamic parameters
    for (std::string table : g_InputPortNameFactory.get_names()) {
      port_list.insert(
        BT::InputPort<_Float64>("value_" + table)
      );
    }

    return port_list;
  }
};

void MotionCommandAction::on_tick()
{
  std::string command;
  _Float64 value, velocity_linear, acceleration_linear, 
                  velocity_angular, acceleration_angular;

  getInput("command", command);
  getInput("value", value);

  if (!getInput("velocity_linear", velocity_linear))
    velocity_linear = 0;
  if (!getInput("acceleration_linear", acceleration_linear))
    acceleration_linear = 0;
  if (!getInput("velocity_angular", velocity_angular))
    velocity_angular = 0;
  if (!getInput("acceleration_angular", acceleration_angular))
    acceleration_angular = 0;

  std::string table = config().blackboard->get<std::string>("table");
  _Float64 value_offset;
  if (table.length() > 0 && getInput("value_" + table, value_offset)) {
    value += value_offset;
    std::cout << "Motion value offset for table '" \
              << table << "' detected" << std::endl;
  }

  if (command == "rotate_relative") {
    g_StrategyMirror.invert_angle(value);
  } else if (command == "rotate_absolute") {
    g_StrategyMirror.mirror_angle(value);
  }

  goal_.command = command;
  goal_.value = value;
  goal_.velocity_linear = velocity_linear;
  goal_.acceleration_linear = acceleration_linear;
  goal_.velocity_angular = velocity_angular;
  goal_.acceleration_angular = acceleration_angular;
}

BT::NodeStatus MotionCommandAction::on_success()
{
  // TODO(filiparag): return success or drift
  setOutput("result", "success");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mep3_behavior_tree

#endif  // MEP3_BEHAVIOR_TREE__MOTION_COMMAND_ACTION_HPP_
