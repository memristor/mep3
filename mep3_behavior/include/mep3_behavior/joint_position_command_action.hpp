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

#ifndef MEP3_BEHAVIOR_TREE__DYNAMIXEL_COMMAND_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__DYNAMIXEL_COMMAND_ACTION_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/table_specific_ports.hpp"
#include "mep3_behavior/team_color_strategy_mirror.hpp"
#include "mep3_msgs/action/joint_position_command.hpp"

namespace mep3_behavior
{
class DynamixelCommandAction
  : public mep3_behavior::BtActionNode<mep3_msgs::action::DynamixelCommand>
{
public:
  explicit DynamixelCommandAction(
    const std::string & xml_tag_name, const BT::NodeConfiguration & config)
  : mep3_behavior::BtActionNode<mep3_msgs::action::DynamixelCommand>(
      xml_tag_name, "joint_position_command", config)
  {
    if (!getInput("position", this->position))
      throw BT::RuntimeError(
        "Dynamixel action requires 'position' argument"
      ); 
    if (!getInput("max_velocity", this->max_velocity))
      this->max_velocity = 99999;
    if (!getInput("max_acceleration", this->max_acceleration))
      this->max_acceleration = 99999;
    if (!getInput("tolerance", this->tolerance))
      this->tolerance = 9;
    if (!getInput("timeout", this->timeout))
      this->timeout = 5;

    std::string table = this->config().blackboard->get<std::string>("table");
    _Float64 position_offset;
    if (table.length() > 0 && getInput("position_" + table, position_offset)) {
      position += position_offset;
    }
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    // Static parameters
    BT::PortsList port_list = providedBasicPorts({
      BT::InputPort<_Float64>("position"),
      BT::InputPort<_Float64>("max_velocity"),
      BT::InputPort<_Float64>("max_acceleration"),
      BT::InputPort<_Float64>("tolerance"),
      BT::InputPort<_Float64>("timeout"),
      BT::OutputPort<int8_t>("result")
    });

    // Dynamic parameters
    for (std::string table : g_InputPortNameFactory.get_names()) {
      port_list.insert(
        BT::InputPort<_Float64>("position_" + table)
      );
    }

    return port_list;
  }

private:
  _Float64 position;
  _Float64 max_velocity;
  _Float64 max_acceleration;
  _Float64 tolerance;
  _Float64 timeout;
};

void DynamixelCommandAction::on_tick()
{
  std::cout << "Set '" << this->action_name_ << "' to " << this->position << "°" <<std::endl;
  goal_.position = position;
  goal_.velocity = velocity;
  goal_.tolerance = tolerance;
  goal_.timeout = timeout;
}

BT::NodeStatus DynamixelCommandAction::on_success()
{
  setOutput("result", 0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DynamixelCommandAction::on_aborted()
{
  setOutput("result", 2);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DynamixelCommandAction::on_cancelled()
{
  setOutput("result", 2);
  return BT::NodeStatus::FAILURE;
}

}  // namespace mep3_behavior

#endif  // MEP3_BEHAVIOR_TREE__DYNAMIXEL_COMMAND_ACTION_HPP_
