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

#ifndef MEP3_BEHAVIOR_TREE__VACUUM_PUMP_COMMAND_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__VACUUM_PUMP_COMMAND_ACTION_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_msgs/action/vacuum_pump_command.hpp"

namespace mep3_behavior
{
class VacuumPumpCommandAction
  : public mep3_behavior::BtActionNode<mep3_msgs::action::VacuumPumpCommand>
{
public:
  explicit VacuumPumpCommandAction(
    const std::string & xml_tag_name, const BT::NodeConfiguration & config)
  : mep3_behavior::BtActionNode<mep3_msgs::action::VacuumPumpCommand>(
      xml_tag_name, "vacuum_pump_command", config)
  {
    if (!getInput("connect", this->connect))
      throw BT::RuntimeError(
        "VacuumPump action requires 'connect' argument"
      );
  }

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int8_t>("connect"),
      BT::OutputPort<int8_t>("result")
    });
  }

private:
  _Float64 connect;
};

void VacuumPumpCommandAction::on_tick()
{
  std::cout << ((this->connect == 1) ? "C" : "Disc") << "onnecting vacuum pump '" \
            << this->action_name_ << "'" << std::endl;
  goal_.connect = connect;
}

BT::NodeStatus VacuumPumpCommandAction::on_success()
{
  setOutput("result", goal_.connect);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus VacuumPumpCommandAction::on_aborted()
{
  setOutput("result", goal_.connect + 2);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus VacuumPumpCommandAction::on_cancelled()
{
  setOutput("result", 4);
  return BT::NodeStatus::FAILURE;
}

}  // namespace mep3_behavior

#endif  // MEP3_BEHAVIOR_TREE__VACUUM_PUMP_COMMAND_ACTION_HPP_
