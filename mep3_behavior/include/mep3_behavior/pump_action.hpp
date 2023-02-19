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

#ifndef MEP3_BEHAVIOR_TREE__PUMP_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__PUMP_ACTION_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/blackboard.hpp"
#include "mep3_msgs/action/vacuum_pump_command.hpp"

namespace mep3_behavior
{
  class PumpAction
      : public BT::RosActionNode<mep3_msgs::action::VacuumPumpCommand>
  {
  public:
    PumpAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::VacuumPumpCommand>(name, conf, params, action_client)
    {
      if (!getInput("connect", connect_))
        throw BT::RuntimeError(
            "Pump action requires 'connect' argument");
    }

    bool setGoal(Goal &goal) override
    {
      goal.connect = connect_;
      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<std::string>("instance"),
          BT::InputPort<int>("connect"),
          BT::OutputPort<int>("result")};
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    int connect_;
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__PUMP_ACTION_HPP_
