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

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_msgs/action/vaccuum_pump_command.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mep3_behavior_tree
{

    class VacuumCommandAction
        : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::VaccuumPumpCommand>
    {
    public:
        explicit VacuumCommandAction(
            const std::string &name,
            const BT::NodeConfiguration &config) :
            mep3_behavior_tree::BtActionNode<mep3_msgs::action::VaccuumPumpCommand>(
                  name,
                  config,
                  "vaccuum_pump_command")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;
        BT::NodeStatus on_aborted() override;
        BT::NodeStatus on_cancelled() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int8_t>("connect"),
                BT::OutputPort<int8_t>("result")
            };
        }
    };

    void VacuumCommandAction::on_tick()
    {
        _Float64 connect;

        getInput("connect", connect);

        goal_.connect = connect;
    }

    BT::NodeStatus VacuumCommandAction::on_success()
    {
        setOutput("result", goal_.connect);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus VacuumCommandAction::on_aborted()
    {
        setOutput("result", goal_.connect + 2);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus VacuumCommandAction::on_cancelled()
    {
        setOutput("result", 4);
        return BT::NodeStatus::FAILURE;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__VACUUM_PUMP_COMMAND_ACTION_HPP_
