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

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_msgs/action/motion_command.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mep3_behavior_tree
{

    class MotionCommandAction
        : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>
    {
    public:
        explicit MotionCommandAction(
            const std::string &name,
            const BT::NodeConfiguration &config) :
            mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>(
                  name,
                  config,
                  "motion_command")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("command"),
                BT::InputPort<_Float64>("value"),
                BT::InputPort<_Float64>("velocity_linear"),
                BT::InputPort<_Float64>("acceleration_linear"),
                BT::InputPort<_Float64>("velocity_angular"),
                BT::InputPort<_Float64>("acceleration_angular"),
                BT::OutputPort<std::string>("result")
            };
        }
    };

    void MotionCommandAction::on_tick()
    {
        std::string command;
        _Float64 value, 
                 velocity_linear,
                 acceleration_linear,
                 velocity_angular,
                 acceleration_angular;

        getInput("command", command);
        getInput("value", value);
        getInput("velocity_linear", velocity_linear);
        getInput("acceleration_linear", acceleration_linear);
        getInput("velocity_angular", velocity_angular);
        getInput("acceleration_angular", acceleration_angular);

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

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__MOTION_COMMAND_ACTION_HPP_
