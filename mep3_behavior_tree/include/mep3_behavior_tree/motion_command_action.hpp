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

    class MotionCommandAction : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>
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
                BT::InputPort<mep3_msgs::action::MotionCommand::Goal>("goal")
            };
        }
    };

    void MotionCommandAction::on_tick()
    {
        mep3_msgs::action::MotionCommand::Goal goal;
        getInput("goal", goal);

        goal_.command = goal.command;
        goal_.value = goal.value;
        goal_.velocity_linear = goal.velocity_linear;
        goal_.acceleration_linear = goal.acceleration_linear;
        goal_.velocity_angular = goal.velocity_angular;
        goal_.acceleration_angular = goal.acceleration_angular;
    }

    BT::NodeStatus MotionCommandAction::on_success()
    {
        std::cout << "Navigation succesful " << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__MOTION_COMMAND_ACTION_HPP_
