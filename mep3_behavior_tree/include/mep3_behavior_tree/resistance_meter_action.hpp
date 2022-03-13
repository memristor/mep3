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

#ifndef MEP3_BEHAVIOR_TREE__RESISTANCE_METER_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__RESISTANCE_METER_ACTION_HPP_

#include <string>

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_msgs/action/resistance_meter.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mep3_behavior_tree
{

    class ResistanceMeterAction
        : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::ResistanceMeter>
    {
    public:
        explicit ResistanceMeterAction(
            const std::string &name,
            const BT::NodeConfiguration &config) :
            mep3_behavior_tree::BtActionNode<mep3_msgs::action::ResistanceMeter>(
                  name,
                  config,
                  "resistance_meter")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;
        BT::NodeStatus on_aborted() override;
        BT::NodeStatus on_cancelled() override;

        static BT::PortsList providedPorts()
        {
          return providedBasicPorts ({
                BT::InputPort<std::string>("measuring_side"),
                BT::OutputPort<int32_t>("resistance")
            });
        }
    };

    void ResistanceMeterAction::on_tick()
    {
        _Float64 measuring_side;

        getInput("measuring_side", measuring_side);

        goal_.measuring_side = measuring_side;
    }

    BT::NodeStatus ResistanceMeterAction::on_success()
    {
        setOutput("resistance", result_.result->resistance);
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus ResistanceMeterAction::on_aborted()
    {
        setOutput("resistance", 0);
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus ResistanceMeterAction::on_cancelled()
    {
        setOutput("resistance", 0);
        return BT::NodeStatus::FAILURE;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__RESISTANCE_METER_ACTION_HPP_
