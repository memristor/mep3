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
#include <iostream>

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_behavior_tree/table_specific_ports.hpp"
#include "mep3_behavior_tree/team_color_strategy_mirror.hpp"
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
                name, "resistance_meter", config
            )
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;
        BT::NodeStatus on_aborted() override;
        BT::NodeStatus on_cancelled() override;

        static BT::PortsList providedPorts()
        {
            // Static parameters
            BT::PortsList port_list = providedBasicPorts({
                BT::InputPort<int32_t>("resistance"),
                BT::InputPort<_Float32>("tolerance"),
            });

            // Dynamic parameters
            for (std::string table : g_InputPortNameFactory.get_names()) {
                port_list.insert(
                    BT::InputPort<int32_t>("resistance_" + table)
                );
            }

            return port_list;
        }
    };

    void ResistanceMeterAction::on_tick()
    {
    }

    BT::NodeStatus ResistanceMeterAction::on_success()
    {
        int32_t measured = result_.result->resistance;

        int32_t expected;
        _Float32 tolerance;
        getInput("resistance", expected);
        getInput("tolerance", tolerance);
        tolerance /= 100.0;

        std::string table = config().blackboard->get<std::string>("table");
        int32_t resistance_offset;
        if (table.length() > 0 && getInput("resistance_" + table, resistance_offset)) {
            expected += resistance_offset;
            std::cout << "Resistance offset for table '" \
                      << table << "' detected" << std::endl;
        }

        g_StrategyMirror.mirror_resistance(expected);

        if (
            measured >= expected * (1.0 - tolerance) && \
            measured <= expected * (1.0 + tolerance)
        ) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus ResistanceMeterAction::on_aborted()
    {
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus ResistanceMeterAction::on_cancelled()
    {
        return BT::NodeStatus::FAILURE;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__RESISTANCE_METER_ACTION_HPP_
