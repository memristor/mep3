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

#ifndef MEP3_BEHAVIOR_TREE__DISTANCE_ANGLE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__DISTANCE_ANGLE_ACTION_HPP_

#include <string>

#include "mep3_behavior_tree/bt_action_node.hpp"
#include "mep3_msgs/action/motion_command.hpp"
#include "mep3_behavior_tree/pose_2d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace mep3_behavior_tree
{

    class DistanceAngleAction : public mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>
    {
    public:
        explicit DistanceAngleAction(
            const std::string &name,
            const BT::NodeConfiguration &config) :
            mep3_behavior_tree::BtActionNode<mep3_msgs::action::MotionCommand>(
                  name,
                  config,
                  "navigate_to_pose")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<BT::Pose2D>("goal")
            };
        }
    };

    void DistanceAngleAction::on_tick()
    {

    }

    BT::NodeStatus DistanceAngleAction::on_success()
    {
        std::cout << "Navigation succesful " << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__DISTANCE_ANGLE_ACTION_HPP_
