// Copyright 2024 Memristor Robotics
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

#ifndef MEP3_BEHAVIOR_TREE__QUERY_OPPONENT_CONTROL_HPP_
#define MEP3_BEHAVIOR_TREE__QUERY_OPPONENT_CONTROL_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/control_node.h"
#include "rclcpp/rclcpp.hpp"


namespace mep3_behavior
{
    class QueryOpponentControl : public BT::ControlNode
    {
    public:
        explicit QueryOpponentControl(const std::string &name);

        ~QueryOpponentControl() override = default;

        void halt() override;

    private:
        rclcpp::Node::SharedPtr node_;

        BT::NodeStatus tick() override;
    };

    QueryOpponentControl::QueryOpponentControl(const std::string &name)
        : ControlNode::ControlNode(name, {})
    {
        setRegistrationID("QueryOpponentControl");
        this->node_ = this->config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus QueryOpponentControl::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        return BT::NodeStatus::SUCCESS;
    }

    void QueryOpponentControl::halt()
    {
        BT::ControlNode::halt();
    }
} // namespace mep3_behavior
#endif // MEP3_BEHAVIOR_TREE__QUERY_OPPONENT_CONTROL_HPP_
