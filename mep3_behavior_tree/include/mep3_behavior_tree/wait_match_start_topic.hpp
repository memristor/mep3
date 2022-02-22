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

#ifndef MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_HPP_
#define MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace mep3_behavior_tree
{

    class WaitMatchStart : public BT::ConditionNode {
    public:
        WaitMatchStart(
            const std::string& name,
            const BT::NodeConfiguration& config_) : 
        BT::ConditionNode(name, config_) {

            node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

            callback_group_ = node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive,
                false
            );
            callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

            rclcpp::SubscriptionOptions sub_option;
            sub_option.callback_group = callback_group_;
            match_start_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
                "/match_start_status",
                rclcpp::SystemDefaultsQoS(),
                std::bind(&WaitMatchStart::matchStartCallback, this, std::placeholders::_1),
                sub_option
            );
        }

        WaitMatchStart() = delete;

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("state"),
            };
        }

    private:
        
        void matchStartCallback(std_msgs::msg::Int8::SharedPtr msg) {
            match_start_state_ = msg->data;
        };

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr match_start_sub_;
        int8_t match_start_state_;
    
    };

    BT::NodeStatus WaitMatchStart::tick() {
        int8_t desired_state;
        getInput("state", desired_state);

        callback_group_executor_.spin_some();

        if (match_start_state_ == desired_state) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__WAIT_MATCH_START_HPP_
