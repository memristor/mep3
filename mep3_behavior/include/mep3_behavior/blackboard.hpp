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

#ifndef MEP3_BEHAVIOR_TREE__BLACKBOARD_HPP_
#define MEP3_BEHAVIOR_TREE__BLACKBOARD_HPP_

#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <cstdio>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/task_sequence_control.hpp"
#include "rclcpp/rclcpp.hpp"

namespace BT
{

enum TeamColor{
  BLUE = 0,
  YELLOW = 1
};

class SharedBlackboard {
private:
   SharedBlackboard() = delete;
   inline static std::shared_ptr<Blackboard> blackboard;

public:
    static Blackboard::Ptr create(std::shared_ptr<rclcpp::Node> node)
    {
        SharedBlackboard::blackboard = Blackboard::create();
        SharedBlackboard::blackboard->set("node", node);
        return SharedBlackboard::blackboard;
    }

    static Blackboard::Ptr access() {
        if (SharedBlackboard::blackboard == nullptr)
        {
            throw RuntimeError("Shared BehaviorTree accessed before cretion");
        }
        return SharedBlackboard::blackboard;
    }
};

}  // namespace BT

#endif // MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_
