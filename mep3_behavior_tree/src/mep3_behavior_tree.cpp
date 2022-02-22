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

#include <filesystem>
#include <cstdio>

#include "mep3_behavior_tree/dynamixel_command_action.hpp"
#include "mep3_behavior_tree/motion_command_action.hpp"
#include "mep3_behavior_tree/navigate_to_action.hpp"
#include "mep3_behavior_tree/precise_navigate_to_action.hpp"
#include "mep3_behavior_tree/vacuum_pump_command_action.hpp"
#include "mep3_behavior_tree/wait_match_start_topic.hpp"

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

int main(int argc, char **argv)
{
    // Load strategy from file
    if (argc < 2)
    {
        std::cerr << "Error: Missing argument: strategy name" << std::endl;
        return 1;
    }
    auto tree_file =
        (std::filesystem::path(ASSETS_DIRECTORY) / "strategies" / argv[1])
            .replace_extension(".xml");
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Error: Strategy file " << tree_file << " does not exist" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mep3_behavior_tree");
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mep3_behavior_tree::MotionCommandAction>(
        "MotionCommandAction"
    );
    factory.registerNodeType<mep3_behavior_tree::NavigateToAction>(
        "NavigateToAction"
    );
    factory.registerNodeType<mep3_behavior_tree::PreciseNavigateToAction>(
        "PreciseNavigateToAction"
    );
    factory.registerNodeType<mep3_behavior_tree::VacuumPumpCommandAction>(
        "VacuumPumpCommandAction"
    );
    factory.registerNodeType<mep3_behavior_tree::DynamixelCommandAction>(
        "DynamixelCommandAction"
    );
    factory.registerNodeType<mep3_behavior_tree::WaitMatchStart>(
        "WaitMatchStart"
    );

    BT::Tree tree = factory.createTreeFromFile(tree_file, blackboard);
    BT::StdCoutLogger logger_cout(tree);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
