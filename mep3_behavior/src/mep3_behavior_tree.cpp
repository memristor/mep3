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

#ifndef ASSETS_DIRECTORY
#define ASSETS_DIRECTORY "mep3_behavior/strategies"
#endif

#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <cstdio>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "mep3_behavior/canbus_send_action.hpp"
#include "mep3_behavior/blackboard.hpp"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/joint_position_command_action.hpp"
#include "mep3_behavior/move_action.hpp"
#include "mep3_behavior/navigate_to_action.hpp"
#include "mep3_behavior/scoreboard_task_action.hpp"
#include "mep3_behavior/task_sequence_control.hpp"
#include "mep3_behavior/pump_action.hpp"
#include "mep3_behavior/wait_match_start_action.hpp"
#include "mep3_behavior/delay_action.hpp"
#include "mep3_behavior/set_shared_blackboard_action.hpp"
#include "mep3_behavior/add_obstacle_action.hpp"
#include "mep3_behavior/remove_obstacle_action.hpp"
#include "rclcpp/rclcpp.hpp"

using KeyValueT = diagnostic_msgs::msg::KeyValue;

int main(int argc, char **argv)
{
  // Load strategy from file
  if (argc < 2)
  {
    std::cerr << "Error: Missing argument: strategy name" << std::endl;
    return 1;
  }

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mep3_behavior");

  // Initialize blackboard
  
  auto blackboard = BT::SharedBlackboard::create(node);

  // Create shared blackboard topic
  auto blackboard_subscription = node->create_subscription<KeyValueT>(
      "/shared_blackboard",
      rclcpp::SystemDefaultsQoS().reliable().transient_local(),
      [blackboard](const KeyValueT::SharedPtr msg)
      {
        blackboard->set(msg->key, msg->value);
      });

  // Set namespace
  std::string name(node->get_namespace());
  name = name.replace(name.find("/"), sizeof("/") - 1, "");
  blackboard->set("namespace", name);

  // Get strategy name
  node->declare_parameter<std::string>("strategy", "strategy");
  auto strategy = node->get_parameter("strategy").as_string();

  auto tree_file_path = (std::filesystem::path(ASSETS_DIRECTORY) / name / strategy).replace_extension(".xml");
  if (!std::filesystem::exists(tree_file_path))
  {
    std::cerr << "Error: Strategy file '" << strategy
              << "' for robot '" << name << "' does not exist" << std::endl;
    std::cerr << "Missing file path: " << tree_file_path << std::endl;
    return 1;
  }

  // Set table
  node->declare_parameter<std::string>("table", "");
  auto table = node->get_parameter("table");
  blackboard->set("table", table.as_string());

  // Get predefined table names
  node->declare_parameter<std::vector<std::string>>("predefined_tables", std::vector<std::string>({}));
  rclcpp::Parameter predefined_tables(
      "predefined_tables",
      std::vector<std::string>({}));
  node->get_parameter("predefined_tables", predefined_tables);
  blackboard->set("predefined_tables", predefined_tables.as_string_array());

  // Set color
  node->declare_parameter<std::string>("color", "blue");
  auto color = node->get_parameter("color");
  if (color.as_string() == "green") {
    blackboard->set("color", BT::TeamColor::GREEN);
  } else {
    blackboard->set("color", BT::TeamColor::BLUE);
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<mep3_behavior::CanbusSendAction>(
      "CanbusSend");
  factory.registerNodeType<mep3_behavior::SetSharedBlackboardAction>(
      "SetSharedBlackboard");
  factory.registerNodeType<mep3_behavior::DelayAction>(
      "Wait");
  BT::RegisterRosAction<mep3_behavior::JointPositionCommandAction>(factory, "JointPosition", {node, "joint_position_command", std::chrono::seconds(30)});
  BT::RegisterRosAction<mep3_behavior::MoveAction>(factory, "Move", {node, "move", std::chrono::seconds(30)});
  BT::RegisterRosAction<mep3_behavior::NavigateToAction>(factory, "Navigate", {node, "navigate_to_pose", std::chrono::seconds(30)});
  BT::RegisterRosAction<mep3_behavior::PumpAction>(factory, "Pump", {node, "pump", std::chrono::seconds(30)});
  factory.registerNodeType<mep3_behavior::ScoreboardTaskAction>(
      "ScoreboardTask");
  factory.registerNodeType<mep3_behavior::WaitMatchStartAction>(
      "WaitMatchStart");
  factory.registerNodeType<mep3_behavior::TaskSequenceControl>(
      "TaskSequence");
  factory.registerNodeType<mep3_behavior::AddObstacleAction>(
      "AddObstacle");
  factory.registerNodeType<mep3_behavior::RemoveObstacleAction>(
      "RemoveObstacle");

  BT::Tree tree = factory.createTreeFromFile(tree_file_path, blackboard);
  BT::StdCoutLogger logger_cout(tree);

  bool finish = false;
  while (!finish && rclcpp::ok())
  {
    finish = tree.tickOnce() == BT::NodeStatus::SUCCESS;
    tree.sleep(std::chrono::milliseconds(20));
  }
  rclcpp::shutdown();
  return 0;
}
