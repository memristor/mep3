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

#include <sys/types.h>
#include <sys/stat.h>

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

time_t get_last_modification_time()
{
  time_t max_time = 0;
  for (auto const &entry : std::filesystem::directory_iterator(ASSETS_DIRECTORY))
  {
    if (entry.path().extension() == ".xml")
    {
      struct stat result;
      if (stat(entry.path().string().c_str(), &result) == 0)
      {
        if (result.st_mtime > max_time)
          max_time = result.st_mtime;
      }
    }
  }
  return max_time;
}

void start_behavior_tree(rclcpp::Node::SharedPtr node, std::string strategy, BT::Blackboard::Ptr blackboard, time_t last_modification_time, bool should_live_reload)
{
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
  BT::RegisterRosAction<mep3_behavior::TranslateAction>(factory, "Translate", {node, "move", std::chrono::seconds(30)});
  BT::RegisterRosAction<mep3_behavior::RotateAction>(factory, "Rotate", {node, "move", std::chrono::seconds(30)});

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

  using std::filesystem::directory_iterator;
  for (auto const &entry : directory_iterator(ASSETS_DIRECTORY))
    if (entry.path().extension() == ".xml")
      factory.registerBehaviorTreeFromFile(entry.path().string());

  BT::Tree tree = factory.createTree(strategy, blackboard);
  BT::StdCoutLogger logger_cout(tree);

  bool finish = false;
  while (!finish && rclcpp::ok())
  {
    finish = tree.tickOnce() == BT::NodeStatus::SUCCESS;
    tree.sleep(std::chrono::milliseconds(10));
    if (should_live_reload && get_last_modification_time() > last_modification_time)
    {
      tree.haltTree();
      return;
    }
  }
}

int main(int argc, char **argv)
{
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

  // Get parameters
  std::string name(node->get_namespace());
  name = name.replace(name.find("/"), sizeof("/") - 1, "");
  node->declare_parameter<std::string>("strategy", "strategy");
  auto strategy = node->get_parameter("strategy").as_string();
  node->declare_parameter<std::string>("table", "");
  auto table = node->get_parameter("table");
  node->declare_parameter<std::vector<std::string>>("predefined_tables", std::vector<std::string>({}));
  rclcpp::Parameter predefined_tables(
      "predefined_tables",
      std::vector<std::string>({}));
  node->get_parameter("predefined_tables", predefined_tables);
  node->declare_parameter<std::string>("color", "blue");
  BT::TeamColor color = node->get_parameter("color").as_string() == "green" ? BT::TeamColor::GREEN : BT::TeamColor::BLUE;

  // Live reloading
  bool should_live_reload = (strategy.find("live") != std::string::npos);
  if (should_live_reload)
  {
    RCLCPP_WARN(node->get_logger(), "Live reloading is enabled!");
  }

  // Populate blackboard
  blackboard->set("namespace", name);
  blackboard->set("predefined_tables", predefined_tables.as_string_array());
  blackboard->set("color", color);
  blackboard->set("table", table.as_string());

  while (rclcpp::ok())
  {
    start_behavior_tree(node, strategy, blackboard, get_last_modification_time(), should_live_reload);
    if (should_live_reload)
    {
      RCLCPP_WARN(node->get_logger(), "Reloading tree!");

      blackboard->clear();
      blackboard->set("namespace", name);
      blackboard->set("predefined_tables", predefined_tables.as_string_array());
      blackboard->set("color", color);
      blackboard->set("table", table.as_string());
      blackboard->set("node", node);
    }
    else
    {
      break;
    }
  }
  rclcpp::shutdown();
  return 0;
}
