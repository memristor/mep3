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

#ifndef MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_

#include <string>
#include <sstream>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/pose_2d.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/move_command.hpp"

using namespace BT;

namespace mep3_behavior
{
  class TranslateAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    TranslateAction(const std::string &name,
                    const NodeConfig &conf,
                    const ActionNodeParams &params,
                    typename std::shared_ptr<ActionClient> action_client)
        : RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<double>("x"),
              BT::InputPort<double>("linear_velocity"),
              BT::InputPort<double>("linear_acceleration"),
              BT::InputPort<std::string>("frame_id"),
              BT::InputPort<bool>("ignore_obstacles"),
              BT::InputPort<bool>("enable_sensors"),
              BT::InputPort<int>("reversing"),
              BT::OutputPort<int>("error")};
    }

    bool setGoal(Goal &goal) override
    {
      getInput<double>("x", goal.target.x);
      getInput<double>("linear_velocity", goal.linear_properties.max_velocity);
      getInput<double>("linear_acceleration", goal.linear_properties.max_acceleration);
      getInput<std::string>("frame_id", goal.header.frame_id);
      getInput<bool>("ignore_obstacles", goal.ignore_obstacles);
      getInput<bool>("enable_sensors", goal.enable_sensors);

      std::cout << "TranslateAction: setGoal" << std::endl;
      std::cout << "  x: " << goal.target.x << std::endl;
      std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
      std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;
      std::cout << "  enable_sensors: " << goal.enable_sensors << std::endl;
      std::cout << "  linear_velocity: " << goal.linear_properties.max_velocity << std::endl;
      std::cout << "  linear_acceleration: " << goal.linear_properties.max_acceleration << std::endl;

      goal.target.theta = 0.0;

      goal.mode = mep3_msgs::msg::MoveCommand::MODE_TRANSLATE;

      return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->error);

      setOutput<int>("error", wr.result->error);

      return wr.result->error ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }
  };

  class RotateAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    RotateAction(const std::string &name,
                 const NodeConfig &conf,
                 const ActionNodeParams &params,
                 typename std::shared_ptr<ActionClient> action_client)
        : RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<double>("angle"),
          BT::InputPort<double>("angular_velocity"),
          BT::InputPort<double>("angular_acceleration"),
          BT::InputPort<std::string>("frame_id"),
          BT::InputPort<bool>("ignore_obstacles"),
          BT::OutputPort<int>("error")};
    }

    bool setGoal(Goal &goal) override
    {
      double yaw_deg;

      getInput<double>("angle", yaw_deg);
      getInput<std::string>("frame_id", goal.header.frame_id);
      getInput<bool>("ignore_obstacles", goal.ignore_obstacles);
      getInput<double>("angular_velocity", goal.angular_properties.max_velocity);
      getInput<double>("angular_acceleration", goal.angular_properties.max_acceleration);
      goal.target.theta = yaw_deg * M_PI / 180.0;
      goal.target.x = 0.0;
      goal.target.y = 0.0;

      std::cout << "RotateAction: setGoal" << std::endl;
      std::cout << "  angle: " << goal.target.theta << std::endl;
      std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
      std::cout << "  angular_velocity: " << goal.angular_properties.max_velocity << std::endl;
      std::cout << "  angular_acceleration: " << goal.angular_properties.max_acceleration << std::endl;
      std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;

      goal.mode = mep3_msgs::msg::MoveCommand::MODE_ROTATE_AT_GOAL;

      return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->error);

      setOutput<int>("error", wr.result->error);

      return wr.result->error ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }
  };

  class MoveAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    MoveAction(const std::string &name,
               const NodeConfig &conf,
               const ActionNodeParams &params,
               typename std::shared_ptr<ActionClient> action_client)
        : RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
    }

    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<std::string>("goal"),
          BT::InputPort<double>("linear_velocity"),
          BT::InputPort<double>("linear_acceleration"),
          BT::InputPort<double>("angular_velocity"),
          BT::InputPort<double>("angular_acceleration"),
          BT::InputPort<std::string>("frame_id"),
          BT::InputPort<bool>("ignore_obstacles"),
          BT::InputPort<int>("mode"),
          BT::InputPort<int>("reversing"),
          BT::OutputPort<int>("error")};
    }

    bool setGoal(Goal &goal) override
    {
      double yaw_deg;
      int mode;
      int reversing;
      std::string position;
      std::string token;

      getInput<std::string>("goal", position);
      getInput<std::string>("frame_id", goal.header.frame_id);
      getInput<bool>("ignore_obstacles", goal.ignore_obstacles);
      getInput<double>("linear_velocity", goal.linear_properties.max_velocity);
      getInput<double>("linear_acceleration", goal.linear_properties.max_acceleration);
      getInput<double>("angular_velocity", goal.angular_properties.max_velocity);
      getInput<double>("angular_velocity", goal.angular_properties.max_acceleration);
      getInput<int>("mode", mode);
      getInput<int>("reversing", reversing);

      std::istringstream iss(position);
      std::getline(iss, token, ';');
      goal.target.x = std::stod(token);

      std::getline(iss, token, ';');
      goal.target.y = std::stod(token);

      std::getline(iss, token, ';');
      goal.target.theta = std::stod(token) * M_PI / 180.0;
      goal.mode = mode;
      goal.reversing = reversing;

      std::cout << "RotateAction: setGoal" << std::endl;
      std::cout << "  x: " << goal.target.x << std::endl;
      std::cout << "  y: " << goal.target.y << std::endl;
      std::cout << "  angle: " << goal.target.theta << std::endl;
      std::cout << "  mode: " << goal.mode << "==" << mode << std::endl;
      std::cout << "  frame_id: " << goal.header.frame_id << std::endl;
      std::cout << "  linear_velocity: " << goal.linear_properties.max_velocity << std::endl;
      std::cout << "  linear_acceleration: " << goal.linear_properties.max_acceleration << std::endl;
      std::cout << "  anguar_velocity: " << goal.angular_properties.max_velocity << std::endl;
      std::cout << "  anguar_acceleration: " << goal.angular_properties.max_acceleration << std::endl;
      std::cout << "  ignore_obstacles: " << goal.ignore_obstacles << std::endl;
      std::cout << "  reversing: " << goal.reversing << std::endl;

      return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      RCLCPP_INFO(node_->get_logger(), "%s: onResultReceived %d", name().c_str(), wr.result->error);

      setOutput<int>("error", wr.result->error);

      return wr.result->error ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
    }
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
