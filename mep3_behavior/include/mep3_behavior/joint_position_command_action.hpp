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

#ifndef MEP3_BEHAVIOR_TREE__JOINT_POSITION_COMMAND_ACTION_HPP_
#define MEP3_BEHAVIOR_TREE__JOINT_POSITION_COMMAND_ACTION_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/blackboard.hpp"
#include "mep3_msgs/action/joint_position_command.hpp"

namespace mep3_behavior
{
  class JointPositionCommandAction
      : public BT::RosActionNode<mep3_msgs::action::JointPositionCommand>
  {
  public:
    JointPositionCommandAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::JointPositionCommand>(name, conf, params, action_client)
    {
      if (!getInput("position", position_))
        throw BT::RuntimeError(
            "Dynamixel action requires 'position' argument");
      if (!getInput("max_velocity", max_velocity_))
        max_velocity_ = 99999;
      if (!getInput("command_mode", command_mode_))
        command_mode_ = 0.0;
      if (!getInput("max_acceleration", max_acceleration_))
        max_acceleration_ = 99999;
      if (!getInput("tolerance", tolerance_))
        tolerance_ = 9;
      if (!getInput("timeout", timeout_))
        timeout_ = 5;
      if (!getInput("max_effort", max_effort_))
        max_effort_ = 99999;

      std::string table = this->config().blackboard->get<std::string>("table");
      double position_offset;
      if (table.length() > 0 && getInput("position_" + table, position_offset))
      {
        position_ += position_offset;
      }
    }

    bool setGoal(Goal &goal) override
    {
      std::cout << "Dynamixel desired position to θ=" << position_
                << " max_velocity=" << max_velocity_
                << " max_effort=" << max_effort_
                << " command_mode=" << command_mode_
                << " max_acceleration=" << max_acceleration_ << std::endl;

      goal.position = position_ * M_PI / 180;
      goal.max_velocity = max_velocity_ * M_PI / 180;
      goal.max_acceleration = max_acceleration_ * M_PI / 180;
      goal.tolerance = tolerance_ * M_PI / 180;
      goal.timeout = timeout_;
      goal.max_effort = max_effort_;
      goal.command_mode = command_mode_;
      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<std::string>("instance"),
          BT::InputPort<double>("position"),
          BT::InputPort<double>("max_velocity"),
          BT::InputPort<double>("command_mode"),
          BT::InputPort<double>("max_acceleration"),
          BT::InputPort<double>("max_effort"),
          BT::InputPort<double>("tolerance"),
          BT::InputPort<double>("timeout"),
          BT::OutputPort<double>("feedback_effort"),
          BT::OutputPort<double>("feedback_position"),
          BT::OutputPort<double>("result")};

      // Dynamic parameters
      for (std::string table : BT::SharedBlackboard::access()->get<std::vector<std::string>>("predefined_tables"))
      {
        port_list.insert(
            BT::InputPort<double>("position_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
      setOutput("result", (double)wr.result->result);
      setOutput("feedback_effort", wr.result->last_effort);
      setOutput("feedback_position", wr.result->last_position);
      std::cout << "Last result: " << (double)wr.result->result << "; last effort: " << (double)wr.result->last_effort << "; last position: " << (double)wr.result->last_position << std::endl;

      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFeeback(const std::shared_ptr<const mep3_msgs::action::JointPositionCommand::Feedback> feedback) override
    {

      setOutput("feedback_effort", feedback->effort);
      std::cout << "Max effort: " << feedback->effort << std::endl;
      setOutput("feedback_position", feedback->position);

      return BT::NodeStatus::RUNNING;
    }

  private:
    double position_;
    double max_velocity_;
    double max_acceleration_;
    double tolerance_;
    double timeout_;
    double max_effort_;
    double command_mode_;
  };

} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__JOINT_POSITION_COMMAND_ACTION_HPP_
