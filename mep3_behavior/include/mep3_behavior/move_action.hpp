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
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/pose_2d.hpp"
#include "mep3_msgs/action/move.hpp"

namespace mep3_behavior
{
  class MoveAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    MoveAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
      if (!getInput("goal", target_pose_))
        throw BT::RuntimeError(
          "Move action requires 'goal' argument");
      if (!getInput("ignore_obstacles", ignore_obstacles_))
              ignore_obstacles_=true;
      if (!getInput("max_velocity", max_velocity_))
        max_velocity_ = 99999;
      if (!getInput("max_acceleration", max_acceleration_))
        max_velocity_ = 99999;
      
      std::string table = this->config().blackboard->get<std::string>("table");
      BT::Pose2D goal_offset;
      if (table.length() > 0 && getInput("goal_" + table, goal_offset)) {
        target_pose_ += goal_offset;
      }

      BT::TeamColor color = this->config().blackboard->get<BT::TeamColor>("color");
      if (color == BT::TeamColor::GREEN) {
        target_pose_ = BT::mirrorPose(target_pose_);
      }
    }

    bool setGoal(Goal &goal) override
    {
      std::cout << "Move to x=" << target_pose_.x \
        << " y=" << target_pose_.y \
        << " θ=" << target_pose_.theta << "°"\
        <<" max_velocity="<<max_velocity_\
        <<" max_acceleration="<<max_acceleration_\
        <<" ignore_obstacles="<<ignore_obstacles_ << std::endl;

      goal.header.frame_id = "map";
      goal.target.x = target_pose_.x;
      goal.target.y = target_pose_.y;
      goal.type = mep3_msgs::action::Move::Goal::TYPE_FULL_NO_REVERSING;
      goal.target.theta = target_pose_.theta / 180.0 * M_PI;
      goal.ignore_obstacles = ignore_obstacles_;
      goal.linear_properties.max_velocity = max_velocity_;
      goal.linear_properties.max_acceleration = max_acceleration_;

      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<std::string>("goal"),
          BT::InputPort<bool>("ignore_obstacles"),
          BT::InputPort<double>("max_velocity"),
          BT::InputPort<double>("max_acceleration")
          };

      // Dynamic parameters
      for (std::string table : BT::SharedBlackboard::access()->get<std::vector<std::string>>("predefined_tables"))
      {
        port_list.insert(
            BT::InputPort<double>("goal_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    BT::Pose2D target_pose_;
    bool ignore_obstacles_;
    double max_velocity_;
    double max_acceleration_;
  };



  class MoveRelativAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    MoveRelativAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
      if (!getInput("goal", target_position_))
        throw BT::RuntimeError(
          "Move action requires 'goal' argument"
        );
      if (!getInput("max_velocity", max_velocity_))
        max_velocity_ = 99999;
       // ala ce robot da leti :)
      if (!getInput("max_acceleration", max_acceleration_))
        max_velocity_ = 99999;

      std::string table = this->config().blackboard->get<std::string>("table");
      double goal_offset;
      if (table.length() > 0 && getInput("goal_" + table, goal_offset)) {
        target_position_ += goal_offset;
      }

    }

    bool setGoal(Goal &goal) override
    {
      std::cout << "MoveRelativ to " << target_position_ \
      << "m max_velocity="<<max_velocity_\
      <<" max_acceleration="<<max_acceleration_<<std::endl;
        

      goal.header.frame_id = "base_link";
      goal.target.x = target_position_;
      goal.type = mep3_msgs::action::Move::Goal::TYPE_TRANSLATE;
      goal.linear_properties.max_velocity = max_velocity_;
      goal.linear_properties.max_acceleration = max_acceleration_;
      goal.ignore_obstacles = true;

      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<double>("goal"),
          BT::InputPort<double>("max_velocity"),
          BT::InputPort<double>("max_acceleration")};

      // Dynamic parameters
      for (std::string table : BT::SharedBlackboard::access()->get<std::vector<std::string>>("predefined_tables"))
      {
        port_list.insert(
            BT::InputPort<double>("goal_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    double target_position_;
    double max_velocity_;
    double max_acceleration_;
  };




class RotateAction
      : public BT::RosActionNode<mep3_msgs::action::Move>
  {
  public:
    RotateAction(const std::string &name,
                               const BT::NodeConfiguration &conf,
                               const BT::ActionNodeParams &params,
                               typename std::shared_ptr<ActionClient> action_client)
        : BT::RosActionNode<mep3_msgs::action::Move>(name, conf, params, action_client)
    {
      if (!getInput("angle", target_angle_))
        throw BT::RuntimeError(
          "Move action requires 'goal' argument"
        );
      if (!getInput("max_velocity", max_velocity_))
        max_velocity_ = 99999;
      
      std::string table = this->config().blackboard->get<std::string>("table");
      double goal_offset;
      if (table.length() > 0 && getInput("angle_" + table, goal_offset)) {
        target_angle_ += goal_offset;
      }

    }

    bool setGoal(Goal &goal) override
    {
      std::cout << "Rotate to θ=" << target_angle_ << "°"\
      << " max_velocity="<<max_velocity_<<std::endl;
        

      goal.header.frame_id = "base_link";
      goal.target.theta = target_angle_ / 180.0 * M_PI;
      goal.type = mep3_msgs::action::Move::Goal::TYPE_ROTATE;
      goal.angular_properties.max_velocity = max_velocity_;
      goal.ignore_obstacles = true;

      return true;
    }

    static BT::PortsList providedPorts()
    {
      // Static parameters
      BT::PortsList port_list = {
          BT::InputPort<double>("angle"),
          BT::InputPort<double>("max_velocity")};

      // Dynamic parameters
      for (std::string table : BT::SharedBlackboard::access()->get<std::vector<std::string>>("predefined_tables"))
      {
        port_list.insert(
            BT::InputPort<double>("angle_" + table));
      }
      return port_list;
    }

    BT::NodeStatus onResultReceived(const WrappedResult & /*wr*/) override
    {
      return BT::NodeStatus::SUCCESS;
    }

  private:
    double target_angle_;
    double max_velocity_;
  };


} // namespace mep3_behavior

#endif // MEP3_BEHAVIOR_TREE__MOVE_ACTION_HPP_
