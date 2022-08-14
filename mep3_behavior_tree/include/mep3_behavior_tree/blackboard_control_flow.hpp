// Copyright 2022 Memristor Robotics
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

#ifndef MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_
#define MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_

#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"

namespace mep3_behavior_tree
{

class CompareBlackboardControl : public BT::ControlNode
{
public:
  CompareBlackboardControl(
    const std::string& name,
    const BT::NodeConfiguration& config_
  ) : BT::ControlNode(name, config_)
  {
    this->running_child_ = -1;

    // Check number of branches
    if (this->childrenCount() > 2) {
      throw BT::RuntimeError(
        "Too many children nodes for CompareBlackboard"
      );
    }
    if (!getInput("key", this->key) || this->key == "") {
      throw BT::RuntimeError(
        "Blackboard key can't be empty"
      );
    }

    // Default port values
    if (!getInput("operator", this->op)) {
      this->op = "eq";
    }
    if (!getInput("type", this->type)) {
      this->type = "str";
    }
    if (!getInput("value", this->value)) {
      if (this->type == "str")
        this->value = "";
      else
        this->value = "0";
    }

    // Input validation
    if (
      this->type == "str" && \
      (this->op != "eq" && this->op != "ne")
    ) {
      throw BT::RuntimeError(
        "Type 'str' can only be compared using 'eq' and 'ne' operators"
      );
    }
    if (
      this->type != "str" && \
      this->type != "int" && \
      this->type != "float"
    ) {
      throw BT::RuntimeError(
        "Unknown type + '" + type + "'"
      );
    }
    if (
      this->op != "eq" && this->op != "ne" && \
      this->op != "lt" && this->op != "le" && \
      this->op != "gt" && this->op != "ge"
    ) {
      throw BT::RuntimeError(
        "Unknown operator '" + op + "'"
      );
    }
  }

  CompareBlackboardControl() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      // Supported value types for operators:
      //  str    std::string    eq, ne       (default)
      //  int    int64_t        eq, ne, lt, le, gt, ge
      //  float  _Float64       eq, ne, lt, le, gt, ge
      BT::InputPort<std::string>("key"),
      BT::InputPort<std::string>("value"),
      BT::InputPort<std::string>("type"),
      BT::InputPort<std::string>("operator")
    };
  }

  void halt() override
  {
    if(this->running_child_ != -1)
    {
      haltChild(this->running_child_);
      this->running_child_ = -1;
    }
    ControlNode::halt();
  }

  BT::NodeStatus tick() override
  {  
    // Control flow branch
    //  0 : then (statement is true)
    //  1 : else (statement is false)
    size_t branch = 1;

    // Retreive current value from Blackboard
    std::string stored;
    try {
      stored = config().blackboard->get<std::string>(this->key);
    } catch (const BT::RuntimeError& _) {
      // Skip to false branch if key is missing
      goto fallback_to_else_branch;
    }

    // Perform operation
    if (this->type == "str") {
      if (this->op == "eq")
        branch = !(stored == this->value);
      else if (this->op == "ne")
        branch = !(stored != this->value);
    } else {
      _Float64 s, v;
      try {
        if (this->type == "int") {
          s = (_Float64) std::stoi(stored);
          v = (_Float64) std::stoi(this->value);
        } else if (this->type == "float") {
          s = std::stof(stored);
          v = std::stof(this->value);
        }
      } catch (const std::invalid_argument& _) {
        // Skip to false branch if conversion fails
        goto fallback_to_else_branch;
      }
      if (this->op == "eq")
        branch = !(s == v);
      else if (this->op == "ne")
        branch = !(s != v);
      else if (this->op == "lt")
        branch = !(s < v);
      else if (this->op == "le")
        branch = !(s <= v);
      else if (this->op == "gt")
        branch = !(s > v);
      else if (this->op == "ge")
        branch = !(s >= v);
    }

    fallback_to_else_branch:

    // Check if branch has child node
    if (branch >= this->childrenCount()) {
      // Return success if branch is empty
      return BT::NodeStatus::SUCCESS;
    }

    // Tick child
    auto& child_branch = this->children_nodes_[branch];
    BT::NodeStatus child_status = child_branch->executeTick();
    if(child_status == BT::NodeStatus::RUNNING)
    {
      this->running_child_ = branch;
    }
    else
    {
      haltChildren();
      this->running_child_ = -1;
    }
    return child_status;
  }
private:
  ssize_t running_child_;
  std::string op, type, key, value;
};

class BlackboardAction : public BT::SyncActionNode
{
public:
  BlackboardAction(
    const std::string& name,
    const BT::NodeConfiguration& config_
  ) : BT::SyncActionNode(name, config_)
  {
    if (!getInput("key", this->key) || this->key == "") {
      throw BT::RuntimeError(
        "Blackboard key can't be empty"
      );
    }

    // Default port values
    if (!getInput("operator", this->op)) {
      this->op = "set";
    }
    if (!getInput("type", this->type)) {
      this->type = "str";
    }
    if (
      !getInput("value", this->value) && \
      (this->op != "del" && this->op != "print")
    ) {
      throw BT::RuntimeError(
        "Value can't be empty"
      );
    }

    // Input validation
    if (
      this->type == "str" && \
      (this->op != "set" && this->op != "del" && this->op != "print")
    ) {
      throw BT::RuntimeError(
        "Type 'str' can only be modified using 'set' and 'del' operators"
      );
    }
    if (
      this->type != "str" && \
      this->type != "int" && \
      this->type != "float"
    ) {
      throw BT::RuntimeError(
        "Unknown type + '" + type + "'"
      );
    }
    if (
      this->op != "print" && \
      this->op != "set" && this->op != "del" && \
      this->op != "add" && this->op != "sub" && \
      this->op != "mul" && this->op != "div"
    ) {
      throw BT::RuntimeError(
        "Unknown operator '" + op + "'"
      );
    }
  }

  BlackboardAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {
      // Supported value types for operators:
      //  str     std::string   print, set, del
      //  int     int64_t       print, set, del, add, sub, mul, div
      //  float  _Float64       print, set, del, add, sub, mul, div
      BT::InputPort<std::string>("key"),
      BT::InputPort<std::string>("value"),
      BT::InputPort<std::string>("type"),
      BT::InputPort<std::string>("operator")
    };
  }

  BT::NodeStatus tick() override
  {
    // Retreive current value from Blackboard
    std::string stored;
    try {
      stored = config().blackboard->get<std::string>(this->key);
      if (this->op == "print") {
        std::cout << "[BT::Blackboard]: ";
        if (stored.length() > 0)
          std::cout << this->key << " = '" << stored  << "'" << std::endl;
        else
          std::cout << this->key << " is empty" << std::endl;
        return BT::NodeStatus::SUCCESS;
      }
    } catch (const BT::RuntimeError& _) {
      // Return failure if key is missing
      if (this->op == "print") {
        std::cout << "[BT::Blackboard]: " << this->key << " is not set" << std::endl;
        return BT::NodeStatus::SUCCESS;
      } else if (this->op != "set" && this->op != "del") {
        std::cerr << "[Error]: Missing Blackboard key '" + this->key + "'" << std::endl;
        return BT::NodeStatus::FAILURE;
      } else if (this->type == "int" || this->type == "float") {
        // Set to zero so type casting doesn't panic
        stored = "0";
      }
    }

    // Perform operation
    if (this->op == "del") {
      stored = "";
    } else if (this->type == "str" && this->op == "set") {
      stored = this->value;
    } else {
      _Float64 s, v;
      try {
        if (this->type == "int") {
          s = (_Float64) std::stoi(stored);
          v = (_Float64) std::stoi(this->value);
        } else if (this->type == "float") {
          s = std::stof(stored);
          v = std::stof(this->value);
        }
      } catch (const std::invalid_argument& _) {
        // Return failure if conversion fails
        std::cerr << "[Error]: Unable to cast value to " + this->type << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      if (this->op == "set")
        stored = this->value;
      else if (this->op == "del")
        stored = "";
      else if (this->op == "add")
        stored = std::to_string(s + v);
      else if (this->op == "sub")
        stored = std::to_string(s - v);
      else if (this->op == "mul")
        stored = std::to_string(s * v);
      else if (this->op == "div")
        stored = std::to_string(s / v);
    }

    try {
      config().blackboard->set<std::string>(key, stored);
    } catch (const BT::RuntimeError& _) {
      // Return failure if set fails
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }
private:
  std::string op, type, key, value;
};

class PassAction : public BT::SyncActionNode
{
public:
  PassAction(
    const std::string& name,
    const BT::NodeConfiguration& config_
  ) : BT::SyncActionNode(name, config_)
  {
  }

  PassAction() = delete;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace mep3_behavior_tree

#endif // MEP3_BEHAVIOR_TREE__BLACKBOARD_CONTROL_FLOW_HPP_
