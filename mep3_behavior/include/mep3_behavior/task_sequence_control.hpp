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

#ifndef MEP3_BEHAVIOR_TREE__TASK_SEQUENCE_CONTROL_HPP_
#define MEP3_BEHAVIOR_TREE__TASK_SEQUENCE_CONTROL_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/control_node.h"

namespace mep3_behavior
{
    class TaskSequenceControl : public BT::ControlNode
    {
    public:
        explicit TaskSequenceControl(const std::string &name);

        ~TaskSequenceControl() override = default;

        void halt() override;

    private:
        std::vector<size_t> tasks_to_finish_;
        size_t active_task_;
        bool initialized_;

        BT::NodeStatus tick() override;
    };

    TaskSequenceControl::TaskSequenceControl(const std::string &name)
        : ControlNode::ControlNode(name, {}), active_task_(0), initialized_(false)
    {
        setRegistrationID("TaskSequenceControl");
    }

    BT::NodeStatus TaskSequenceControl::tick()
    {
        setStatus(BT::NodeStatus::RUNNING);

        if (!initialized_)
        {
            for (size_t i = 0; i < childrenCount(); i++)
                tasks_to_finish_.push_back(i);
            initialized_ = true;
        }

        while (tasks_to_finish_.size() > 0)
        {
            TreeNode *current_child_node = children_nodes_[tasks_to_finish_[active_task_]];
            const BT::NodeStatus child_status = current_child_node->executeTick();

            switch (child_status)
            {
            case BT::NodeStatus::RUNNING:
            {
                return child_status;
            }
            case BT::NodeStatus::FAILURE:
            {
                active_task_++;
                if (active_task_ >= tasks_to_finish_.size())
                    active_task_ = 0;

                return BT::NodeStatus::RUNNING;
            }
            case BT::NodeStatus::SUCCESS:
            {
                tasks_to_finish_.erase(tasks_to_finish_.begin() + active_task_);

                if (active_task_ >= tasks_to_finish_.size())
                    active_task_ = 0;
            }
            break;
            case BT::NodeStatus::IDLE:
            {
                throw BT::LogicError("A child node must never return IDLE");
            }
            case BT::NodeStatus::SKIPPED:
            {
            }
            } // end switch
        }     // end while loop

        // The entire while loop completed. This means that all the children returned SUCCESS.
        if (tasks_to_finish_.size() == 0)
        {
            haltChildren();
            for (size_t i = 0; i < children_nodes_.size(); i++)
                tasks_to_finish_.push_back(i);
        }
        return BT::NodeStatus::SUCCESS;
    }

    void TaskSequenceControl::halt()
    {
        // DO NOT reset current_child_idx_ on halt
        BT::ControlNode::halt();
    }
} // namespace mep3_behavior
#endif // MEP3_BEHAVIOR_TREE__TASK_SEQUENCE_CONTROL_HPP_
