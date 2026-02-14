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

#ifndef MEP3_BEHAVIOR_TREE__ARUCO_DETECTION_HPP
#define MEP3_BEHAVIOR_TREE__ARUCO_DETECTION_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/blackboard.hpp"

#include "mep3_msgs/action/aruco.hpp"

using ArucoAction = mep3_msgs::action::Aruco;

namespace mep3_behavior{
    class ArucoDetectionAction : public BT::RosActionNode<mep3_msgs::action::Aruco>
    {
    public:
        ArucoDetectionAction(const std::string &name,
                             const BT::NodeConfig &conf,
                             const BT::ActionNodeParams &params,
                             typename std::shared_ptr<ActionClient> action_client) 
            : RosActionNode<mep3_msgs::action::Aruco>(name, conf, params, action_client)
        {
            if(!getInput<std::string>("camera_select", camera_select_)){
                throw BT::RuntimeError("Missing argument camera_select!");
            }
        }

        static BT::PortsList providedPorts(){
            BT::PortsList port_list = {
                BT::InputPort<std::string>("camera_select"),
            };

            return port_list;
        }

        bool setGoal(Goal &goal){
            auto blackboard = BT::SharedBlackboard::access();
            if(!blackboard->get("color", color_)){
                throw BT::RuntimeError("Missing color argument!");
            }

            if (camera_select_ != "front" && camera_select_ != "back") {
                throw BT::RuntimeError("Wrong camera_select argument, expected format: \"front\" or \"back\"!");
            }
            goal.camera_select = camera_select_;

            if(color_ == BT::TeamColor::BLUE){
                goal.color = "blue";
            }else if(color_ == BT::TeamColor::YELLOW){
                goal.color = "yellow";
            }

            std::cout << "ArucoAction: setGoal" << std::endl;
            std::cout << "  camera_select: " << goal.camera_select << std::endl;
            std::cout << "  color: " << goal.color << std::endl;

            return true;
        }

        BT::NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            auto blackboard = BT::SharedBlackboard::access();
            aruco_mask_ = (int)wr.result->result_mask;
            blackboard->set("aruco_mask", aruco_mask_);

            uint8_t err_mask = wr.result->result_mask & 0xf0;
            return err_mask ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }
        
        virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
            return BT::NodeStatus::FAILURE;
        }


    private:
        std::string camera_select_;
        BT::TeamColor color_;
        int aruco_mask_;

    };
}   // namespace mep3_behavior



#endif