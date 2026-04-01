#ifndef MEP3_BEHAVIOR_TREE__CAMERA_DETECTION_HPP
#define MEP3_BEHAVIOR_TREE__CAMERA_DETECTION_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"


#include "mep3_behavior/bt_action_node.hpp"
#include "mep3_behavior/blackboard.hpp"

#include "mep3_msgs/action/camera.hpp"
#include <stdint.h>

using namespace BT;
namespace mep3_behavior
{
  class CameraDetection : public BT::RosActionNode<mep3_msgs::action::Camera>
  {
  public:
    CameraDetection(const std::string &name,
                          const BT::NodeConfig &conf,
                          const BT::ActionNodeParams &params,
                          typename std::shared_ptr<ActionClient> action_client) 
        : RosActionNode<mep3_msgs::action::Camera>(name, conf, params, action_client)
    {
        if(!getInput<std::string>("group_select", group_select_)){
            throw BT::RuntimeError("Missing argument group_select!");
        }
    }

    static BT::PortsList providedPorts(){
        BT::PortsList port_list = {
            BT::InputPort<std::string>("group_select"),
        };

        return port_list;
    }

    bool setGoal(Goal &goal){
        int group = std::stoi(group_select_);

        if (group > 7) {
            throw BT::RuntimeError("Wrong group_select argument, expected range from 0 to 18!");
        }
        goal.group_select = (uint8_t)group;

        std::cout << "ArucoCameta: setGoal" << std::endl;
        std::cout << "  group_select: " << group << std::endl;

        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override
    {
        auto blackboard = BT::SharedBlackboard::access();

        blackboard->set("camera_result", 0);

        camera_result_ = (uint8_t)wr.result->response;
        blackboard->set("camera_result", camera_result_);

        std::cout << "Groot2 recived response: " << (int)camera_result_ << std::endl;
        
        if(camera_result_ > 0){
            return BT::NodeStatus::SUCCESS;
        }else{
            return BT::NodeStatus::FAILURE;
        }
        
    }
    
    virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
        return BT::NodeStatus::FAILURE;
    }

    private:
        std::string group_select_;
        int camera_result_;

  };
};


#endif