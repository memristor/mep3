#ifndef NAV2_BT_NODES_H
#define NAV2_BT_NODES_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Custom type
struct Pose2D
{
    double x, y, theta;
};

// ovo je za konverziju iz stringa proslijedjenog iz Groot-a
// nisam siguran da bas ovako radi ali probaj

// https://www.behaviortree.dev/tutorial_03_generic_ports/

// Ovo sam uzeo odavde: https://www.behaviortree.dev/tutorial_04_sequence/
class MoveBaseAction : public BT::SyncActionNode
{
public:
    MoveBaseAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
        auto client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Pose2D>("goal")};
    }

    BT::NodeStatus tick() override;

private:
};

BT::NodeStatus MoveBaseAction::tick()
{

    return BT::NodeStatus::SUCCESS;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(MoveBase::MoveBaseAction)

#endif