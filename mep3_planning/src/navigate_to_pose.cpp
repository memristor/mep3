#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "mep3_planning/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace mep3_planning
{

    NavigateToPose::NavigateToPose(
        const std::string &xml_tag_name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : mep3_planning::BtActionNode<nav2_msgs::action::NavigateToPose>(
              xml_tag_name, action_name,
              conf)
    {
    }

    void NavigateToPose::on_tick()
    {
        geometry_msgs::msg::PoseStamped goal;
        getInput("goal", goal);

        goal_.pose = goal;
    }

    BT::NodeStatus NavigateToPose::on_success()
    {
        std::cout << "Navigation succesful " << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

}

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<mep3_planning::NavigateToPose>(
            name, "navigate_to_pose", config);
    };

    factory.registerBuilder<mep3_planning::NavigateToPose>(
        "NavigateToPose", builder);
}