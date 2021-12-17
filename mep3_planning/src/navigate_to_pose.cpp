#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "mep3_planning/navigate_to_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace BT
{
    // Reference: https://www.behaviortree.dev/tutorial_03_generic_ports/
    template <>
    inline Pose2D convertFromString(StringView str)
    {
        // The next line should be removed...
        printf("Converting string: \"%s\"\n", str.data());

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }
}

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
        BT::Pose2D goal;
        getInput("goal", goal);

        goal_.pose.header.frame_id = "map";
        goal_.pose.header.stamp = node_->get_clock()->now();
        goal_.pose.pose.position.x = goal.x;
        goal_.pose.pose.position.y = goal.y;
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
