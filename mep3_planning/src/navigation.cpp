#include "mep3_planning/navigation.hpp"

namespace BT
{
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

MoveBaseAction::MoveBaseAction(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
    rclcpp::NodeOptions opts;
    ros_node = std::make_shared<FibonacciActionClient>(name, opts);
    auto client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(ros_node, "navigate_to_pose");
}

BT::NodeStatus MoveBaseAction::tick()
{

    return BT::NodeStatus::SUCCESS;
}