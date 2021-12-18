#ifndef MR2_BEHAVIOR_TREES__MOVE_HPP_
#define MR2_BEHAVIOR_TREES__MOVE_HPP_

#include <string>

#include "mep3_planning/bt_action_node.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace BT
{
    struct Pose2D
    {
        double x, y, theta;
    };

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

    class NavigateToAction : public mep3_planning::BtActionNode<nav2_msgs::action::NavigateToPose>
    {
    public:
        explicit NavigateToAction(
            const std::string &name,
            const BT::NodeConfiguration &config)
            : mep3_planning::BtActionNode<nav2_msgs::action::NavigateToPose>(
                  name,
                  config,
                  "navigate_to_pose")
        {
        }

        void on_tick() override;
        BT::NodeStatus on_success() override;

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<BT::Pose2D>("goal")};
        }
    };

    void NavigateToAction::on_tick()
    {
        BT::Pose2D goal;
        getInput("goal", goal);

        goal_.pose.header.frame_id = "map";
        goal_.pose.header.stamp = node_->get_clock()->now();

        // Position
        goal_.pose.pose.position.x = goal.x;
        goal_.pose.pose.position.y = goal.y;

        // Orientation (yaw)
        // https://math.stackexchange.com/questions/1499415/finding-the-quaternion-that-performs-a-rotation
        goal_.pose.pose.orientation.x = 2.0 * std::cos(goal.theta / 2.0);
        goal_.pose.pose.orientation.z = 2.0 * std::sin(goal.theta / 2.0);
    }

    BT::NodeStatus NavigateToAction::on_success()
    {
        std::cout << "Navigation succesful " << std::endl;

        return BT::NodeStatus::SUCCESS;
    }

}

#endif
