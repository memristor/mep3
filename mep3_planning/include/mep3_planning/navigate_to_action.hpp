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
                  name,
                  config)
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

}

#endif