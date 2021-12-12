#ifndef INIT_INTERFACE_HPP
#define INIT_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behavior_tree_ros_node.hpp"

namespace mep3_planning
{
    /**
     * The interface allows us to pass a ROS node to a Behavior Tree node.
     * See: https://www.behaviortree.dev/tutorial_08_additional_args/#method-2-use-an-init-method
     */
    class InitInterface
    {
    public:
        virtual void init(BehaviorTreeRosNode::SharedPtr ros_node) = 0;
    };
}

#endif
