#ifndef BEHAVIOR_TREE_ROS_NODE_HPP
#define BEHAVIOR_TREE_ROS_NODE_HPP

#include "rclcpp/rclcpp.hpp"

namespace mep3_planning
{
    /**
     * Stores objects that need be shared between BehaviorTree nodes.
     */
    class BehaviorTreeRosNode : public rclcpp::Node
    {
    public:
        BehaviorTreeRosNode();
    };
}

#endif
