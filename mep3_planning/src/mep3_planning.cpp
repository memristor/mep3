#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "mep3_planning/navigate_to_action.hpp"
#include "mep3_planning/behavior_tree_ros_node.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>

mep3_planning::BehaviorTreeRosNode::BehaviorTreeRosNode() : rclcpp::Node("behavior_tree_ros_node") {}

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

int main(int argc, char **argv)
{
    // Load strategy from file
    if (argc < 2)
    {
        std::cerr << "Error: Missing argument: strategy name" << std::endl;
        return 1;
    }
    auto tree_file = (std::filesystem::path(ASSETS_DIRECTORY) / "strategies" / argv[1]).replace_extension(".xml");
    if (!std::filesystem::exists(tree_file))
    {
        std::cerr << "Error: Strategy file " << tree_file << " does not exist" << std::endl;
        return 1;
    }

    // Create a behavior tree
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<NavigateToAction>("NavigateToAction");
    auto tree = factory.createTreeFromFile(tree_file);
    BT::StdCoutLogger logger_cout(tree);

    // Pass the ROS node to Behavior Tree nodes
    rclcpp::init(argc, argv);
    mep3_planning::BehaviorTreeRosNode::SharedPtr behavior_tree_ros_node = std::make_shared<mep3_planning::BehaviorTreeRosNode>();
    for (auto &node : tree.nodes)
        if (mep3_planning::InitInterface *behavior_tree_node = dynamic_cast<mep3_planning::InitInterface *>(node.get()))
            behavior_tree_node->init(behavior_tree_ros_node);
    tree.tickRoot();
    rclcpp::shutdown();

    return 0;
}
