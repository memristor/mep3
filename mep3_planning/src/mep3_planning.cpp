#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "mep3_planning/navigate_to_pose.hpp"
// #include "mep3_planning/behavior_tree_ros_node.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>

// mep3_planning::BehaviorTreeRosNode::BehaviorTreeRosNode() : rclcpp::Node("behavior_tree_ros_node") {}

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

    auto node = rclcpp::Node::make_shared("mep3_planning");

    BT::BehaviorTreeFactory factory;

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromFile(tree_file, blackboard);
    BT::StdCoutLogger logger_cout(tree);

    rclcpp::init(argc, argv);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
