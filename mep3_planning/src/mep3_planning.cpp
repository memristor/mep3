#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "mep3_planning/navigate_to_action.hpp"
// #include "mep3_planning/behavior_tree_ros_node.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>

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

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mep3_planning");
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<mep3_planning::NavigateToAction>("NavigateToAction");

    BT::Tree tree = factory.createTreeFromFile(tree_file, blackboard);
    BT::StdCoutLogger logger_cout(tree);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
