#include "behaviortree_cpp_v3/bt_factory.h"
#include "mep3_planning/navigation.hpp"

#include <cstdio>
#include <filesystem>
#include <iostream>

using namespace BT;

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

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

    BehaviorTreeFactory factory;

    auto tree = factory.createTreeFromFile(tree_file);
    tree.tickRoot();

    return 0;
}
