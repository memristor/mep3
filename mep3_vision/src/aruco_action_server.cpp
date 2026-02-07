#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

int main(void)
{
    auto logger = rclcpp::get_logger("rclcpp");
    for (int i = 0; i < 10; ++i)
        RCLCPP_INFO(logger, "IDE HAS");
    return 0;
}