#include "rclcpp/rclcpp.hpp"

int main(void)
{
    auto logger = rclcpp::get_logger("rclcpp");
    for (int i = 0; i < 10; ++i)
        RCLCPP_INFO(logger, "IDE GAS");
    return 0;
}