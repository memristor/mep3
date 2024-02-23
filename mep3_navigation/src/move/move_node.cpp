#include <thread>
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "move.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Executor> executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto move = std::make_shared<mep3_navigation::Move>("move");
    move->init();

    RCLCPP_INFO(move->get_logger(), "update rate is %d Hz", move->get_update_rate());

    // As here: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
    std::thread move_thread(
        [move]()
        {
            rclcpp::Time end_period = move->now();
            rclcpp::Duration period(std::chrono::nanoseconds(1000000000 / move->get_update_rate()));

            while (rclcpp::ok())
            {
                end_period += period;
                std::this_thread::sleep_for(std::chrono::nanoseconds((end_period - move->now()).nanoseconds()));

                move->update();
            }
        });

    executor->add_node(move);
    executor->spin();
    move_thread.join();
    rclcpp::shutdown();
    return 0;
}
