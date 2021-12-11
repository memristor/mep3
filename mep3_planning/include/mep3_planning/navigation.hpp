#ifndef NAV2_BT_NODES_H
#define NAV2_BT_NODES_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

struct Pose2D
{
    double x, y, theta;
};

class FibonacciActionClient : public rclcpp::Node
{
public:
    using Fibonacci = nav2_msgs::action::NavigateToPose;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit FibonacciActionClient(const std::string &name, const rclcpp::NodeOptions &options)
        : Node(name, options)
    {
    }

    void send_goal()
    {
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
    {
    }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
    }
};

class MoveBaseAction : public BT::SyncActionNode
{
public:
    MoveBaseAction(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList
    providedPorts()
    {
        return {BT::InputPort<Pose2D>("goal")};
    }

    BT::NodeStatus tick() override;

    // This overloaded method is used to stop the execution of this node.
    // void halt() override
    // {
    //     _halt_requested.store(true);
    // }

private:
    FibonacciActionClient::SharedPtr ros_node;
    std::atomic_bool _halt_requested;
};

#endif