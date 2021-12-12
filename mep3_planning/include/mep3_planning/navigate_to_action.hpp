#ifndef NAVIGATE_TO_ACTION_HPP
#define NAVIGATE_TO_ACTION_HPP

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "mep3_planning/init_interface.hpp"
#include "mep3_planning/behavior_tree_ros_node.hpp"

struct Pose2D
{
    double x, y, theta;
};

class NavigateToAction : public BT::CoroActionNode, public mep3_planning::InitInterface
{
public:
    NavigateToAction(const std::string &name, const BT::NodeConfiguration &config)
        : BT::CoroActionNode(name, config)
    {
    }

    void init(mep3_planning::BehaviorTreeRosNode::SharedPtr ros_node) override
    {
        _ros_node = ros_node;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Pose2D>("goal")};
    }

    // TODO: Improve error handling 
    BT::NodeStatus tick() override
    {
        auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(_ros_node, "navigate_to_pose");
        if (!action_client->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(_ros_node->get_logger(), "Action server not available after waiting");
            return BT::NodeStatus::FAILURE;
        }

        // Take the goal from the InputPort of the Node
        Pose2D goal;
        if (!getInput<Pose2D>("goal", goal))
            throw BT::RuntimeError("missing required input [goal]");

        _aborted = false;

        RCLCPP_INFO(_ros_node->get_logger(), "Sending goal %f %f %f %f", goal.x, goal.y);

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = _ros_node->get_clock()->now();
        goal_msg.pose.pose.position.x = goal.x;
        goal_msg.pose.pose.position.y = goal.y;

        auto goal_handle_future = action_client->async_send_goal(goal_msg);
        if (rclcpp::spin_until_future_complete(_ros_node, goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(_ros_node->get_logger(), "send goal call failed");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle = goal_handle_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(_ros_node->get_logger(), "Goal was rejected by server");
            return BT::NodeStatus::FAILURE;
        }

        auto result_future = action_client->async_get_result(goal_handle);

        RCLCPP_INFO(_ros_node->get_logger(), "Waiting for result");
        if (rclcpp::spin_until_future_complete(_ros_node, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(_ros_node->get_logger(), "get result call failed ");
            return BT::NodeStatus::FAILURE;
        }

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = result_future.get();

        switch (wrapped_result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(_ros_node->get_logger(), "Goal was aborted");
            return BT::NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(_ros_node->get_logger(), "Goal was canceled");
            return BT::NodeStatus::FAILURE;
        default:
            RCLCPP_ERROR(_ros_node->get_logger(), "Unknown result code");
            return BT::NodeStatus::FAILURE;
        }

        if (_aborted)
        {
            RCLCPP_INFO(_ros_node->get_logger(), "MoveBase aborted");
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(_ros_node->get_logger(), "result received");
        return BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override
    {
        _aborted = true;
        CoroActionNode::halt();
    }

private:
    mep3_planning::BehaviorTreeRosNode::SharedPtr _ros_node;
    bool _aborted;
};

#endif
