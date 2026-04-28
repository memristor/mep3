#ifndef OBJECT_DETECTION_SUBSCRIBER_HPP
#define OBJECT_DETECTION_SUBSCRIBER_HPP

#include <functional>
#include <memory>
#include <stdint.h>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/u_int32.hpp"
#include "mep3_msgs/action/camera.hpp"

#define MAX_GROUP 17

namespace mep3_vision
{
    typedef std_msgs::msg::UInt32 table_msg;
    typedef mep3_msgs::action::Camera CameraAction;
    using GoalHandleCamera = rclcpp_action::ServerGoalHandle<CameraAction>;

    class ObjectDetection : public rclcpp::Node{

    public:
        explicit ObjectDetection(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        bool buffer_updated_;
        uint32_t buffer_;
        uint8_t group_select_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<table_msg>::SharedPtr subscription_;
        rclcpp_action::Server<CameraAction>::SharedPtr action_server_;

        void callback(const table_msg::SharedPtr msg);
        void watchdog();

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CameraAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCamera> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleCamera> goal_handle);
        void execute(const std::shared_ptr<GoalHandleCamera> goal_handle);
    };

} // namespace mep3_vision

#endif
