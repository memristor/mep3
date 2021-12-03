#ifndef DISTANCE_ANGLE_REGULATOR_HPP
#define DISTANCE_ANGLE_REGULATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"

#include "mep3_msgs/msg/motion_command.hpp"
#include "mep3_navigation/distance_angle/motion_profile.hpp"

extern "C"
{
#include <mep3_navigation/distance_angle/pid_regulator.h>
}

using std::placeholders::_1;

class DistanceAngleRegulator : public rclcpp::Node
{
public:
    DistanceAngleRegulator();
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void command_callback(const mep3_msgs::msg::MotionCommand::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<mep3_msgs::msg::MotionCommand>::SharedPtr command_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    pid_regulator_t regulator_distance_;
    pid_regulator_t regulator_angle_;
    double robot_distance_;
    double robot_angle_;
    double prev_robot_x_;
    double prev_robot_y_;
    bool position_initialized_;
    bool debug_;

    MotionProfile distance_profile_;
    MotionProfile angle_profile_;

    double goal_distance_;
    OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    double angle_normalize(double angle);
    void forward(double distance);
    void rotate_relative(double angle);
    void rotate_absolute(double angle);
};

#endif
