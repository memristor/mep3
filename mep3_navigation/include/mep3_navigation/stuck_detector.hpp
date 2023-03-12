#ifndef MEP3_NAVIGATION__STUCK_DETECTOR_HPP_
#define MEP3_NAVIGATION__STUCK_DETECTOR_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>

namespace mep3_navigation {
class StuckDetector {
public:
  StuckDetector(rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
    node_ = node;

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1,
        std::bind(&StuckDetector::on_cmd_vel, this, std::placeholders::_1));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1,
        std::bind(&StuckDetector::on_odom, this, std::placeholders::_1));

    last_expected_linear_velocity_time_ = rclcpp::Clock().now();
    last_expected_angular_velocity_time_ = rclcpp::Clock().now();
    last_cmd_vel_time_ = rclcpp::Clock().now();

    can_publisher_ = node_->create_publisher<can_msgs::msg::Frame>("can_send", 1);
  }

  void softstop() {
    can_msgs::msg::Frame msg;
    msg.id = 0x00002000;
    msg.dlc = 1;
    msg.data[0] = 0x11; // CMD_RESET_REGULATORS
    can_publisher_->publish(msg);
    msg.data[0] = 0x16; // CMD_SETPOINTS_DISABLE
    can_publisher_->publish(msg);
  }

  bool is_stuck() {
    if (rclcpp::Clock().now() - last_cmd_vel_time_ >
        cmd_vel_duration_timeout) {
      last_cmd_vel_.linear.x = 0;
      last_cmd_vel_.angular.z = 0;
    }

    if (abs(last_cmd_vel_.linear.x) > 0 &&
        rclcpp::Clock().now() - last_expected_linear_velocity_time_ >
            linear_velocity_time_threshold)
      return true;
    if (abs(last_cmd_vel_.angular.z) > 0 &&
        rclcpp::Clock().now() - last_expected_angular_velocity_time_ >
            angular_velocity_time_threshold)
      return true;
    return false;
  }

  double linear_velocity_threshold{0.005};
  double angular_velocity_threshold{0.005};
  rclcpp::Duration linear_velocity_time_threshold{rclcpp::Duration::from_seconds(0.5)};
  rclcpp::Duration angular_velocity_time_threshold{rclcpp::Duration::from_seconds(0.5)};
  rclcpp::Duration cmd_vel_duration_timeout{rclcpp::Duration::from_seconds(0.1)};
  double linear_moving_average_weight{0.01};
  double angular_moving_average_weight{0.01};

private:
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
    average_linear_velocity_ =
        (1 - linear_moving_average_weight) * average_linear_velocity_ +
        linear_moving_average_weight * msg->twist.twist.linear.x;
    average_angular_velocity_ =
        (1 - angular_moving_average_weight) * average_angular_velocity_ +
        angular_moving_average_weight * msg->twist.twist.angular.z;

    if (abs(last_cmd_vel_.linear.x) < linear_velocity_threshold)
      last_expected_linear_velocity_time_ = rclcpp::Clock().now();
    if (abs(last_cmd_vel_.angular.z) < angular_velocity_threshold)
      last_expected_angular_velocity_time_ = rclcpp::Clock().now();
    if (abs(average_linear_velocity_) > linear_velocity_threshold)
      last_expected_linear_velocity_time_ = rclcpp::Clock().now();
    if (abs(average_angular_velocity_) > angular_velocity_threshold)
      last_expected_angular_velocity_time_ = rclcpp::Clock().now();
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr &msg) {
    last_cmd_vel_ = *msg;
    last_cmd_vel_time_ = rclcpp::Clock().now();
  }

  double average_linear_velocity_{0};
  double average_angular_velocity_{0};

  rclcpp::Time last_expected_linear_velocity_time_;
  rclcpp::Time last_expected_angular_velocity_time_;
  rclcpp::Time last_cmd_vel_time_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
};
} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__STUCK_DETECTOR_HPP_
