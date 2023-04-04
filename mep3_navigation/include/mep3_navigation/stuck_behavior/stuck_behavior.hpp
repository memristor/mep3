#ifndef MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_
#define MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/motion_properties.hpp"
#include "nav2_util/node_utils.hpp"
#include "ruckig/ruckig.hpp"
#include "mep3_navigation/stuck_detector.hpp"

namespace mep3_navigation
{
  using ActionT = mep3_msgs::action::Move;

  class StuckBehavior : public nav2_behaviors::TimedBehavior<ActionT>
  {
  public:
    StuckBehavior();
    ~StuckBehavior();

    virtual nav2_behaviors::Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) final override;

    virtual nav2_behaviors::Status onCycleUpdate() final override;

  protected:
    void initializeTranslation(double diff_x, double diff_y);
    void regulateTranslation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y);

    std::string command_global_frame_;
    std::string odom_frame_;
    tf2::Transform tf_odom_target_;
    bool ignore_obstacles_;

    rclcpp::Duration timeout_{0, 0};
    rclcpp::Time end_time_;
    // double simulate_ahead_distance_;

    mep3_msgs::msg::MotionProperties linear_properties_;
    mep3_msgs::msg::MotionProperties angular_properties_;

    mep3_msgs::msg::MotionProperties default_linear_properties_;
    mep3_msgs::msg::MotionProperties default_angular_properties_;

    ruckig::Ruckig<1> *translation_ruckig_{nullptr};
    ruckig::InputParameter<1> translation_ruckig_input_;
    ruckig::OutputParameter<1> translation_ruckig_output_;
    double translation_last_input_;

    // MoveState state_;
    uint8_t type_;

  };
}

#endif // MEP3_NAVIGATION__STUCK_BEHAVIOR_HPP_
