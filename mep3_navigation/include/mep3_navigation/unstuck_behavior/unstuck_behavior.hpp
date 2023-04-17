#ifndef MEP3_NAVIGATION__UNSTUCK_BEHAVIOR_HPP_
#define MEP3_NAVIGATION__UNSTUCK_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/motion_properties.hpp"
#include "nav2_util/node_utils.hpp"

namespace mep3_navigation
{
  using ActionT = mep3_msgs::action::Move;

  class UnstuckBehavior : public nav2_behaviors::TimedBehavior<ActionT>
  {
  public:
    UnstuckBehavior(double linear_x=0.4, double angular_z=0.4);
    ~UnstuckBehavior();

    virtual nav2_behaviors::Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) final override;
    virtual nav2_behaviors::Status onCycleUpdate() final override;

  protected:
    bool ignore_obstacles_;
    rclcpp::Duration timeout_{0, 0};
    rclcpp::Time end_time_;

    enum class State {NotStuck, FindClosest, GotoClosest};
    State state;

    geometry_msgs::msg::Pose2D orig_pose2d; // origin
    geometry_msgs::msg::Pose2D dest_pose2d; // destination
    geometry_msgs::msg::Pose2D pose2d;
    geometry_msgs::msg::PoseStamped current_pose;
    geometry_msgs::msg::Twist cmd_vel;
  };
}

#endif // MEP3_NAVIGATION__UNSTUCK_BEHAVIOR_HPP_
