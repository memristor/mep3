#ifndef MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_
#define MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/motion_properties.hpp"
#include "nav2_util/node_utils.hpp"

namespace mep3_navigation
{
  enum MoveState
  {
    INITIALIZE_ROTATION_TOWARDS_GOAL,
    REGULATE_ROTATION_TOWARDS_GOAL,
    INITIALIZE_TRANSLATION,
    REGULATE_TRANSLATION,
    INITIALIZE_ROTATION_AT_GOAL,
    REGULATE_ROTATION_AT_GOAL
  };

  using ActionT = mep3_msgs::action::Move;

  class MoveBehavior : public nav2_behaviors::TimedBehavior<ActionT>
  {
  public:
    MoveBehavior()
        : nav2_behaviors::TimedBehavior<ActionT>(),
          feedback_(std::make_shared<typename ActionT::Feedback>())
    {
    }

    ~MoveBehavior() = default;

    nav2_behaviors::Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
    {
      global_frame_ = command->header.frame_id;
      odom_frame_ = command->odom_frame;
      ignore_obstacles_ = command->ignore_obstacles;
      timeout_ = command->timeout;
      end_time_ = steady_clock_.now() + timeout_;
      linear_properties_ = command->linear_properties;
      angular_properties_ = command->angular_properties;

      tf2::Transform tf_global_target;
      tf_global_target.setOrigin(tf2::Vector3(command->target.x, command->target.y, 0.0));
      tf_global_target.setRotation(tf2::Quaternion(
          tf2::Vector3(0, 0, 1), command->target.theta));

      geometry_msgs::msg::PoseStamped tf_global_odom_message;
      if (!nav2_util::getCurrentPose(
              tf_global_odom_message, *this->tf_, global_frame_, odom_frame_,
              this->transform_tolerance_))
      {
        RCLCPP_ERROR(this->logger_, "Initial global_frame -> odom_frame_ is not available.");
        return nav2_behaviors::Status::FAILED;
      }
      tf2::Transform tf_global_odom;
      tf2::convert(tf_global_odom_message.pose, tf_global_odom);

      tf2::Transform tf_odom_target = tf_global_target.inverse() * tf_global_odom;
      tf_odom_target_inversed_ = tf_odom_target.inverse();

      return nav2_behaviors::Status::SUCCEEDED;
    }

    nav2_behaviors::Status onCycleUpdate()
    {
      // Timeout
      rclcpp::Duration time_remaining = end_time_ - this->steady_clock_.now();
      if (time_remaining.seconds() < 0.0 && timeout_.seconds() > 0.0)
      {
        this->stopRobot();
        RCLCPP_WARN(
            this->logger_,
            "Exceeded time allowance before reaching the Move goal - Exiting Move");
        return nav2_behaviors::Status::FAILED;
      }

      // Target in the base frame
      geometry_msgs::msg::PoseStamped tf_odom_base_message;
      if (!nav2_util::getCurrentPose(
              tf_odom_base_message, *this->tf_, odom_frame_, robot_base_frame_,
              this->transform_tolerance_))
      {
        RCLCPP_ERROR(this->logger_, "Initial odom_frame -> base frame is not available.");
        return nav2_behaviors::Status::FAILED;
      }
      tf2::Transform tf_odom_base;
      tf2::convert(tf_odom_base_message.pose, tf_odom_base);
      const tf2::Transform tf_base_target = tf_odom_target_inversed_ * tf_odom_base;

      const double final_yaw = tf2::getYaw(tf_base_target.getRotation());
      const double target_x = tf_base_target.getOrigin().x();
      const double target_y = tf_base_target.getOrigin().y();
      const double target_yaw = atan2(target_y, target_x);

      // FSM
      // TODO
      switch (state_)
      {
      case MoveState::INITIALIZE_ROTATION_TOWARDS_GOAL:
        initializeRotation();
        break;
      case MoveState::REGULATE_ROTATION_TOWARDS_GOAL:
        regulateRotation(target_yaw);
        break;
      case MoveState::INITIALIZE_TRANSLATION:
        initializeTranslation();
        break;
      case MoveState::REGULATE_TRANSLATION:
        if (abs(target_x) < linear_properties_.tolerance)
        {
          this->stopRobot();
          state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
        }
        else
        {
          regulateTranslation(target_x, target_y);
        }
        break;
      case MoveState::INITIALIZE_ROTATION_AT_GOAL:
        initializeRotation();
        break;
      case MoveState::REGULATE_ROTATION_AT_GOAL:
        regulateRotation(final_yaw);
        break;
      }

      auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel->linear.y = 0.0;
      cmd_vel->angular.z = 0.0;
      cmd_vel->linear.x = 0.0;

      if (!ignore_obstacles_)
      {
        geometry_msgs::msg::PoseStamped current_pose;
        if (!nav2_util::getCurrentPose(
                current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
                this->transform_tolerance_))
        {
          RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
          return nav2_behaviors::Status::FAILED;
        }

        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x = current_pose.pose.position.x;
        pose2d.y = current_pose.pose.position.y;
        pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

        // TODO: Change distance
        if (!isCollisionFree(0.03, cmd_vel.get(), pose2d))
        {
          this->stopRobot();
          RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
          return nav2_behaviors::Status::FAILED;
        }
      }

      this->vel_pub_->publish(std::move(cmd_vel));

      return nav2_behaviors::Status::RUNNING;
    }

  protected:
    void initializeRotation()
    {
    }

    void regulateRotation(double target_yaw)
    {
    }

    void initializeTranslation()
    {
    }

    void regulateTranslation(double target_x, double target_y)
    {
    }

    void onConfigure() override
    {
      auto node = this->node_.lock();
      if (!node)
      {
        throw std::runtime_error{"Failed to lock node"};
      }

      nav2_util::declare_parameter_if_not_declared(
          node,
          "simulate_ahead_time", rclcpp::ParameterValue(2.0));
      node->get_parameter("simulate_ahead_time", simulate_ahead_time_);
    }

    bool isCollisionFree(
        const double &distance,
        geometry_msgs::msg::Twist *cmd_vel,
        geometry_msgs::msg::Pose2D &pose2d)
    {
      // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
      int cycle_count = 0;
      double sim_position_change;
      const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
      geometry_msgs::msg::Pose2D init_pose = pose2d;
      bool fetch_data = true;

      while (cycle_count < max_cycle_count)
      {
        sim_position_change = cmd_vel->linear.x * (cycle_count / this->cycle_frequency_);
        pose2d.x = init_pose.x + sim_position_change * cos(init_pose.theta);
        pose2d.y = init_pose.y + sim_position_change * sin(init_pose.theta);
        cycle_count++;

        if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data))
        {
          return false;
        }
        fetch_data = false;
      }
      return true;
    }

    inline double normalizeAngle(double angle)
    {
      while (angle > M_PI)
        angle -= 2.0 * M_PI;
      while (angle < -M_PI)
        angle += 2.0 * M_PI;
      return angle;
    }

    typename ActionT::Feedback::SharedPtr feedback_;

    std::string global_frame_;
    std::string odom_frame_;
    tf2::Transform tf_odom_target_inversed_;
    bool ignore_obstacles_;

    mep3_msgs::msg::MotionProperties linear_properties_;
    mep3_msgs::msg::MotionProperties angular_properties_;

    rclcpp::Duration timeout_{0, 0};
    rclcpp::Time end_time_;
    double simulate_ahead_time_;

    MoveState state_;
  };

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_
