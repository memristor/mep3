#ifndef MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_
#define MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "mep3_msgs/action/move.hpp"
#include "mep3_msgs/msg/motion_properties.hpp"
#include "nav2_util/node_utils.hpp"
#include "ruckig/ruckig.hpp"

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
      type_ = command->type;

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

      tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;

      switch (type_)
      {
      case mep3_msgs::action::Move::Goal::TYPE_ROTATE:
        state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
        break;
      case mep3_msgs::action::Move::Goal::TYPE_TRANSLATE:
        state_ = MoveState::INITIALIZE_TRANSLATION;
        break;
      case mep3_msgs::action::Move::Goal::TYPE_FULL:
      case mep3_msgs::action::Move::Goal::TYPE_SKIP_FINAL_ROTATION:
        state_ = MoveState::INITIALIZE_ROTATION_TOWARDS_GOAL;
        break;
      }
      return nav2_behaviors::Status::SUCCEEDED;
    }

    nav2_behaviors::Status onCycleUpdate()
    {
      // Timeout
      rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
      if (time_remaining.seconds() < 0.0 && timeout_.seconds() > 0.0)
      {
        stopRobot();
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
      const tf2::Transform tf_base_target = tf_odom_base.inverse() * tf_odom_target_;

      const double final_yaw = tf2::getYaw(tf_base_target.getRotation());
      const double diff_x = tf_base_target.getOrigin().x();
      const double diff_y = tf_base_target.getOrigin().y();
      const double diff_yaw = atan2(diff_y, diff_x);

      // FSM
      // TODO
      auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
      switch (state_)
      {
      case MoveState::INITIALIZE_ROTATION_TOWARDS_GOAL:
        initializeRotation(diff_yaw);
        regulateRotation(cmd_vel.get(), diff_yaw);
        state_ = MoveState::REGULATE_ROTATION_TOWARDS_GOAL;
        break;
      case MoveState::REGULATE_ROTATION_TOWARDS_GOAL:
        regulateRotation(cmd_vel.get(), diff_yaw);
        if (abs(diff_yaw) < angular_properties_.tolerance)
        {
          debouncing_counter_++;
          if (debouncing_counter_ >= debouncing_counter_max_)
          {
            stopRobot();
            state_ = MoveState::INITIALIZE_TRANSLATION;
            debouncing_counter_ = 0;
          }
        }
        else
        {
          debouncing_counter_ = 0;
        }
        break;
      case MoveState::INITIALIZE_TRANSLATION:
        initializeTranslation(diff_x, diff_y);
        state_ = MoveState::REGULATE_TRANSLATION;
        break;
      case MoveState::REGULATE_TRANSLATION:
        regulateTranslation(cmd_vel.get(), diff_x, diff_y);
        if (abs(diff_x) < linear_properties_.tolerance)
        {
          debouncing_counter_++;
          if (debouncing_counter_ >= debouncing_counter_max_)
          {
            stopRobot();
            if (type_ == mep3_msgs::action::Move::Goal::TYPE_SKIP_FINAL_ROTATION ||
                type_ == mep3_msgs::action::Move::Goal::TYPE_TRANSLATE)
              return nav2_behaviors::Status::SUCCEEDED;
            state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;
            debouncing_counter_ = 0;
          }
        }
        else
        {
          debouncing_counter_ = 0;
        }
        break;
      case MoveState::INITIALIZE_ROTATION_AT_GOAL:
        initializeRotation(final_yaw);
        regulateRotation(cmd_vel.get(), final_yaw);
        state_ = MoveState::REGULATE_ROTATION_AT_GOAL;
        break;
      case MoveState::REGULATE_ROTATION_AT_GOAL:
        regulateRotation(cmd_vel.get(), final_yaw);
        if (abs(final_yaw) < angular_properties_.tolerance)
        {
          debouncing_counter_++;
          if (debouncing_counter_ >= debouncing_counter_max_)
          {
            stopRobot();
            return nav2_behaviors::Status::SUCCEEDED;
            debouncing_counter_ = 0;
          }
        }
        else
        {
          debouncing_counter_ = 0;
        }
        break;
      }

      // Stop if there is a collision
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
          RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting MoveBehavior");
          return nav2_behaviors::Status::FAILED;
        }
      }

      this->vel_pub_->publish(std::move(cmd_vel));
      return nav2_behaviors::Status::RUNNING;
    }

  protected:
    void initializeRotation(double diff_yaw)
    {
      if (rotation_ruckig_ != nullptr)
        delete rotation_ruckig_;

      rotation_ruckig_ = new ruckig::Ruckig<1>{1.0 / cycle_frequency_};
      rotation_ruckig_input_.max_velocity = {angular_properties_.max_velocity};
      rotation_ruckig_input_.max_acceleration = {angular_properties_.max_acceleration};
      rotation_ruckig_input_.max_jerk = {99999999999.0};
      rotation_ruckig_input_.target_position = {0};
      rotation_ruckig_input_.current_position = {diff_yaw};
      rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
      rotation_ruckig_output_.new_position = {diff_yaw};
      rotation_ruckig_output_.new_velocity = {0.0};
      rotation_ruckig_output_.new_acceleration = {0.0};
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
    }

    void regulateRotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw)
    {
      if (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) != ruckig::Finished)
        rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
      const double error_yaw = diff_yaw - rotation_ruckig_output_.new_position[0];
      cmd_vel->angular.z = angular_properties_.kp * error_yaw;
    }

    void initializeTranslation(double diff_x, double diff_y)
    {
      if (translation_ruckig_ != nullptr)
        delete translation_ruckig_;

      translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / cycle_frequency_};
      translation_ruckig_input_.max_velocity = {linear_properties_.max_velocity};
      translation_ruckig_input_.max_acceleration = {linear_properties_.max_acceleration};
      translation_ruckig_input_.max_jerk = {99999999999.0};
      translation_ruckig_input_.target_position = {0};
      translation_ruckig_input_.current_position = {diff_x};
      translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
      translation_ruckig_output_.new_position = {diff_x};
      translation_ruckig_output_.new_velocity = {0.0};
      translation_ruckig_output_.new_acceleration = {0.0};
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
    }

    void regulateTranslation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
    {
      if (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) != ruckig::Finished)
        translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
      const double error_x = diff_x - translation_ruckig_output_.new_position[0];
      cmd_vel->linear.x = linear_properties_.kp * error_x;
      cmd_vel->angular.z = diff_y;
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

      double debouncing_duration;
      nav2_util::declare_parameter_if_not_declared(
          node,
          "debouncing_duration", rclcpp::ParameterValue(0.05));
      node->get_parameter("debouncing_duration", debouncing_duration);
      debouncing_counter_max_ = static_cast<int>(debouncing_duration * cycle_frequency_);
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

    typename ActionT::Feedback::SharedPtr feedback_;

    std::string global_frame_;
    std::string odom_frame_;
    tf2::Transform tf_odom_target_;
    bool ignore_obstacles_;

    mep3_msgs::msg::MotionProperties linear_properties_;
    mep3_msgs::msg::MotionProperties angular_properties_;

    rclcpp::Duration timeout_{0, 0};
    rclcpp::Time end_time_;
    double simulate_ahead_time_;

    int debouncing_counter_;
    int debouncing_counter_max_;

    ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
    ruckig::InputParameter<1> rotation_ruckig_input_;
    ruckig::OutputParameter<1> rotation_ruckig_output_;

    ruckig::Ruckig<1> *translation_ruckig_{nullptr};
    ruckig::InputParameter<1> translation_ruckig_input_;
    ruckig::OutputParameter<1> translation_ruckig_output_;

    MoveState state_;
    uint8_t type_;
  };

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_
