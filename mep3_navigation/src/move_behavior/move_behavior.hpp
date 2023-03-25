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
#include "mep3_navigation/stuck_detector.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))
#define min(x, y) (((x) < (y)) ? (x) : (y))

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
      command_global_frame_ = command->header.frame_id;
      odom_frame_ = command->odom_frame;
      ignore_obstacles_ = command->ignore_obstacles;
      timeout_ = command->timeout;
      end_time_ = steady_clock_.now() + timeout_;
      linear_properties_ = command->linear_properties;
      angular_properties_ = command->angular_properties;
      type_ = command->type;

      // Apply defaults
      if (command_global_frame_ == "")
        command_global_frame_ = "map";
      if (odom_frame_ == "")
        odom_frame_ = "odom";
      if (linear_properties_.max_velocity == 0.0)
        linear_properties_.max_velocity = default_linear_properties_.max_velocity;
      if (linear_properties_.max_acceleration == 0.0)
        linear_properties_.max_acceleration = default_linear_properties_.max_acceleration;
      if (linear_properties_.kp == 0.0)
        linear_properties_.kp = default_linear_properties_.kp;
      if (linear_properties_.kd == 0.0)
        linear_properties_.kd = default_linear_properties_.kd;
      if (linear_properties_.tolerance == 0.0)
        linear_properties_.tolerance = default_linear_properties_.tolerance;
      if (angular_properties_.max_velocity == 0.0)
        angular_properties_.max_velocity = default_angular_properties_.max_velocity;
      if (angular_properties_.max_acceleration == 0.0)
        angular_properties_.max_acceleration = default_angular_properties_.max_acceleration;
      if (angular_properties_.kp == 0.0)
        angular_properties_.kp = default_angular_properties_.kp;
      if (angular_properties_.kd == 0.0)
        angular_properties_.kd = default_angular_properties_.kd;
      if (angular_properties_.tolerance == 0.0)
        angular_properties_.tolerance = default_angular_properties_.tolerance;

      // Target in the global frame
      tf2::Transform tf_global_target;
      tf_global_target.setOrigin(tf2::Vector3(command->target.x, command->target.y, 0.0));
      tf_global_target.setRotation(tf2::Quaternion(
          tf2::Vector3(0, 0, 1), command->target.theta));

      geometry_msgs::msg::PoseStamped tf_global_odom_message;
      if (!nav2_util::getCurrentPose(
              tf_global_odom_message, *this->tf_, command_global_frame_, odom_frame_,
              this->transform_tolerance_))
      {
        RCLCPP_ERROR(this->logger_, "Initial global_frame -> odom_frame_ is not available.");
        return nav2_behaviors::Status::FAILED;
      }
      tf2::Transform tf_global_odom;
      tf2::convert(tf_global_odom_message.pose, tf_global_odom);
      tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;

      // Reset multiturn
      multiturn_n_ = 0;
      use_multiturn_ = false;
      previous_yaw_ = tf2::getYaw(tf_global_target.getRotation());

      // Kickoff FSM
      switch (type_)
      {
      case mep3_msgs::action::Move::Goal::TYPE_ROTATE:
        state_ = MoveState::INITIALIZE_ROTATION_AT_GOAL;

        // Multiturn support
        if (command_global_frame_ == "base_link") {
          if (command->target.theta > M_PI)
            multiturn_n_ = (command->target.theta + M_PI) / (2 * M_PI);
          else if (command->target.theta < -M_PI)
            multiturn_n_ = (command->target.theta - M_PI) / (2 * M_PI);
          use_multiturn_ = true;
        }
        break;
      case mep3_msgs::action::Move::Goal::TYPE_TRANSLATE:
        state_ = MoveState::INITIALIZE_TRANSLATION;
        break;
      case mep3_msgs::action::Move::Goal::TYPE_FULL_NO_REVERSING:
      case mep3_msgs::action::Move::Goal::TYPE_FULL_AUTO_REVERSING:
      case mep3_msgs::action::Move::Goal::TYPE_FULL_FORCE_REVERSING:
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

      const double final_yaw_raw = tf2::getYaw(tf_base_target.getRotation());
      if (use_multiturn_)
        if (final_yaw_raw - previous_yaw_ > M_PI)
          multiturn_n_--;
        else if (final_yaw_raw - previous_yaw_ < -M_PI)
          multiturn_n_++;
      previous_yaw_ = final_yaw_raw;
      const double final_yaw = final_yaw_raw + multiturn_n_ * 2 * M_PI;
      const double diff_x = tf_base_target.getOrigin().x();
      const double diff_y = tf_base_target.getOrigin().y();

      double diff_yaw = 0;
      if (type_ == mep3_msgs::action::Move::Goal::TYPE_FULL_AUTO_REVERSING)
      {
        const double diff_yaw_back = atan2(-diff_y, -diff_x);
        const double diff_yaw_forward = atan2(diff_y, diff_x);
        diff_yaw = (abs(diff_yaw_back) < abs(diff_yaw_forward)) ? diff_yaw_back : diff_yaw_forward;
      }
      else if (type_ == mep3_msgs::action::Move::Goal::TYPE_FULL_FORCE_REVERSING)
      {
        diff_yaw = atan2(-diff_y, -diff_x);
      }
      else
      {
        diff_yaw = atan2(diff_y, diff_x);
      }

      // FSM
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

        const double sim_position_change = sign(cmd_vel->linear.x) * simulate_ahead_distance_;
        pose2d.x += sim_position_change * cos(pose2d.theta);
        pose2d.y += sim_position_change * sin(pose2d.theta);
        if (!collision_checker_->isCollisionFree(pose2d))
        {
          stopRobot();
          RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting MoveBehavior");
          return nav2_behaviors::Status::FAILED;
        }
      }

      // Stop if the robot is stuck
      if (stuck_detector_->is_stuck())
      {
        stopRobot();
        stuck_detector_->softstop();
        RCLCPP_WARN(this->logger_, "Robot is stuck - Exiting MoveBehavior");
        return nav2_behaviors::Status::FAILED;
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
      rotation_last_input_ = rotation_ruckig_output_.new_position[0];
    }

    void regulateRotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw)
    {
      if (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) != ruckig::Finished)
        rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
      const double error_yaw = diff_yaw - rotation_ruckig_output_.new_position[0];
      const double d_input = rotation_ruckig_output_.new_position[0] - rotation_last_input_;
      rotation_last_input_ = rotation_ruckig_output_.new_position[0];
      cmd_vel->angular.z = angular_properties_.kp * error_yaw - angular_properties_.kd * d_input;
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
      translation_last_input_ = translation_ruckig_output_.new_position[0];
    }

    void regulateTranslation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
    {
      if (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) != ruckig::Finished)
        translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
      const double error_x = diff_x - translation_ruckig_output_.new_position[0];
      const double d_input = translation_ruckig_output_.new_position[0] - translation_last_input_;
      translation_last_input_ = translation_ruckig_output_.new_position[0];
      cmd_vel->linear.x = linear_properties_.kp * error_x - linear_properties_.kd * d_input;
      cmd_vel->angular.z = diff_y * 0.5;
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
          "simulate_ahead_distance", rclcpp::ParameterValue(0.2));
      node->get_parameter("simulate_ahead_distance", simulate_ahead_distance_);

      double debouncing_duration;
      nav2_util::declare_parameter_if_not_declared(
          node,
          "debouncing_duration", rclcpp::ParameterValue(0.05));
      node->get_parameter("debouncing_duration", debouncing_duration);
      debouncing_counter_max_ = static_cast<int>(debouncing_duration * cycle_frequency_);

      // Linear
      nav2_util::declare_parameter_if_not_declared(
          node,
          "linear.kp", rclcpp::ParameterValue(15.0));
      node->get_parameter("linear.kp", default_linear_properties_.kp);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "linear.kd", rclcpp::ParameterValue(0.0));
      node->get_parameter("linear.kd", default_linear_properties_.kd);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "linear.max_velocity", rclcpp::ParameterValue(0.5));
      node->get_parameter("linear.max_velocity", default_linear_properties_.max_velocity);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "linear.max_acceleration", rclcpp::ParameterValue(0.5));
      node->get_parameter("linear.max_acceleration", default_linear_properties_.max_acceleration);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "linear.tolerance", rclcpp::ParameterValue(0.01));
      node->get_parameter("linear.tolerance", default_linear_properties_.tolerance);

      // Angular
      nav2_util::declare_parameter_if_not_declared(
          node,
          "angular.kp", rclcpp::ParameterValue(15.0));
      node->get_parameter("angular.kp", default_angular_properties_.kp);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "angular.kd", rclcpp::ParameterValue(0.0));
      nav2_util::declare_parameter_if_not_declared(
          node,
          "angular.max_velocity", rclcpp::ParameterValue(0.5));
      node->get_parameter("angular.max_velocity", default_angular_properties_.max_velocity);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "angular.max_acceleration", rclcpp::ParameterValue(0.5));
      node->get_parameter("angular.max_acceleration", default_angular_properties_.max_acceleration);
      nav2_util::declare_parameter_if_not_declared(
          node,
          "angular.tolerance", rclcpp::ParameterValue(0.03));
      node->get_parameter("angular.tolerance", default_angular_properties_.tolerance);

      stuck_detector_ = std::make_shared<StuckDetector>(node);
    }

    typename ActionT::Feedback::SharedPtr feedback_;

    std::string command_global_frame_;
    std::string odom_frame_;
    tf2::Transform tf_odom_target_;
    bool ignore_obstacles_;

    mep3_msgs::msg::MotionProperties linear_properties_;
    mep3_msgs::msg::MotionProperties angular_properties_;

    mep3_msgs::msg::MotionProperties default_linear_properties_;
    mep3_msgs::msg::MotionProperties default_angular_properties_;

    rclcpp::Duration timeout_{0, 0};
    rclcpp::Time end_time_;
    double simulate_ahead_distance_;

    int debouncing_counter_;
    int debouncing_counter_max_;

    ruckig::Ruckig<1> *rotation_ruckig_{nullptr};
    ruckig::InputParameter<1> rotation_ruckig_input_;
    ruckig::OutputParameter<1> rotation_ruckig_output_;
    double rotation_last_input_;
    double previous_yaw_;
    int multiturn_n_;
    bool use_multiturn_;

    ruckig::Ruckig<1> *translation_ruckig_{nullptr};
    ruckig::InputParameter<1> translation_ruckig_input_;
    ruckig::OutputParameter<1> translation_ruckig_output_;
    double translation_last_input_;

    MoveState state_;
    uint8_t type_;

    std::shared_ptr<StuckDetector> stuck_detector_;
  };

} // namespace mep3_navigation

#endif // MEP3_NAVIGATION__MOVE_BEHAVIOR_HPP_
