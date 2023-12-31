#include "move.hpp"

#define sign(x) (((x) > 0) - ((x) < 0))

namespace mep3_navigation
{
  void Move::on_command_received(const mep3_msgs::msg::MoveCommand::SharedPtr msg)
  {
    if (state_ == mep3_msgs::msg::MoveState::STATE_IDLE)
    {
      init_move(msg);
      return;
    }

    command_->target = msg->target;
    command_->header.stamp = msg->header.stamp;

    update_odom_target_tf();
  }

  void Move::on_action()
  {
    auto result = std::make_shared<mep3_msgs::action::Move::Result>();
    auto goal = action_server_->get_current_goal();

    if (state_ != mep3_msgs::msg::MoveState::STATE_IDLE)
    {
      action_server_->terminate_current(result);
      return;
    }

    mep3_msgs::msg::MoveCommand::SharedPtr msg = std::make_shared<mep3_msgs::msg::MoveCommand>();
    msg->header = goal->header;
    msg->odom_frame = goal->odom_frame;
    msg->target = goal->target;
    msg->linear_properties = goal->linear_properties;
    msg->angular_properties = goal->angular_properties;
    msg->ignore_obstacles = goal->ignore_obstacles;
    msg->timeout = goal->timeout;
    msg->reversing = goal->reversing;
    msg->mode = goal->mode;

    init_move(msg);

    while (rclcpp::ok() && state_ != mep3_msgs::msg::MoveState::STATE_IDLE)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    result->error = state_msg_.error;
    if (state_msg_.error == mep3_msgs::msg::MoveState::ERROR_NONE)
      action_server_->succeeded_current(result);
    else
      action_server_->terminate_current(result);
  }

  bool Move::update_odom_target_tf()
  {
    tf2::Transform tf_global_target;
    tf_global_target.setOrigin(tf2::Vector3(command_->target.x, command_->target.y, 0.0));
    tf_global_target.setRotation(tf2::Quaternion(
        tf2::Vector3(0, 0, 1), command_->target.theta));

    geometry_msgs::msg::TransformStamped tf_global_odom_message;
    try
    {
      tf_global_odom_message = tf_->lookupTransform(command_->header.frame_id, command_->odom_frame, command_->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR(get_logger(), "Initial global_frame -> command_->odom_frame is not available.");

      state_msg_.error = mep3_msgs::msg::MoveState::ERROR_MISSING_TRANSFORM;
      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      state_pub_->publish(state_msg_);

      return false;
    }

    tf2::Transform tf_global_odom;
    tf2::convert(tf_global_odom_message.transform, tf_global_odom);
    tf_odom_target_ = tf_global_odom.inverse() * tf_global_target;
    target_updated_ = true;

    // Reset multiturn
    multiturn_n_ = 0;
    use_multiturn_ = false;
    previous_yaw_ = tf2::getYaw(tf_global_target.getRotation());
    return true;
  }

  bool Move::init_move(const mep3_msgs::msg::MoveCommand::SharedPtr command)
  {
    command_ = command;
    end_time_ = command_->timeout + now();

    // Apply defaults
    if (command_->header.frame_id == "")
      command_->header.frame_id = "map";
    if (command_->odom_frame == "")
      command_->odom_frame = "odom";
    if (command_->linear_properties.max_velocity == 0.0)
      command_->linear_properties.max_velocity = default_command_->linear_properties.max_velocity;
    if (command_->linear_properties.max_acceleration == 0.0)
      command_->linear_properties.max_acceleration = default_command_->linear_properties.max_acceleration;
    if (command_->linear_properties.kp == 0.0)
      command_->linear_properties.kp = default_command_->linear_properties.kp;
    if (command_->linear_properties.kd == 0.0)
      command_->linear_properties.kd = default_command_->linear_properties.kd;
    if (command_->linear_properties.tolerance == 0.0)
      command_->linear_properties.tolerance = default_command_->linear_properties.tolerance;
    if (command_->angular_properties.max_velocity == 0.0)
      command_->angular_properties.max_velocity = default_command_->angular_properties.max_velocity;
    if (command_->angular_properties.max_acceleration == 0.0)
      command_->angular_properties.max_acceleration = default_command_->angular_properties.max_acceleration;
    if (command_->angular_properties.kp == 0.0)
      command_->angular_properties.kp = default_command_->angular_properties.kp;
    if (command_->angular_properties.kd == 0.0)
      command_->angular_properties.kd = default_command_->angular_properties.kd;
    if (command_->angular_properties.tolerance == 0.0)
      command_->angular_properties.tolerance = default_command_->angular_properties.tolerance;

    if (!update_odom_target_tf())
      return false;

    // Kickoff FSM
    lock_tf_odom_base_ = false;
    if (command_->mode & mep3_msgs::msg::MoveCommand::MODE_ROTATE_TOWARDS_GOAL)
    {
      state_ = mep3_msgs::msg::MoveState::STATE_ROTATING_TOWARDS_GOAL;
      return true;
    }
    if (command_->mode & mep3_msgs::msg::MoveCommand::MODE_TRANSLATE)
    {
      state_ = mep3_msgs::msg::MoveState::STATE_TRANSLATING;
      return true;
    }
    if (command_->mode & mep3_msgs::msg::MoveCommand::MODE_ROTATE_AT_GOAL)
    {
      state_ = mep3_msgs::msg::MoveState::STATE_ROTATING_AT_GOAL;

      // Multiturn. We allow multiturn only if the goal is in the base frame.
      if (command_->mode == mep3_msgs::msg::MoveCommand::MODE_ROTATE_AT_GOAL && command_->header.frame_id == "base_link") {
        if (command->target.theta > M_PI)
          multiturn_n_ = (command->target.theta + M_PI) / (2 * M_PI);
        else if (command->target.theta < -M_PI)
          multiturn_n_ = (command->target.theta - M_PI) / (2 * M_PI);
        use_multiturn_ = true;
      }
      return true;
    }
    RCLCPP_ERROR(get_logger(), "Invalid MoveCommand, at least one of rotate_towards_goal, translate, or rotate_at_goal must be true.");
    return false;
  }

  double Move::get_diff_final_orientation(const tf2::Transform &tf_base_target)
  {
    const double final_yaw_raw = tf2::getYaw(tf_base_target.getRotation());
    if (use_multiturn_)
    {
      if (final_yaw_raw - previous_yaw_ > M_PI)
        multiturn_n_--;
      else if (final_yaw_raw - previous_yaw_ < -M_PI)
        multiturn_n_++;
    }
    previous_yaw_ = final_yaw_raw;
    return final_yaw_raw + multiturn_n_ * 2 * M_PI;
  }

  double Move::get_diff_heading(const tf2::Transform &tf_base_target)
  {
    if (command_->reversing == mep3_msgs::msg::MoveCommand::REVERSING_AUTO)
    {
      const double diff_yaw_back = atan2(-tf_base_target.getOrigin().y(), -tf_base_target.getOrigin().x());
      const double diff_yaw_forward = atan2(tf_base_target.getOrigin().y(), tf_base_target.getOrigin().x());
      return (abs(diff_yaw_back) < abs(diff_yaw_forward)) ? diff_yaw_back : diff_yaw_forward;
    }
    if (command_->reversing == mep3_msgs::msg::MoveCommand::REVERSING_FORCE)
      return atan2(-tf_base_target.getOrigin().y(), -tf_base_target.getOrigin().x());
    return atan2(tf_base_target.getOrigin().y(), tf_base_target.getOrigin().x());
  }

  double Move::get_distance(const tf2::Transform &tf_base_target)
  {
    return sqrt(tf_base_target.getOrigin().x() * tf_base_target.getOrigin().x() + tf_base_target.getOrigin().y() * tf_base_target.getOrigin().y());
  }

  void Move::state_rotating_towards_goal(const tf2::Transform &tf_base_target, const tf2::Transform &tf_odom_base, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    const double diff_yaw = get_diff_heading(tf_base_target);

    if (should_init)
    {
      debouncing_reset();
      const double distance_to_goal = get_distance(tf_base_target);
      if (distance_to_goal < command_->linear_properties.tolerance)
      {
        // In case we are already at the goal we skip rotation towards the goal and translation.
        state_ = mep3_msgs::msg::MoveState::STATE_ROTATING_AT_GOAL;
        return;
      }
      else
      {
        // TODO: Parametrize this threshold
        if (distance_to_goal < 0.15)
        {
          // When a robot is very close to the goal we cannot use atan2(diff_y, diff_x) as the goal shifts during the rotation.
          lock_tf_odom_base_ = true;
          locked_tf_odom_base_ = tf_odom_base;
        }
        init_rotation(diff_yaw);
      }
    }

    regulate_rotation(cmd_vel, diff_yaw);
    if (abs(diff_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        stop_robot();
        lock_tf_odom_base_ = false;
        state_ = mep3_msgs::msg::MoveState::STATE_TRANSLATING;
        debouncing_reset();
      }
      return;
    }
    debouncing_reset();
  }

  void Move::stop_robot()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
  }

  void Move::state_translating(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    if (should_init)
    {
      debouncing_reset();
      init_translation(tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    }

    regulate_translation(cmd_vel, tf_base_target.getOrigin().x(), tf_base_target.getOrigin().y());
    if (abs(tf_base_target.getOrigin().x()) < command_->linear_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        stop_robot();
        if (command_->mode & mep3_msgs::msg::MoveCommand::MODE_ROTATE_AT_GOAL)
        {
          state_ = mep3_msgs::msg::MoveState::STATE_ROTATING_AT_GOAL;
          debouncing_reset();
        }
        else
        {
          state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
          return;
        }
      }
      return;
    }
    debouncing_reset();
  }

  void Move::state_rotating_at_goal(const tf2::Transform &tf_base_target, geometry_msgs::msg::Twist *cmd_vel)
  {
    const bool should_init = (state_ != previous_state_);
    const double final_yaw = get_diff_final_orientation(tf_base_target);

    if (should_init)
    {
      debouncing_reset();
      init_rotation(final_yaw);
    }

    regulate_rotation(cmd_vel, final_yaw);
    if (abs(final_yaw) < command_->angular_properties.tolerance)
    {
      if (now() >= debouncing_end_)
      {
        stop_robot();
        debouncing_reset();
        state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      }
      return;
    }
    debouncing_reset();
  }

  void Move::update()
  {
    if (state_ == mep3_msgs::msg::MoveState::STATE_IDLE)
    {
      previous_state_ = state_;
      return;
    }

    const uint8_t previous_state = state_;

    // Timeout
    rclcpp::Duration time_remaining = end_time_ - now();
    if (time_remaining.seconds() < 0.0 && rclcpp::Duration(command_->timeout).seconds() > 0.0)
    {
      stop_robot();
      RCLCPP_WARN(
          get_logger(),
          "Exceeded time allowance before reaching the Move goal - Exiting Move");
      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      return;
    }

    // Target in the base frame
    geometry_msgs::msg::PoseStamped tf_odom_base_message;
    if (!nav2_util::getCurrentPose(
            tf_odom_base_message, *tf_, command_->odom_frame, robot_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(get_logger(), "Initial odom_frame -> base frame is not available.");
      state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
      return;
    }
    tf2::Transform tf_odom_base;
    tf2::convert(tf_odom_base_message.pose, tf_odom_base);
    if (lock_tf_odom_base_)
    {
      tf_odom_base.getOrigin().setX(locked_tf_odom_base_.getOrigin().x());
      tf_odom_base.getOrigin().setY(locked_tf_odom_base_.getOrigin().y());
    }
    tf2::Transform tf_base_target = tf_odom_base.inverse() * tf_odom_target_;

    // FSM
    state_msg_.error = mep3_msgs::msg::MoveState::ERROR_NONE;
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    switch (state_)
    {
    case mep3_msgs::msg::MoveState::STATE_ROTATING_TOWARDS_GOAL:
      state_rotating_towards_goal(tf_base_target, tf_odom_base, cmd_vel.get());
      break;
    case mep3_msgs::msg::MoveState::STATE_TRANSLATING:
      state_translating(tf_base_target, cmd_vel.get());
      break;
    case mep3_msgs::msg::MoveState::STATE_ROTATING_AT_GOAL:
      state_rotating_at_goal(tf_base_target, cmd_vel.get());
      break;
    }

    // Stop if there is a collision
    if (!command_->ignore_obstacles)
    {
      geometry_msgs::msg::PoseStamped current_pose;
      if (!nav2_util::getCurrentPose(
              current_pose, *tf_, odom_frame_, robot_frame_,
              transform_tolerance_))
      {
        RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
        state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
        return;
      }

      geometry_msgs::msg::Pose2D pose2d;
      pose2d.x = current_pose.pose.position.x;
      pose2d.y = current_pose.pose.position.y;
      pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

      const double stopping_distance = stopping_distance_ + (cmd_vel->linear.x * cmd_vel->linear.x) / (2 * command_->linear_properties.max_acceleration);
      const double sim_position_change = sign(cmd_vel->linear.x) * stopping_distance;
      pose2d.x += sim_position_change * cos(pose2d.theta);
      pose2d.y += sim_position_change * sin(pose2d.theta);

      bool is_collision_ahead = false;
      try {
        const double score = collision_checker_->scorePose(pose2d);
        if (score >= 254)
          is_collision_ahead = true;
      } catch (const std::exception& e) {
        RCLCPP_ERROR_ONCE(get_logger(), "Collision checker failed: %s", e.what());
      }

      if (is_collision_ahead)
      {
        stop_robot();
        RCLCPP_WARN(get_logger(), "Collision Ahead - Exiting Move");

        state_msg_.error = mep3_msgs::msg::MoveState::ERROR_OBSTACLE;

        state_ = mep3_msgs::msg::MoveState::STATE_IDLE;
        update_state_msg(tf_base_target);
        state_pub_->publish(state_msg_);
        return;
      }
    }

    cmd_vel_pub_->publish(std::move(cmd_vel));

    update_state_msg(tf_base_target);
    state_pub_->publish(state_msg_);

    previous_state_ = previous_state;
  }

  void Move::update_state_msg(tf2::Transform &tf_base_target)
  {
    state_msg_.state = state_;
    state_msg_.distance_xy = get_distance(tf_base_target);
    state_msg_.distance_x = tf_base_target.getOrigin().x();
    state_msg_.distance_yaw = get_diff_final_orientation(tf_base_target);
  }

  void Move::init_rotation(double diff_yaw)
  {
    if (rotation_ruckig_ != nullptr)
      delete rotation_ruckig_;

    rotation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    rotation_ruckig_input_.max_velocity = {command_->angular_properties.max_velocity};
    rotation_ruckig_input_.max_acceleration = {command_->angular_properties.max_acceleration};
    rotation_ruckig_input_.max_jerk = {99999999999.0};
    rotation_ruckig_input_.target_position = {0};
    rotation_ruckig_input_.current_position = {diff_yaw};
    rotation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_);

    last_error_yaw_ = 0;
  }

  void Move::regulate_rotation(geometry_msgs::msg::Twist *cmd_vel, double diff_yaw)
  {
    if (target_updated_)
    {
      double prev = rotation_ruckig_input_.current_position[0];
      rotation_ruckig_input_.current_position[0] = diff_yaw - last_error_yaw_;
      target_updated_ = false;
    }

    const double previous_input = rotation_ruckig_output_.new_position[0];
    const bool is_trajectory_finished = (rotation_ruckig_->update(rotation_ruckig_input_, rotation_ruckig_output_) == ruckig::Finished);
    last_error_yaw_ = diff_yaw - rotation_ruckig_output_.new_position[0];
    const double d_input = rotation_ruckig_output_.new_position[0] - previous_input;
    cmd_vel->angular.z = command_->angular_properties.kp * last_error_yaw_ - command_->angular_properties.kd * d_input;

    if (!is_trajectory_finished)
      rotation_ruckig_output_.pass_to_input(rotation_ruckig_input_);
  }

  void Move::init_translation(double diff_x, double diff_y)
  {
    if (translation_ruckig_ != nullptr)
      delete translation_ruckig_;

    translation_ruckig_ = new ruckig::Ruckig<1>{1.0 / update_rate_};
    translation_ruckig_input_.max_velocity = {command_->linear_properties.max_velocity};
    translation_ruckig_input_.max_acceleration = {command_->linear_properties.max_acceleration};
    translation_ruckig_input_.max_jerk = {99999999999.0};
    translation_ruckig_input_.target_position = {0};
    translation_ruckig_input_.current_position = {diff_x};
    translation_ruckig_input_.control_interface = ruckig::ControlInterface::Position;
    translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_);

    last_error_x_ = 0;
    last_error_y_ = 0;
  }

  void Move::regulate_translation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
  {
    if (target_updated_)
    {
      double prev = translation_ruckig_input_.current_position[0];
      translation_ruckig_input_.current_position[0] = diff_x - last_error_x_;
      RCLCPP_INFO(get_logger(), "diff_x: %f, last_error_x_: %f", diff_x, last_error_x_);
      RCLCPP_INFO(get_logger(), "prev: %f, current: %f", prev, translation_ruckig_input_.current_position[0]);
      target_updated_ = false;
    }

    const double previous_input = translation_ruckig_output_.new_position[0];
    const bool is_trajectory_finished = (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) == ruckig::Finished);
    last_error_x_ = diff_x - translation_ruckig_output_.new_position[0];
    const double d_input = translation_ruckig_output_.new_position[0] - previous_input;
    cmd_vel->linear.x = command_->linear_properties.kp * last_error_x_ - command_->linear_properties.kd * d_input;

    // TODO: Parameterize this + add kd
    cmd_vel->angular.z = diff_y * cmd_vel->linear.x * 1.0;

    if (!is_trajectory_finished)
      translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
  }

  void Move::init() {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    command_sub_ = create_subscription<mep3_msgs::msg::MoveCommand>(
        "~/command", 1, std::bind(&Move::on_command_received, this, std::placeholders::_1));
    state_pub_ = create_publisher<mep3_msgs::msg::MoveState>("~/state", 1);
    action_server_ = std::make_shared<nav2_util::SimpleActionServer<mep3_msgs::action::Move>>(
        this, "~/move", std::bind(&Move::on_action, this));
    action_server_->activate();

    tf_ =
        std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_);

    costmap_sub_ = std::make_shared<nav2_costmap_2d::CostmapSubscriber>(
        shared_from_this(), "local_costmap/costmap_raw");
    footprint_sub_ = std::make_shared<nav2_costmap_2d::FootprintSubscriber>(
        shared_from_this(), "local_costmap/published_footprint", *tf_, robot_frame_, transform_tolerance_);
    collision_checker_ =
        std::make_shared<nav2_costmap_2d::CostmapTopicCollisionChecker>(
            *costmap_sub_, *footprint_sub_, get_name());
  }

  Move::Move(std::string name) : Node(name)
  {
    // Read parameters
    declare_parameter("update_rate", rclcpp::ParameterValue(50));
    get_parameter("update_rate", update_rate_);

    double command_timeout;
    declare_parameter("command_timeout", rclcpp::ParameterValue(0.5));
    get_parameter("command_timeout", command_timeout);
    command_timeout_ = rclcpp::Duration::from_seconds(command_timeout);

    double debouncing_duration;
    declare_parameter("debouncing_duration", rclcpp::ParameterValue(0.3));
    get_parameter("debouncing_duration", debouncing_duration);
    debouncing_duration_ = rclcpp::Duration::from_seconds(debouncing_duration);

    declare_parameter("stopping_distance", rclcpp::ParameterValue(0.2));
    get_parameter("stopping_distance", stopping_distance_);

    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.5));
    get_parameter("transform_tolerance", transform_tolerance_);

    declare_parameter("robot_frame", rclcpp::ParameterValue(std::string("base_link")));
    get_parameter("robot_frame", robot_frame_);

    declare_parameter("odom_frame", rclcpp::ParameterValue(std::string("odom")));
    get_parameter("odom_frame", odom_frame_);

    // Linear
    declare_parameter("linear.kp", rclcpp::ParameterValue(3.0));
    get_parameter("linear.kp", default_command_->linear_properties.kp);

    declare_parameter("linear.kd", rclcpp::ParameterValue(0.0));
    get_parameter("linear.kd", default_command_->linear_properties.kd);

    declare_parameter("linear.max_velocity", rclcpp::ParameterValue(0.1));
    get_parameter("linear.max_velocity", default_command_->linear_properties.max_velocity);

    declare_parameter("linear.max_acceleration", rclcpp::ParameterValue(1.5));
    get_parameter("linear.max_acceleration", default_command_->linear_properties.max_acceleration);

    declare_parameter("linear.tolerance", rclcpp::ParameterValue(0.01));
    get_parameter("linear.tolerance", default_command_->linear_properties.tolerance);

    // Angular
    declare_parameter("angular.kp", rclcpp::ParameterValue(5.0));
    get_parameter("angular.kp", default_command_->angular_properties.kp);

    declare_parameter("angular.kd", rclcpp::ParameterValue(0.0));
    get_parameter("angular.kd", default_command_->angular_properties.kd);

    declare_parameter("angular.max_velocity", rclcpp::ParameterValue(0.5));
    get_parameter("angular.max_velocity", default_command_->angular_properties.max_velocity);

    declare_parameter("angular.max_acceleration", rclcpp::ParameterValue(0.1));
    get_parameter("angular.max_acceleration", default_command_->angular_properties.max_acceleration);

    declare_parameter("angular.tolerance", rclcpp::ParameterValue(0.03));
    get_parameter("angular.tolerance", default_command_->angular_properties.tolerance);
  }

  void Move::debouncing_reset()
  {
    debouncing_end_ = now() + debouncing_duration_;
  }
};
