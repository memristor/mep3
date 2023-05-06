#include "mep3_navigation/unstuck_behavior/unstuck_behavior.hpp"
#include "pluginlib/class_list_macros.hpp"

mep3_navigation::UnstuckBehavior::UnstuckBehavior(double linear_x, double angular_z) :
  nav2_behaviors::TimedBehavior<ActionT>(),
  state(State::NotStuck)
{
  RCLCPP_INFO(this->logger_, "UNSTUCK BEHAVIOR CONSTRUCTOR!");
}

mep3_navigation::UnstuckBehavior::~UnstuckBehavior() = default;

nav2_behaviors::Status mep3_navigation::UnstuckBehavior::onRun(const std::shared_ptr<const typename ActionT::Goal> command)
{
  ignore_obstacles_ = command->ignore_obstacles;
  timeout_ = rclcpp::Duration::from_seconds(10.0);
  end_time_ = steady_clock_.now() + timeout_;
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status mep3_navigation::UnstuckBehavior::onCycleUpdate()
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

  // Stop if there is a collision
  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_ptr = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel);
  if (!ignore_obstacles_)
    {
      // if (!nav2_util::getCurrentPose(
      //                                current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
      //                                this->transform_tolerance_))
        if (!nav2_util::getCurrentPose(
                                       current_pose, *this->tf_, odom_frame_, robot_base_frame_,
                                       this->transform_tolerance_))
        {
          RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
          return nav2_behaviors::Status::FAILED;
        }

      pose2d.x = current_pose.pose.position.x;
      pose2d.y = current_pose.pose.position.y;
      pose2d.theta = tf2::getYaw(current_pose.pose.orientation);
      RCLCPP_ERROR(this->logger_, "global_frame: %s", this->global_frame_.c_str());
      RCLCPP_ERROR(this->logger_, "robot_base_frame: %s", this->robot_base_frame_.c_str());
      RCLCPP_ERROR(this->logger_, "x: %.2f, y: %.2f", pose2d.x, pose2d.y);


      switch(state) {
      case State::NotStuck:
        {
          orig_pose2d = pose2d;
          // cmd_vel.linear.x = 0.4;
          // this->vel_pub_->publish(std::move(cmd_vel_ptr));
          if (!collision_checker_->isCollisionFree(pose2d)) state = State::FindClosest;
          else return nav2_behaviors::Status::SUCCEEDED;
          break;
        }
      case State::FindClosest:
        {
          stopRobot();
          // define how precise you need the destination position
          for (double sim_position_change = 0.01; sim_position_change < 0.2; sim_position_change += 0.01)
            {
              dest_pose2d.x = pose2d.x + sim_position_change * cos(pose2d.theta);
              dest_pose2d.y = pose2d.y + sim_position_change * sin(pose2d.theta);
              if (collision_checker_->isCollisionFree(dest_pose2d))
                {
                  pose2d = dest_pose2d;
                  cmd_vel.linear.x = 0.2;
                  break;
                }

              dest_pose2d.x = pose2d.x - sim_position_change * cos(pose2d.theta);
              dest_pose2d.y = pose2d.y - sim_position_change * sin(pose2d.theta);
              if (collision_checker_->isCollisionFree(dest_pose2d))
                {
                  pose2d = dest_pose2d;
                  cmd_vel.linear.x = -0.2;
                  break;
                }
            }
          state = State::GotoClosest;
          break;
        }
      case State::GotoClosest:
        {
          this->vel_pub_->publish(std::move(cmd_vel_ptr));
          // check if arrived to dest
          if (std::abs(orig_pose2d.x - pose2d.x) < std::abs(dest_pose2d.x - pose2d.x))
            {
              state = State::NotStuck;
            }
          break;
        }
      }
    }
  return nav2_behaviors::Status::RUNNING;
}


PLUGINLIB_EXPORT_CLASS(mep3_navigation::UnstuckBehavior, nav2_core::Behavior)

