#include "mep3_navigation/stuck_behavior/stuck_behavior.hpp"
#include "pluginlib/class_list_macros.hpp"

mep3_navigation::StuckBehavior::StuckBehavior() :
  nav2_behaviors::TimedBehavior<ActionT>()
{
  RCLCPP_INFO(this->logger_, "STUCK BEHAVIOR CONSTRUCTOR!");
}

mep3_navigation::StuckBehavior::~StuckBehavior() = default;

nav2_behaviors::Status mep3_navigation::StuckBehavior::onRun(const std::shared_ptr<const typename ActionT::Goal> command)
{
  RCLCPP_INFO(this->logger_, "onRun succesful!");
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status mep3_navigation::StuckBehavior::onCycleUpdate()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  initializeTranslation(3, 0);
  regulateTranslation(cmd_vel.get(), 3, 0);
  this->vel_pub_->publish(std::move(cmd_vel));
  RCLCPP_WARN(this->logger_, "onCycleUpdate successful!");
  return nav2_behaviors::Status::RUNNING;
}


void mep3_navigation::StuckBehavior::initializeTranslation(double diff_x, double diff_y)
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

void mep3_navigation::StuckBehavior::regulateTranslation(geometry_msgs::msg::Twist *cmd_vel, double diff_x, double diff_y)
{
  if (translation_ruckig_->update(translation_ruckig_input_, translation_ruckig_output_) != ruckig::Finished)
    translation_ruckig_output_.pass_to_input(translation_ruckig_input_);
  const double error_x = diff_x - translation_ruckig_output_.new_position[0];
  const double d_input = translation_ruckig_output_.new_position[0] - translation_last_input_;
  translation_last_input_ = translation_ruckig_output_.new_position[0];
  cmd_vel->linear.x = linear_properties_.kp * error_x - linear_properties_.kd * d_input;
  cmd_vel->angular.z = diff_y * 0.5;
}


PLUGINLIB_EXPORT_CLASS(mep3_navigation::StuckBehavior, nav2_core::Behavior)

